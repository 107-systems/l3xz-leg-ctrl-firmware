/**
 * This software is distributed under the terms of the MIT License.
 * Copyright (c) 2023 LXRobotics.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz-leg-ctrl-firmware/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <limits>

#include <SPI.h>
#include <Wire.h>

#include <107-Arduino-Cyphal.h>
#include <107-Arduino-Cyphal-Support.h>

#include <107-Arduino-AS504x.h>
#include <107-Arduino-MCP2515.h>
#include <107-Arduino-littlefs.h>
#include <107-Arduino-24LCxx.hpp>

#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
#define DBG_ENABLE_DEBUG
//#define DBG_ENABLE_VERBOSE
#include <107-Arduino-Debug.hpp>

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static uint8_t const EEPROM_I2C_DEV_ADDR = 0x50;

static int const MKRCAN_MCP2515_CS_PIN  = D3;
static int const MKRCAN_MCP2515_INT_PIN = D9;
static int const AS504x_A_CS_PIN        = D4;
static int const AS504x_B_CS_PIN        = D5;
static int const LED1_PIN               = D2;
static int const BUMPER_PIN             = D6;

static uint16_t const UPDATE_PERIOD_ANGLE_ms     = 50;
static uint16_t const UPDATE_PERIOD_BUMPER_ms    = 500;
static uint16_t const UPDATE_PERIOD_HEARTBEAT_ms = 1000;

static uint32_t const WATCHDOG_DELAY_ms = 1000;

static SPISettings const MCP2515x_SPI_SETTING{10*1000*1000UL, MSBFIRST, SPI_MODE0};
static SPISettings const AS504x_SPI_SETTING  {10*1000*1000UL, MSBFIRST, SPI_MODE1};

/**************************************************************************************
 * VARIABLES
 **************************************************************************************/

DEBUG_INSTANCE(80, Serial);

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const &);
uavcan::node::ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(uavcan::node::ExecuteCommand::Request_1_1 const &);
float const a_angle_deg();
float const b_angle_deg();

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515([]()
                       {
                         SPI.beginTransaction(MCP2515x_SPI_SETTING);
                         digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
                       },
                       []()
                       {
                         digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
                         SPI.endTransaction();
                       },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr,
                       [](MCP2515::EFLG const err_flag) { DBG_ERROR("MCP2515::OnError, error code = \"%s\"", MCP2515::toStr(err_flag)); },
                       [](MCP2515::EFLG const err_flag) { DBG_ERROR("MCP2515::OnWarning, warning code = \"%s\"", MCP2515::toStr(err_flag)); });

Node::Heap<Node::DEFAULT_O1HEAP_SIZE> node_heap;
Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); });

Publisher<uavcan::node::Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<uavcan::node::Heartbeat_1_0>(1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<uavcan::si::unit::angle::Scalar_1_0> as5048a_pub;
Publisher<uavcan::si::unit::angle::Scalar_1_0> as5048b_pub;
Publisher<uavcan::primitive::scalar::Bit_1_0> bumper_pub;

ServiceServer execute_command_srv = node_hdl.create_service_server<uavcan::node::ExecuteCommand::Request_1_1, uavcan::node::ExecuteCommand::Response_1_1>(2*1000*1000UL, onExecuteCommand_1_1_Request_Received);

ArduinoAS504x angle_A_pos_sensor([]() { SPI.beginTransaction(AS504x_SPI_SETTING); },
                                 []() { SPI.endTransaction(); },
                                 []() { digitalWrite(AS504x_A_CS_PIN, LOW); },
                                 []() { digitalWrite(AS504x_A_CS_PIN, HIGH); },
                                 [](uint8_t const d) -> uint8_t { return SPI.transfer(d); },
                                 delayMicroseconds);
ArduinoAS504x angle_B_pos_sensor([]() { SPI.beginTransaction(AS504x_SPI_SETTING); },
                                 []() { SPI.endTransaction(); },
                                 []() { digitalWrite(AS504x_B_CS_PIN, LOW); },
                                 []() { digitalWrite(AS504x_B_CS_PIN, HIGH); },
                                 [](uint8_t const d) -> uint8_t { return SPI.transfer(d); },
                                 delayMicroseconds);

/* LITTLEFS/EEPROM ********************************************************************/

static EEPROM_24LCxx eeprom(EEPROM_24LCxx_Type::LC64,
                            EEPROM_I2C_DEV_ADDR,
                            [](size_t const dev_addr) { Wire.beginTransmission(dev_addr); },
                            [](uint8_t const data) { Wire.write(data); },
                            []() { return Wire.endTransmission(); },
                            [](uint8_t const dev_addr, size_t const len) -> size_t { return Wire.requestFrom(dev_addr, len); },
                            []() { return Wire.available(); },
                            []() { return Wire.read(); });

static littlefs::FilesystemConfig filesystem_config
  (
    +[](const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) -> int
    {
      eeprom.read_page((block * c->block_size) + off, (uint8_t *)buffer, size);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) -> int
    {
      eeprom.write_page((block * c->block_size) + off, (uint8_t const *)buffer, size);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c, lfs_block_t block) -> int
    {
      for(size_t off = 0; off < c->block_size; off += eeprom.page_size())
        eeprom.fill_page((block * c->block_size) + off, 0xFF);
      return LFS_ERR_OK;
    },
    +[](const struct lfs_config *c) -> int
    {
      return LFS_ERR_OK;
    },
    eeprom.page_size(),
    eeprom.page_size(),
    (eeprom.page_size() * 4), /* littlefs demands (erase) block size to exceed read/prog size. */
    eeprom.device_size() / (eeprom.page_size() * 4),
    500,
    eeprom.page_size(),
    eeprom.page_size()
  );
static littlefs::Filesystem filesystem(filesystem_config);

#if __GNUC__ >= 11
cyphal::support::platform::storage::littlefs::KeyValueStorage kv_storage(filesystem);
#endif /* __GNUC__ >= 11 */

/* REGISTER ***************************************************************************/

static uint16_t     node_id            = std::numeric_limits<uint16_t>::max();
static CanardPortID port_id_as5048_a   = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_as5048_b   = std::numeric_limits<CanardPortID>::max();
static CanardPortID port_id_bumper     = std::numeric_limits<CanardPortID>::max();
static float        a_angle_offset_deg = 0.0f;
static float        b_angle_offset_deg = 0.0f;


#if __GNUC__ >= 11

const auto node_registry = node_hdl.create_registry();

const auto reg_rw_cyphal_node_id              = node_registry->expose("cyphal.node.id", {true}, node_id);
const auto reg_ro_cyphal_node_description     = node_registry->route ("cyphal.node.description", {true}, []() { return  "L3X-Z LEG_CONTROLLER"; });
const auto reg_rw_cyphal_pub_AS5048_a_id      = node_registry->expose("cyphal.pub.AS5048_A.id", {true}, port_id_as5048_a);
const auto reg_ro_cyphal_pub_AS5048_a_type    = node_registry->route ("cyphal.pub.AS5048_A.type", {true}, []() { return "uavcan.primitive.scalar.Real32.1.0"; });
const auto reg_rw_cyphal_pub_AS5048_b_id      = node_registry->expose("cyphal.pub.AS5048_B.id", {true}, port_id_as5048_b);
const auto reg_ro_cyphal_pub_AS5048_b_type    = node_registry->route ("cyphal.pub.AS5048_B.type", {true}, []() { return "uavcan.primitive.scalar.Real32.1.0"; });
const auto reg_rw_cyphal_pub_bumper_id        = node_registry->expose("cyphal.pub.bumper.id", {true}, port_id_bumper);
const auto reg_ro_cyphal_pub_bumper_type      = node_registry->route ("cyphal.pub.bumper.type", {true}, []() { return "uavcan.primitive.scalar.Bit.1.0"; });
const auto reg_rw_cyphal_a_angle_offset_deg   = node_registry->expose("cyphal.l3xz-leg-ctrl.angle_offset_deg.a", {true}, a_angle_offset_deg);
const auto reg_rw_cyphal_b_angle_offset_deg   = node_registry->expose("cyphal.l3xz-leg-ctrl.angle_offset_deg.b", {true}, b_angle_offset_deg);

#endif /* __GNUC__ >= 11 */

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  // while(!Serial) { } /* only for debug */
  delay(1000);

  Debug.prettyPrintOn(); /* Enable pretty printing on a shell. */

  /* LITTLEFS/EEPROM ********************************************************************/
  Wire.begin();
  Wire.setClock(400*1000UL); /* Set fast mode. */

  if (!eeprom.isConnected()) {
    DBG_ERROR("Connecting to EEPROM failed.");
    return;
  }
  Serial.println(eeprom);

  if (auto const err_mount = filesystem.mount(); err_mount.has_value()) {
    DBG_ERROR("Mounting failed with error code %d", static_cast<int>(err_mount.value()));
    (void)filesystem.format();
  }

  if (auto const err_mount = filesystem.mount(); err_mount.has_value()) {
    DBG_ERROR("Mounting failed again with error code %d", static_cast<int>(err_mount.value()));
    return;
  }

#if __GNUC__ >= 11
  auto const rc_load = cyphal::support::load(kv_storage, *node_registry);
  if (rc_load.has_value()) {
    DBG_ERROR("cyphal::support::load failed with %d", static_cast<int>(rc_load.value()));
    return;
  }
#endif /* __GNUC__ >= 11 */

  (void)filesystem.unmount();

  /* If the node ID contained in the register points to an undefined
   * node ID, assign node ID 0 to this node.
   */
  if (node_id > CANARD_NODE_ID_MAX)
    node_id = 0;
  node_hdl.setNodeId(static_cast<CanardNodeID>(node_id));
  /* Update/create all objects which depend on persistently stored
   * register values.
   */
  if (port_id_as5048_a != std::numeric_limits<CanardPortID>::max())
    as5048a_pub = node_hdl.create_publisher<uavcan::si::unit::angle::Scalar_1_0>(port_id_as5048_a, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_as5048_b != std::numeric_limits<CanardPortID>::max())
    as5048b_pub = node_hdl.create_publisher<uavcan::si::unit::angle::Scalar_1_0>(port_id_as5048_b, 1*1000*1000UL /* = 1 sec in usecs. */);
  if (port_id_bumper != std::numeric_limits<CanardPortID>::max())
    bumper_pub = node_hdl.create_publisher<uavcan::primitive::scalar::Bit_1_0>  (port_id_bumper,   1*1000*1000UL /* = 1 sec in usecs. */);

  DBG_INFO("Node ID: %d\n\r\tAS5048 A ID = %d\n\r\tAS5048 B ID = %d\n\r\tBUMPER   ID = %d",
           node_id, port_id_as5048_a, port_id_as5048_b, port_id_bumper);
  DBG_INFO("\tANG. OFF. A = %0.2f\n\r\tANG. OFF. B = %0.2f",
           a_angle_offset_deg, b_angle_offset_deg);


  /* NODE INFO ************************************************************************/
  static const auto node_info = node_hdl.create_node_info
  (
    /* uavcan.node.Version.1.0 protocol_version */
    1, 0,
    /* uavcan.node.Version.1.0 hardware_version */
    1, 0,
    /* uavcan.node.Version.1.0 software_version */
    0, 1,
    /* saturated uint64 software_vcs_revision_id */
#ifdef CYPHAL_NODE_INFO_GIT_VERSION
    CYPHAL_NODE_INFO_GIT_VERSION,
#else
    0,
#endif
    /* saturated uint8[16] unique_id */
    cyphal::support::UniqueId::instance().value(),
    /* saturated uint8[<=50] name */
    "107-systems.l3xz-leg-ctrl");

  /* Setup LED pins and initialize */
  pinMode(LED1_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);

  /* Setup bumper pin. */
  pinMode(BUMPER_PIN, INPUT_PULLUP);

  /* Setup SPI access */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* set AS504x pins */
  pinMode(AS504x_A_CS_PIN, OUTPUT);
  digitalWrite(AS504x_A_CS_PIN, HIGH);
  pinMode(AS504x_B_CS_PIN, OUTPUT);
  digitalWrite(AS504x_B_CS_PIN, HIGH);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);

  CanardFilter const CAN_FILTER_SERVICES = canardMakeFilterForServices(node_id);
  DBG_INFO("CAN Filter #1\n\r\tExt. Mask : %8X\n\r\tExt. ID   : %8X",
           CAN_FILTER_SERVICES.extended_mask,
           CAN_FILTER_SERVICES.extended_can_id);

  /* Only pass service requests/responses for this node ID through to receive buffer #0. */
  uint32_t const RXMB0_MASK = CAN_FILTER_SERVICES.extended_mask;
  size_t const RXMB0_FILTER_SIZE = 2;
  uint32_t const RXMB0_FILTER[RXMB0_FILTER_SIZE] =
  {
    MCP2515::CAN_EFF_BITMASK | CAN_FILTER_SERVICES.extended_can_id,
    MCP2515::CAN_EFF_BITMASK | 0
  };
  mcp2515.enableFilter(MCP2515::RxB::RxB0, RXMB0_MASK, RXMB0_FILTER, RXMB0_FILTER_SIZE);

  /* Only pass messages with ID 0 to receive buffer #1 (filtering out most). */
  uint32_t const RXMB1_MASK = 0x01FFFFFF;
  size_t const RXMB1_FILTER_SIZE = 4;
  uint32_t const RXMB1_FILTER[RXMB1_FILTER_SIZE] =
  {
    MCP2515::CAN_EFF_BITMASK | 0,
    MCP2515::CAN_EFF_BITMASK | 0,
    MCP2515::CAN_EFF_BITMASK | 0,
    MCP2515::CAN_EFF_BITMASK | 0
  };
  mcp2515.enableFilter(MCP2515::RxB::RxB1, RXMB1_MASK, RXMB1_FILTER, RXMB1_FILTER_SIZE);

  /* Leave configuration and enable MCP2515. */
  mcp2515.setNormalMode();

  /* Enable watchdog. */
  rp2040.wdt_begin(WATCHDOG_DELAY_ms);
  rp2040.wdt_reset();

  DBG_INFO("init complete.");
}

void loop()
{
  /* Deal with all pending events of the MCP2515 -
   * signaled by the INT pin being driven LOW.
   */
  while (digitalRead(MKRCAN_MCP2515_INT_PIN) == LOW)
    mcp2515.onExternalEventHandler();

  /* Process all pending Cyphal actions.
   */
  node_hdl.spinSome();

  /* Publish all the gathered data, although at various
   * different intervals.
   */
  static unsigned long prev_heartbeat = 0;
  static unsigned long prev_bumper = 0;
  static unsigned long prev_angle_sensor = 0;

  unsigned long const now = millis();


  if((now - prev_heartbeat) > UPDATE_PERIOD_BUMPER_ms)
  {
    uavcan::node::Heartbeat_1_0 msg;

    msg.uptime = millis() / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);

    DBG_INFO("TX Heartbeat (uptime: %d)", msg.uptime);

    prev_heartbeat = now;

    digitalWrite(LED1_PIN, !digitalRead(LED1_PIN));
  }

  if((now - prev_bumper) > UPDATE_PERIOD_BUMPER_ms)
  {
    uavcan::primitive::scalar::Bit_1_0 uavcan_bumper;
    uavcan_bumper.value = digitalRead(BUMPER_PIN) == LOW;

    if (bumper_pub)
      bumper_pub->publish(uavcan_bumper);

    prev_bumper = now;
  }

  if((now - prev_angle_sensor) > UPDATE_PERIOD_ANGLE_ms)
  {
    uavcan::si::unit::angle::Scalar_1_0 uavcan_as5048_a;
    uavcan_as5048_a.radian = (a_angle_deg() - a_angle_offset_deg) * M_PI / 180.0f;
    if (as5048a_pub)
      as5048a_pub->publish(uavcan_as5048_a);
    DBG_VERBOSE("TX femur angle: %0.1f (raw: %0.1f, offset: %0.1f)", uavcan_as5048_a.radian * 180.0f / M_PI, a_angle_deg(), a_angle_offset_deg);


    uavcan::si::unit::angle::Scalar_1_0 uavcan_as5048_b;
    uavcan_as5048_b.radian = (b_angle_deg() - b_angle_offset_deg) * M_PI / 180.0f;
    if (as5048b_pub)
      as5048b_pub->publish(uavcan_as5048_b);
    DBG_VERBOSE("TX tibia angle: %0.1f (raw: %0.1f, offset: %0.1f)", uavcan_as5048_b.radian * 180.0f / M_PI, b_angle_deg(), b_angle_offset_deg);

    prev_angle_sensor = now;
  }

  /* Feed the watchdog only if not an async reset is
   * pending because we want to restart via yakut.
   */
  if (!cyphal::support::platform::is_async_reset_pending())
    rp2040.wdt_reset();
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame);
}

uavcan::node::ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(uavcan::node::ExecuteCommand::Request_1_1 const & req)
{
  uavcan::node::ExecuteCommand::Response_1_1 rsp;

  if (req.command == uavcan::node::ExecuteCommand::Request_1_1::COMMAND_RESTART)
  {
    if (auto const opt_err = cyphal::support::platform::reset_async(std::chrono::milliseconds(1000)); opt_err.has_value())
    {
      DBG_ERROR("reset_async failed with error code %d", static_cast<int>(opt_err.value()));
      rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == uavcan::node::ExecuteCommand::Request_1_1::COMMAND_STORE_PERSISTENT_STATES)
  {
    if (auto const err_mount = filesystem.mount(); err_mount.has_value())
    {
      DBG_ERROR("Mounting failed with error code %d", static_cast<int>(err_mount.value()));
      rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    /* Feed the watchdog. */
    rp2040.wdt_reset();
#if __GNUC__ >= 11
    auto const rc_save = cyphal::support::save(kv_storage, *node_registry, []() { rp2040.wdt_reset(); });
    if (rc_save.has_value())
    {
      DBG_ERROR("cyphal::support::save failed with %d", static_cast<int>(rc_save.value()));
      rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    /* Feed the watchdog. */
    rp2040.wdt_reset();
    rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_SUCCESS;
#endif /* __GNUC__ >= 11 */
    (void)filesystem.unmount();
    /* Feed the watchdog. */
    rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == uavcan::node::ExecuteCommand::Request_1_1::COMMAND_FACTORY_RESET)
  {
    /* erasing eeprom by writing FF in every cell */
    size_t const NUM_PAGES = eeprom.device_size() / eeprom.page_size();
    for(size_t page = 0; page < NUM_PAGES; page++)
    {
      uint16_t const page_addr = page * eeprom.page_size();
      eeprom.fill_page(page_addr, 0xFF);
      rp2040.wdt_reset();
    }

    /* Send the response. */
    rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == 0xCAFE)
  {
    /* Capture the angle offset. */
    a_angle_offset_deg = a_angle_deg();
    b_angle_offset_deg = b_angle_deg();

    DBG_INFO("onExecuteCommand_1_1_Request_Received:\n\r\toffset femur: %0.2f\n\r\toffset tibia: %0.2f",
             a_angle_offset_deg,
             b_angle_offset_deg);

    rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else {
    rsp.status = uavcan::node::ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
  }

  return rsp;
}

float const a_angle_deg()
{
  float const a_angle_raw = angle_A_pos_sensor.angle_raw();
  float const a_angle_deg = ((a_angle_raw * 360.0) / 16384.0f /* 2^14 */);
  return a_angle_deg;
}

float const b_angle_deg()
{
  float const b_angle_raw = angle_B_pos_sensor.angle_raw();
  float const b_angle_deg = ((b_angle_raw * 360.0) / 16384.0f /* 2^14 */);
  return b_angle_deg;
}
