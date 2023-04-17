/*
 * Firmare for the leg controller for the L3X-Z Hexapod
 *
 * Hardware:
 *   - Arduino Nano RP2040 Connect
 *   - MCP2515
 * https://github.com/107-systems/l3xz-hw_leg-controller
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

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
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::si::unit;
using namespace uavcan::_register;
using namespace uavcan::primitive::scalar;


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

static CanardNodeID const DEFAULT_LEG_CONTROLLER_NODE_ID = 31;

static CanardPortID const ID_AS5048_A = 1001U;
static CanardPortID const ID_AS5048_B = 1002U;
static CanardPortID const ID_BUMPER   = 1003U;

static uint16_t UPDATE_PERIOD_ANGLE_ms  = 50;
static uint16_t UPDATE_PERIOD_BUMPER_ms = 500;

static SPISettings  const MCP2515x_SPI_SETTING{10*1000*1000UL, MSBFIRST, SPI_MODE0};
static SPISettings  const AS504x_SPI_SETTING  {10*1000*1000UL, MSBFIRST, SPI_MODE1};

/**************************************************************************************
 * VARIABLES
 **************************************************************************************/

DEBUG_INSTANCE(80, Serial);

static float a_angle_deg        = 0.0f;
static float b_angle_deg        = 0.0f;
static float a_angle_offset_deg = 0.0f;
static float b_angle_offset_deg = 0.0f;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const &);
ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const &);

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
                       nullptr);

Node::Heap<Node::DEFAULT_O1HEAP_SIZE> node_heap;
Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); }, DEFAULT_LEG_CONTROLLER_NODE_ID);

Publisher<Heartbeat_1_0> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0>
  (Heartbeat_1_0::_traits_::FixedPortId, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<angle::Scalar_1_0> as5048a_pub = node_hdl.create_publisher<angle::Scalar_1_0>
  (ID_AS5048_A, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<angle::Scalar_1_0> as5048b_pub = node_hdl.create_publisher<angle::Scalar_1_0>
  (ID_AS5048_B, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Bit_1_0> bumper_pub = node_hdl.create_publisher<Bit_1_0>
  (ID_BUMPER, 1*1000*1000UL /* = 1 sec in usecs. */);

ServiceServer execute_command_srv = node_hdl.create_service_server<ExecuteCommand::Request_1_1, ExecuteCommand::Response_1_1>(
  ExecuteCommand::Request_1_1::_traits_::FixedPortId,
  2*1000*1000UL,
  onExecuteCommand_1_1_Request_Received);

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

static CanardNodeID node_id = DEFAULT_LEG_CONTROLLER_NODE_ID;

#if __GNUC__ >= 11

const auto node_registry = node_hdl.create_registry();

const auto reg_rw_uavcan_node_id              = node_registry->expose("cyphal.node.id", {true}, node_id);
const auto reg_ro_uavcan_node_description     = node_registry->route ("cyphal.node.description", {true}, []() { return  "L3X-Z LEG_CONTROLLER"; });
const auto reg_ro_uavcan_pub_AS5048_a_id      = node_registry->route ("cyphal.pub.AS5048_a.id", {true}, []() { return ID_AS5048_A; });
const auto reg_ro_uavcan_pub_AS5048_a_type    = node_registry->route ("cyphal.pub.AS5048_a.type", {true}, []() { return "uavcan.primitive.scalar.Real32.1.0"; });
const auto reg_ro_uavcan_pub_AS5048_b_id      = node_registry->route ("cyphal.pub.AS5048_b.id", {true}, []() { return ID_AS5048_B; });
const auto reg_ro_uavcan_pub_AS5048_b_type    = node_registry->route ("cyphal.pub.AS5048_b.type", {true}, []() { return "uavcan.primitive.scalar.Real32.1.0"; });
const auto reg_ro_uavcan_pub_bumper_id        = node_registry->route ("cyphal.pub.bumper.id", {true}, []() { return ID_BUMPER; });
const auto reg_ro_uavcan_pub_bumper_type      = node_registry->route ("cyphal.pub.bumper.type", {true}, []() { return "uavcan.primitive.scalar.Bit.1.0"; });

#endif /* __GNUC__ >= 11 */

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  while(!Serial) { } /* only for debug */

  /* LITTLEFS/EEPROM ********************************************************************/
  Wire.begin();

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
  DBG_INFO("cyphal::support::load ... ");
  auto const rc_load = cyphal::support::load(kv_storage, *node_registry);
  if (rc_load.has_value()) {
    DBG_ERROR("cyphal::support::load failed with %d", static_cast<int>(rc_load.value()));
    return;
  }
  node_hdl.setNodeId(node_id); /* Update node if a different value has been loaded from the permanent storage. */
#endif /* __GNUC__ >= 11 */

  (void)filesystem.unmount();

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
  mcp2515.setNormalMode();

  DBG_INFO("initialisation finished");
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


  if((now - prev_heartbeat) > 1000)
  {
    Heartbeat_1_0 msg;

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
    Bit_1_0 uavcan_bumper;
    uavcan_bumper.value = digitalRead(BUMPER_PIN);
    bumper_pub->publish(uavcan_bumper);

    prev_bumper = now;
  }

  if((now - prev_angle_sensor) > UPDATE_PERIOD_ANGLE_ms)
  {
    {
      float const a_angle_raw = angle_A_pos_sensor.angle_raw();
      a_angle_deg = ((a_angle_raw * 360.0) / 16384.0f /* 2^14 */);
    }
    angle::Scalar_1_0 uavcan_as5048_a;
    uavcan_as5048_a.radian = (a_angle_deg - a_angle_offset_deg) * M_PI / 180.0f;
    as5048a_pub->publish(uavcan_as5048_a);
    DBG_VERBOSE("TX femur angle: %0.1f (offset: %0.1f)", a_angle_deg, a_angle_offset_deg);

    {
      float const b_angle_raw = angle_B_pos_sensor.angle_raw();
      b_angle_deg = ((b_angle_raw * 360.0) / 16384.0f /* 2^14 */);
    }
    angle::Scalar_1_0 uavcan_as5048_b;
    uavcan_as5048_b.radian = (b_angle_deg - b_angle_offset_deg) * M_PI / 180.0f;
    as5048b_pub->publish(uavcan_as5048_b);
    DBG_VERBOSE("TX tibia angle: %0.1f (offset: %0.1f)", b_angle_deg, b_angle_offset_deg);

    prev_angle_sensor = now;
  }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame);
}

ExecuteCommand::Response_1_1 onExecuteCommand_1_1_Request_Received(ExecuteCommand::Request_1_1 const & req)
{
  ExecuteCommand::Response_1_1 rsp;

  if (req.command == ExecuteCommand::Request_1_1::COMMAND_RESTART)
  {
    if (auto const opt_err = cyphal::support::platform::reset_async(std::chrono::milliseconds(1000)); opt_err.has_value())
    {
      DBG_ERROR("reset_async failed with error code %d", static_cast<int>(opt_err.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == ExecuteCommand::Request_1_1::COMMAND_STORE_PERSISTENT_STATES)
  {
    if (auto const err_mount = filesystem.mount(); err_mount.has_value())
    {
      DBG_ERROR("Mounting failed with error code %d", static_cast<int>(err_mount.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
#if __GNUC__ >= 11
    auto const rc_save = cyphal::support::save(kv_storage, *node_registry);
    if (rc_save.has_value())
    {
      DBG_ERROR("cyphal::support::save failed with %d", static_cast<int>(rc_save.value()));
      rsp.status = ExecuteCommand::Response_1_1::STATUS_FAILURE;
      return rsp;
    }
     rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
#endif /* __GNUC__ >= 11 */
    (void)filesystem.unmount();
    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else if (req.command == 0xCAFE)
  {
    /* Capture the angle offset. */
    a_angle_offset_deg = a_angle_deg;
    b_angle_offset_deg = b_angle_deg;

    DBG_INFO("onExecuteCommand_1_1_Request_Received:\n\toffset femur: %0.1f\n\toffset tibia: %0.1f",
             a_angle_offset_deg,
             b_angle_offset_deg);

    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else {
    rsp.status = ExecuteCommand::Response_1_1::STATUS_BAD_COMMAND;
  }

  return rsp;
}
