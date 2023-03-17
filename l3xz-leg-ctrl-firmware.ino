/*
 * Software for the leg controller for the L3X-Z Hexapod
 *
 * Hardware:
 *   - Arduino Nano 33 IoT
 *   - MCP2515
 * https://github.com/107-systems/l3xz-hw_leg-controller
 *
 * Used Subject-IDs
 * 1001 - pub - Real32 - input voltage
 * 1002 - pub - Real32 - AS5048-A-angle
 * 1003 - pub - Real32 - AS5048-B-angle
 * 1004 - pub - Bit    - bumper
 * 1005 - sub - Bit    - LED1
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <SPI.h>
#include <Wire.h>

#include <107-Arduino-Cyphal.h>
#include <107-Arduino-MCP2515.h>
#include <107-Arduino-AS504x.h>
#include <107-Arduino-UniqueId.h>
#include <107-Arduino-CriticalSection.h>
#include <I2C_eeprom.h>

#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
#define DBG_ENABLE_DEBUG
#include <107-Arduino-Debug.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

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

static SPISettings  const MCP2515x_SPI_SETTING{1000000, MSBFIRST, SPI_MODE0};
static SPISettings  const AS504x_SPI_SETTING{1000000, MSBFIRST, SPI_MODE1};

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
Publisher<Real32_1_0> as5048a_pub = node_hdl.create_publisher<Real32_1_0>
  (ID_AS5048_A, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Real32_1_0> as5048b_pub = node_hdl.create_publisher<Real32_1_0>
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

I2C_eeprom ee(0x50, I2C_DEVICESIZE_24LC64);

/* REGISTER ***************************************************************************/

static CanardNodeID node_id = DEFAULT_LEG_CONTROLLER_NODE_ID;
static uint16_t update_period_angle_ms = 50;
static uint16_t update_period_bumper_ms = 500;

#if __GNUC__ >= 11

Registry node_registry(node_hdl, micros);

const auto reg_rw_uavcan_node_id              = node_registry.expose("cyphal.node.id", {}, node_id);
const auto reg_ro_uavcan_node_description     = node_registry.route ("cyphal.node.description", {true}, []() { return  "L3X-Z LEG_CONTROLLER"; });
const auto reg_ro_uavcan_pub_AS5048_a_id      = node_registry.route ("cyphal.pub.AS5048_a.id", {true}, []() { return ID_AS5048_A; });
const auto reg_ro_uavcan_pub_AS5048_a_type    = node_registry.route ("cyphal.pub.AS5048_a.type", {true}, []() { return "uavcan.primitive.scalar.Real32.1.0"; });
const auto reg_ro_uavcan_pub_AS5048_b_id      = node_registry.route ("cyphal.pub.AS5048_b.id", {true}, []() { return ID_AS5048_B; });
const auto reg_ro_uavcan_pub_AS5048_b_type    = node_registry.route ("cyphal.pub.AS5048_b.type", {true}, []() { return "uavcan.primitive.scalar.Real32.1.0"; });
const auto reg_ro_uavcan_pub_bumper_id        = node_registry.route ("cyphal.pub.bumper.id", {true}, []() { return ID_BUMPER; });
const auto reg_ro_uavcan_pub_bumper_type      = node_registry.route ("cyphal.pub.bumper.type", {true}, []() { return "uavcan.primitive.scalar.Bit.1.0"; });
const auto reg_rw_aux_update_period_angle_ms  = node_registry.expose("aux.update_period_ms.angle", {}, update_period_angle_ms);
const auto reg_rw_aux_update_period_bumper_ms = node_registry.expose("aux.update_period_ms.bumper", {}, update_period_bumper_ms);

#endif /* __GNUC__ >= 11 */

/* NODE INFO **************************************************************************/

static NodeInfo node_info
(
  node_hdl,
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
  OpenCyphalUniqueId(),
  /* saturated uint8[<=50] name */
  "107-systems.l3xz-leg-ctrl"
);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  while(!Serial) { } /* only for debug */

  /* Setup LED pins and initialize */
  pinMode(LED1_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  pinMode(BUMPER_PIN, INPUT_PULLUP);

  /* Setup I2C Eeprom */
  ee.begin();
  if (! ee.isConnected())
  {
    DBG_ERROR("can't find EEPROM.");
    for(;;) { }
  }

  uint8_t const eeNodeID = ee.readByte(0);
  DBG_INFO("node-ID read from EEPROM: %d", static_cast<int>(eeNodeID));

  /* create UAVCAN class */
  node_hdl.setNodeId(eeNodeID);
  node_id = eeNodeID;

  /* Setup SPI access */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* set AS504x pins */
  pinMode(AS504x_A_CS_PIN, OUTPUT);
  digitalWrite(AS504x_A_CS_PIN, HIGH);
  pinMode(AS504x_B_CS_PIN, OUTPUT);
  digitalWrite(AS504x_B_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), []() { mcp2515.onExternalEventHandler(); }, FALLING);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();

  DBG_INFO("initialisation finished");
}

void loop()
{
  /* Process all pending OpenCyphal actions.
   */
  {
    CriticalSection crit_sec;
    node_hdl.spinSome();
  }

  /* Publish all the gathered data, although at various
   * different intervals.
   */
  static unsigned long prev_heartbeat = 0;
  static unsigned long prev_bumper = 0;
  static unsigned long prev_angle_sensor = 0;

  unsigned long const now = millis();


  if((now - prev_heartbeat) > 1000)
  {
    digitalWrite(LED1_PIN, !digitalRead(LED1_PIN));

    Heartbeat_1_0 msg;

    msg.uptime = millis() / 1000;
    msg.health.value = uavcan::node::Health_1_0::NOMINAL;
    msg.mode.value = uavcan::node::Mode_1_0::OPERATIONAL;
    msg.vendor_specific_status_code = 0;

    heartbeat_pub->publish(msg);

    DBG_INFO("TX Heartbeat (uptime: %d)", msg.uptime);

    prev_heartbeat = now;
  }

  if((now - prev_bumper) > update_period_bumper_ms)
  {
    Bit_1_0 uavcan_bumper;
    uavcan_bumper.value = digitalRead(BUMPER_PIN);
    bumper_pub->publish(uavcan_bumper);

    prev_bumper = now;
  }

  if((now - prev_angle_sensor) > update_period_angle_ms)
  {
    {
      float const a_angle_raw = angle_A_pos_sensor.angle_raw();
      a_angle_deg = ((a_angle_raw * 360.0) / 16384.0f /* 2^14 */);
    }
    Real32_1_0 uavcan_as5048_a;
    uavcan_as5048_a.value = (a_angle_deg - a_angle_offset_deg) * M_PI / 180.0f;
    as5048a_pub->publish(uavcan_as5048_a);
    DBG_INFO("TX femur angle: %0.f (offset: %0.2f)", a_angle_deg, a_angle_offset_deg);

    {
      float const b_angle_raw = angle_B_pos_sensor.angle_raw();
      b_angle_deg = ((b_angle_raw * 360.0) / 16384.0f /* 2^14 */);
    }
    Real32_1_0 uavcan_as5048_b;
    uavcan_as5048_b.value = (b_angle_deg - b_angle_offset_deg) * M_PI / 180.0f;
    as5048b_pub->publish(uavcan_as5048_b);
    DBG_INFO("TX tibia angle: %0.f (offset: %0.2f)", b_angle_deg, b_angle_offset_deg);

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

  if (req.command == 0xCAFE)
  {
    /* Capture the angle offset. */
    a_angle_offset_deg = a_angle_deg;
    b_angle_offset_deg = b_angle_deg;

    DBG_INFO("onExecuteCommand_1_1_Request_Received:\n\toffset femur: %0.2f\n\toffset tibia: %0.2f",
             a_angle_offset_deg,
             b_angle_offset_deg);

    rsp.status = ExecuteCommand::Response_1_1::STATUS_SUCCESS;
  }
  else
    rsp.status = ExecuteCommand::Response_1_1::STATUS_NOT_AUTHORIZED;

  return rsp;
}
