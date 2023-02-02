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

#undef max
#undef min
#include <algorithm>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const MKRCAN_MCP2515_CS_PIN  = 3;
static int const MKRCAN_MCP2515_INT_PIN = 9;
static int const AS504x_A_CS_PIN        = 4;
static int const AS504x_B_CS_PIN        = 5;
static int const LED1_PIN               = 2;
static int const BUMPER_PIN             = 6;

static CanardNodeID const DEFAULT_LEG_CONTROLLER_NODE_ID = 31;

static CanardPortID const ID_INPUT_VOLTAGE       = 1001U;
static CanardPortID const ID_AS5048_A            = 1002U;
static CanardPortID const ID_AS5048_B            = 1003U;
static CanardPortID const ID_BUMPER              = 1004U;
static CanardPortID const ID_LED1                = 1005U;

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
void onLed1_Received (Bit_1_0<ID_LED1> const & uavcan_led1);
ExecuteCommand_1_1::Response<> onExecuteCommand_1_1_Request_Received(ExecuteCommand_1_1::Request<> const &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515([]()
                       {
                         noInterrupts();
                         SPI.beginTransaction(MCP2515x_SPI_SETTING);
                         digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
                       },
                       []()
                       {
                         digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
                         SPI.endTransaction();
                         interrupts();
                       },
                       [](uint8_t const d) { return SPI.transfer(d); },
                       micros,
                       onReceiveBufferFull,
                       nullptr);

Node::Heap<Node::DEFAULT_O1HEAP_SIZE> node_heap;
Node node_hdl(node_heap.data(), node_heap.size(), micros, [] (CanardFrame const & frame) { return mcp2515.transmit(frame); }, DEFAULT_LEG_CONTROLLER_NODE_ID);

Publisher<Heartbeat_1_0<>> heartbeat_pub = node_hdl.create_publisher<Heartbeat_1_0<>>
  (Heartbeat_1_0<>::PORT_ID, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Real32_1_0<ID_INPUT_VOLTAGE>> input_voltage_pub = node_hdl.create_publisher<Real32_1_0<ID_INPUT_VOLTAGE>>
  (ID_INPUT_VOLTAGE, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Real32_1_0<ID_AS5048_A>> as5048a_pub = node_hdl.create_publisher<Real32_1_0<ID_AS5048_A>>
  (ID_AS5048_A, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Real32_1_0<ID_AS5048_B>> as5048b_pub = node_hdl.create_publisher<Real32_1_0<ID_AS5048_B>>
  (ID_AS5048_B, 1*1000*1000UL /* = 1 sec in usecs. */);
Publisher<Bit_1_0<ID_BUMPER>> bumper_pub = node_hdl.create_publisher<Bit_1_0<ID_BUMPER>>
  (ID_BUMPER, 1*1000*1000UL /* = 1 sec in usecs. */);

Subscription led_sub = node_hdl.create_subscription<Bit_1_0<ID_LED1>>(
  Bit_1_0<ID_LED1>::PORT_ID, CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC, onLed1_Received);

ServiceServer execute_command_srv = node_hdl.create_service_server<ExecuteCommand_1_1::Request<>, ExecuteCommand_1_1::Response<>>(
  ExecuteCommand_1_1::Request<>::PORT_ID,
  2*1000*1000UL,
  onExecuteCommand_1_1_Request_Received);

ArduinoAS504x angle_A_pos_sensor([]()
                                 {
                                   noInterrupts();
                                   SPI.beginTransaction(AS504x_SPI_SETTING);
                                 },
                                 []()
                                 {
                                  SPI.endTransaction();
                                  interrupts();
                                 },
                                 [](){ digitalWrite(AS504x_A_CS_PIN, LOW); },
                                 [](){ digitalWrite(AS504x_A_CS_PIN, HIGH); },
                                 [](uint8_t const d) -> uint8_t { return SPI.transfer(d); },
                                 delayMicroseconds);
ArduinoAS504x angle_B_pos_sensor([]()
                                 {
                                   noInterrupts();
                                   SPI.beginTransaction(AS504x_SPI_SETTING);
                                 },
                                 []()
                                 {
                                  SPI.endTransaction();
                                  interrupts();
                                 },
                                 [](){ digitalWrite(AS504x_B_CS_PIN, LOW); },
                                 [](){ digitalWrite(AS504x_B_CS_PIN, HIGH); },
                                 [](uint8_t const d) -> uint8_t { return SPI.transfer(d); },
                                 delayMicroseconds);

I2C_eeprom ee(0x50, I2C_DEVICESIZE_24LC64);

static uint16_t update_period_vbat_ms   = 3000;
static uint16_t update_period_angle_ms  =   50;
static uint16_t update_period_bumper_ms =  500;

/* REGISTER ***************************************************************************/

static RegisterNatural8  reg_rw_uavcan_node_id             ("uavcan.node.id",              Register::Access::ReadWrite, Register::Persistent::No, DEFAULT_LEG_CONTROLLER_NODE_ID, [&node_hdl](uint8_t const reg_val) { node_hdl.setNodeId(reg_val); });
static RegisterString    reg_ro_uavcan_node_description    ("uavcan.node.description",     Register::Access::ReadWrite, Register::Persistent::No, "L3X-Z LEG_CONTROLLER");
static RegisterNatural16 reg_ro_uavcan_pub_AS5048_a_id     ("uavcan.pub.AS5048_a.id",      Register::Access::ReadOnly,  Register::Persistent::No, ID_AS5048_A);
static RegisterString    reg_ro_uavcan_pub_AS5048_a_type   ("uavcan.pub.AS5048_a.type",    Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Real32.1.0");
static RegisterNatural16 reg_ro_uavcan_pub_AS5048_b_id     ("uavcan.pub.AS5048_b.id",      Register::Access::ReadOnly,  Register::Persistent::No, ID_AS5048_B);
static RegisterString    reg_ro_uavcan_pub_AS5048_b_type   ("uavcan.pub.AS5048_b.type",    Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Real32.1.0");
static RegisterNatural16 reg_ro_uavcan_pub_bumper_id       ("uavcan.pub.bumper.id",        Register::Access::ReadOnly,  Register::Persistent::No, ID_BUMPER);
static RegisterString    reg_ro_uavcan_pub_bumper_type     ("uavcan.pub.bumper.type",      Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0");
static RegisterNatural16 reg_ro_uavcan_sub_led1_id         ("uavcan.sub.led1.id",          Register::Access::ReadOnly,  Register::Persistent::No, ID_LED1);
static RegisterString    reg_ro_uavcan_sub_led1_type       ("uavcan.sub.led1.type",        Register::Access::ReadOnly,  Register::Persistent::No, "uavcan.primitive.scalar.Bit.1.0");
static RegisterNatural16 reg_rw_aux_update_period_angle_ms ("aux.update_period_ms.angle",  Register::Access::ReadWrite, Register::Persistent::No, update_period_angle_ms,        nullptr, nullptr, [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(50)); });
static RegisterNatural16 reg_rw_aux_update_period_bumper_ms("aux.update_period_ms.bumper", Register::Access::ReadWrite, Register::Persistent::No, update_period_bumper_ms,       nullptr, nullptr, [](uint16_t const & val) { return std::min(val, static_cast<uint16_t>(100)); });
static RegisterList      reg_list(node_hdl);

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
  NULL,
  /* saturated uint8[16] unique_id */
  OpenCyphalUniqueId(),
  /* saturated uint8[<=50] name */
  "107-systems.l3xz-leg-ctrl"
);

Heartbeat_1_0<> hb_msg;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  //while(!Serial) { } /* only for debug */

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
  reg_rw_uavcan_node_id.set(eeNodeID);

  /* Setup SPI access */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* set AS504x pins */
  pinMode(AS504x_A_CS_PIN, OUTPUT);
  digitalWrite(AS504x_A_CS_PIN, LOW);
  pinMode(AS504x_B_CS_PIN, OUTPUT);
  digitalWrite(AS504x_B_CS_PIN, LOW);


  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), []() { mcp2515.onExternalEventHandler(); }, FALLING);

  /* Initialize MCP2515 */
  mcp2515.begin();
  mcp2515.setBitRate(CanBitRate::BR_250kBPS_16MHZ);
  mcp2515.setNormalMode();

  /* Configure initial heartbeat */
  hb_msg.data.uptime = 0;
  hb_msg.data.health.value = uavcan_node_Health_1_0_NOMINAL;
  hb_msg.data.mode.value = uavcan_node_Mode_1_0_INITIALIZATION;
  hb_msg.data.vendor_specific_status_code = 0;

  /* Register callbacks for node info and register api.
   */
  reg_list.add(reg_rw_uavcan_node_id);
  reg_list.add(reg_ro_uavcan_node_description);
  reg_list.add(reg_ro_uavcan_pub_AS5048_a_id);
  reg_list.add(reg_ro_uavcan_pub_AS5048_b_id);
  reg_list.add(reg_ro_uavcan_pub_bumper_id);
  reg_list.add(reg_ro_uavcan_sub_led1_id);
  reg_list.add(reg_ro_uavcan_pub_AS5048_a_type);
  reg_list.add(reg_ro_uavcan_pub_AS5048_b_type);
  reg_list.add(reg_ro_uavcan_pub_bumper_type);
  reg_list.add(reg_ro_uavcan_sub_led1_type);
  reg_list.add(reg_rw_aux_update_period_angle_ms);
  reg_list.add(reg_rw_aux_update_period_bumper_ms);

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
  static unsigned long prev_battery_voltage = 0;

  unsigned long const now = millis();


  if((now - prev_heartbeat) > 1000)
  {
     hb_msg.data.uptime = millis() / 1000;
     hb_msg.data.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;
     heartbeat_pub->publish(hb_msg);
     DBG_INFO("TX Heartbeat (uptime: %d)", hb_msg.data.uptime);

     prev_heartbeat = now;
   }

  if((now - prev_bumper) > update_period_bumper_ms)
  {
    Bit_1_0<ID_BUMPER> uavcan_bumper;
    uavcan_bumper.data.value = digitalRead(BUMPER_PIN);
    bumper_pub->publish(uavcan_bumper);

    prev_bumper = now;
  }

  if((now - prev_angle_sensor) > update_period_angle_ms)
  {
    float const a_angle_raw = angle_A_pos_sensor.angle_raw();
    a_angle_deg = ((a_angle_raw * 360.0) / 16384.0f /* 2^14 */);
    Real32_1_0<ID_AS5048_A> uavcan_as5048_a;
    uavcan_as5048_a.data.value = a_angle_deg - a_angle_offset_deg;
    as5048a_pub->publish(uavcan_as5048_a);
    DBG_INFO("TX femur angle: %0.f (offset: %0.2f)", a_angle_deg, a_angle_offset_deg);

    float const b_angle_raw = angle_B_pos_sensor.angle_raw();
    b_angle_deg = ((b_angle_raw * 360.0) / 16384.0f /* 2^14 */);
    Real32_1_0<ID_AS5048_B> uavcan_as5048_b;
    uavcan_as5048_b.data.value = b_angle_deg - b_angle_offset_deg;
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

void onLed1_Received(Bit_1_0<ID_LED1> const & uavcan_led1)
{
  DBG_INFO("onLed1_Received: %d", uavcan_led1.data.value);

  if(uavcan_led1.data.value)
    digitalWrite(LED1_PIN, HIGH);
  else
    digitalWrite(LED1_PIN, LOW);
}

ExecuteCommand_1_1::Response<> onExecuteCommand_1_1_Request_Received(ExecuteCommand_1_1::Request<> const & req)
{
  ExecuteCommand_1_1::Response<> rsp;

  rsp.data.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_NOT_AUTHORIZED;

  if (req.data.command == 0xCAFE)
  {
    /* Capture the angle offset. */
    a_angle_offset_deg = a_angle_deg;
    b_angle_offset_deg = b_angle_deg;

    DBG_INFO("onExecuteCommand_1_0_Request_Received:\n\toffset femur: %0.2f\n\toffset tibia: %0.2f", a_angle_offset_deg, b_angle_offset_deg);

    rsp.data.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_SUCCESS;
  }

  return rsp;
}