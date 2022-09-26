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
#include <I2C_eeprom.h>
#include <Adafruit_SleepyDog.h>

#define DBG_ENABLE_ERROR
#define DBG_ENABLE_WARNING
#define DBG_ENABLE_INFO
#define DBG_ENABLE_DEBUG
#include <107-Arduino-Debug.hpp>

#include "NodeInfo.h"

/**************************************************************************************
 * DEFINES
 **************************************************************************************/

#define LED1_PIN 2
#define LED2_PIN A7
#define LED3_PIN A6
#define BUMPER 6
#define ANALOG_PIN A1

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int          const MKRCAN_MCP2515_CS_PIN  = 3;
static int          const MKRCAN_MCP2515_INT_PIN = 9;
static int          const AS504x_A_CS_PIN        = 4;
static int          const AS504x_B_CS_PIN        = 5;

static CanardNodeID const LEG_CONTROLLER_NODE_ID = 31;

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
void onLed1_Received (CanardRxTransfer const &, Node &);

/* Cyphal Service Requests */
void onList_1_0_Request_Received(CanardRxTransfer const &, Node &);
void onGetInfo_1_0_Request_Received(CanardRxTransfer const &, Node &);
void onAccess_1_0_Request_Received(CanardRxTransfer const &, Node &);
void onExecuteCommand_1_0_Request_Received(CanardRxTransfer const &, Node &);

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

Node node_hdl([](CanardFrame const & frame) -> bool { return mcp2515.transmit(frame); });

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

static uint16_t updateinterval_inputvoltage=3*1000;
static uint16_t updateinterval_angle=50;
static uint16_t updateinterval_bumper=500;

/* REGISTER ***************************************************************************/

static RegisterNatural8  reg_rw_uavcan_node_id                 ("uavcan.node.id",                 Register::Access::ReadWrite, LEG_CONTROLLER_NODE_ID,              [&node_hdl](RegisterNatural8 const & reg) { node_hdl.setNodeId(reg.get()); });
static RegisterString    reg_ro_uavcan_node_description        ("uavcan.node.description",        Register::Access::ReadWrite, "L3X-Z LEG_CONTROLLER",              nullptr);
static RegisterNatural16 reg_ro_uavcan_pub_inputvoltage_id     ("uavcan.pub.inputvoltage.id",     Register::Access::ReadOnly,  ID_INPUT_VOLTAGE,                    nullptr);
static RegisterString    reg_ro_uavcan_pub_inputvoltage_type   ("uavcan.pub.inputvoltage.type",   Register::Access::ReadOnly,  "uavcan.primitive.scalar.Real32.1.0",nullptr);
static RegisterNatural16 reg_ro_uavcan_pub_AS5048_a_id         ("uavcan.pub.AS5048_a.id",         Register::Access::ReadOnly,  ID_AS5048_A,                         nullptr);
static RegisterString    reg_ro_uavcan_pub_AS5048_a_type       ("uavcan.pub.AS5048_a.type",       Register::Access::ReadOnly,  "uavcan.primitive.scalar.Real32.1.0",nullptr);
static RegisterNatural16 reg_ro_uavcan_pub_AS5048_b_id         ("uavcan.pub.AS5048_b.id",         Register::Access::ReadOnly,  ID_AS5048_B,                         nullptr);
static RegisterString    reg_ro_uavcan_pub_AS5048_b_type       ("uavcan.pub.AS5048_b.type",       Register::Access::ReadOnly,  "uavcan.primitive.scalar.Real32.1.0",nullptr);
static RegisterNatural16 reg_ro_uavcan_pub_bumper_id           ("uavcan.pub.bumper.id",           Register::Access::ReadOnly,  ID_BUMPER,                           nullptr);
static RegisterString    reg_ro_uavcan_pub_bumper_type         ("uavcan.pub.bumper.type",         Register::Access::ReadOnly,  "uavcan.primitive.scalar.Bit.1.0",   nullptr);
static RegisterNatural16 reg_ro_uavcan_sub_led1_id             ("uavcan.sub.led1.id",             Register::Access::ReadOnly,  ID_LED1,                             nullptr);
static RegisterString    reg_ro_uavcan_sub_led1_type           ("uavcan.sub.led1.type",           Register::Access::ReadOnly,  "uavcan.primitive.scalar.Bit.1.0",   nullptr);
static RegisterNatural16 reg_rw_aux_updateinterval_inputvoltage("aux.updateinterval.inputvoltage",Register::Access::ReadWrite, updateinterval_inputvoltage,         [&node_hdl](RegisterNatural16 const & reg) { updateinterval_inputvoltage=reg.get(); if(updateinterval_inputvoltage<100) updateinterval_inputvoltage=100; });
static RegisterNatural16 reg_rw_aux_updateinterval_angle       ("aux.updateinterval.angle",       Register::Access::ReadWrite, updateinterval_angle,                [&node_hdl](RegisterNatural16 const & reg) { updateinterval_angle=reg.get(); if(updateinterval_angle<50) updateinterval_angle=50; });
static RegisterNatural16 reg_rw_aux_updateinterval_bumper      ("aux.updateinterval.bumper",      Register::Access::ReadWrite, updateinterval_bumper,               [&node_hdl](RegisterNatural16 const & reg) { updateinterval_bumper=reg.get(); if(updateinterval_bumper<100) updateinterval_bumper=100; });
static RegisterList      reg_list;

Heartbeat_1_0<> hb;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Watchdog.enable(1000);

  Serial.begin(115200);
  //while(!Serial) { } /* only for debug */

  /* Setup LED pins and initialize */
  pinMode(LED1_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED2_PIN, LOW);
  pinMode(LED3_PIN, OUTPUT);
  digitalWrite(LED3_PIN, LOW);
  pinMode(BUMPER, INPUT_PULLUP);

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
  hb.data.uptime = 0;
  hb = Heartbeat_1_0<>::Health::NOMINAL;
  hb = Heartbeat_1_0<>::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 0;

  /* Subscribe to the GetInfo request */
  node_hdl.subscribe<GetInfo_1_0::Request<>>(onGetInfo_1_0_Request_Received);
  reg_list.subscribe(node_hdl);
  reg_list.add(reg_rw_uavcan_node_id);
  reg_list.add(reg_ro_uavcan_node_description);
  reg_list.add(reg_ro_uavcan_pub_inputvoltage_id);
  reg_list.add(reg_ro_uavcan_pub_AS5048_a_id);
  reg_list.add(reg_ro_uavcan_pub_AS5048_b_id);
  reg_list.add(reg_ro_uavcan_pub_bumper_id);
  reg_list.add(reg_ro_uavcan_sub_led1_id);
  reg_list.add(reg_ro_uavcan_pub_inputvoltage_type);
  reg_list.add(reg_ro_uavcan_pub_AS5048_a_type);
  reg_list.add(reg_ro_uavcan_pub_AS5048_b_type);
  reg_list.add(reg_ro_uavcan_pub_bumper_type);
  reg_list.add(reg_ro_uavcan_sub_led1_type);
  reg_list.add(reg_rw_aux_updateinterval_inputvoltage);
  reg_list.add(reg_rw_aux_updateinterval_angle);
  reg_list.add(reg_rw_aux_updateinterval_bumper);
  /* Subscribe to the reception of Bit message. */
  node_hdl.subscribe<Bit_1_0<ID_LED1>>(onLed1_Received);
  /* Subscribe to incoming service requests */
  node_hdl.subscribe<ExecuteCommand_1_0::Request<>>(onExecuteCommand_1_0_Request_Received);

  DBG_INFO("initialisation finished");

  /* Feed the watchdog to keep it from biting. */
  Watchdog.reset();
}

void loop()
{
  /* Process all pending OpenCyphal actions.
   */
  node_hdl.spinSome();

  /* Publish all the gathered data, although at various
   * different intervals.
   */
  static unsigned long prev_led = 0;
  static unsigned long prev_heartbeat = 0;
  static unsigned long prev_bumper = 0;
  static unsigned long prev_angle_sensor = 0;
  static unsigned long prev_battery_voltage = 0;

  unsigned long const now = millis();

  if((now - prev_led) > 200)
  {
    if (digitalRead(LED2_PIN) == LOW)
    {
      digitalWrite(LED2_PIN, HIGH);
      digitalWrite(LED3_PIN, LOW);
    }
    else
    {
      digitalWrite(LED2_PIN, LOW);
      digitalWrite(LED3_PIN, HIGH);
    }

    prev_led = now;
  }

  if((now - prev_heartbeat) > 1000)
  {
     hb.data.uptime = millis() / 1000;
     hb = Heartbeat_1_0<>::Mode::OPERATIONAL;
     node_hdl.publish(hb);
     DBG_INFO("TX Heartbeat (uptime: %d)", hb.data.uptime);

     prev_heartbeat = now;
   }

  if((now - prev_bumper) > updateinterval_bumper)
  {
    Bit_1_0<ID_BUMPER> uavcan_bumper;
    uavcan_bumper.data.value = digitalRead(BUMPER);
    node_hdl.publish(uavcan_bumper);

    prev_bumper = now;
  }

  if((now - prev_angle_sensor) > updateinterval_angle)
  {
    float const a_angle_raw = angle_A_pos_sensor.angle_raw();
    a_angle_deg = ((a_angle_raw * 360.0) / 16384.0f /* 2^14 */);
    Real32_1_0<ID_AS5048_A> uavcan_as5048_a;
    uavcan_as5048_a.data.value = a_angle_deg - a_angle_offset_deg;
    node_hdl.publish(uavcan_as5048_a);
    DBG_INFO("TX femur angle: %0.f (offset: %0.2f)", a_angle_deg, a_angle_offset_deg);

    float const b_angle_raw = angle_B_pos_sensor.angle_raw();
    b_angle_deg = ((b_angle_raw * 360.0) / 16384.0f /* 2^14 */);
    Real32_1_0<ID_AS5048_B> uavcan_as5048_b;
    uavcan_as5048_b.data.value = b_angle_deg - b_angle_offset_deg;
    node_hdl.publish(uavcan_as5048_b);
    DBG_INFO("TX tibia angle: %0.f (offset: %0.2f)", b_angle_deg, b_angle_offset_deg);

    prev_angle_sensor = now;
  }

  if((now - prev_battery_voltage) > updateinterval_inputvoltage)
  {
    float const analog = analogRead(ANALOG_PIN)*3.3*11.0/1023.0;
    Real32_1_0<ID_INPUT_VOLTAGE> uavcan_input_voltage;
    uavcan_input_voltage.data.value = analog;
    node_hdl.publish(uavcan_input_voltage);
    DBG_INFO("TX vbat: %0.2f", analog);

    prev_battery_voltage = now;
  }

  /* Feed the watchdog to keep it from biting. */
  Watchdog.reset();
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const & frame)
{
  node_hdl.onCanFrameReceived(frame, micros());
}

void onLed1_Received(CanardRxTransfer const & transfer, Node & /* node */)
{
  Bit_1_0<ID_LED1> const uavcan_led1 = Bit_1_0<ID_LED1>::deserialize(transfer);

  DBG_INFO("onLed1_Received: %d", uavcan_led1.data.value);

  if(uavcan_led1.data.value)
    digitalWrite(LED1_PIN, HIGH);
  else
    digitalWrite(LED1_PIN, LOW);
}

void onGetInfo_1_0_Request_Received(CanardRxTransfer const &transfer, Node & node_hdl)
{
  DBG_INFO("onGetInfo_1_0_Request_Received");

  GetInfo_1_0::Response<> rsp = GetInfo_1_0::Response<>();
  memcpy(&rsp.data, &NODE_INFO, sizeof(uavcan_node_GetInfo_Response_1_0));
  node_hdl.respond(rsp, transfer.metadata.remote_node_id, transfer.metadata.transfer_id);
}

void onExecuteCommand_1_0_Request_Received(CanardRxTransfer const & transfer, Node & node_hdl)
{
  ExecuteCommand_1_0::Request<> req = ExecuteCommand_1_0::Request<>::deserialize(transfer);

  if (req.data.command == 0xCAFE && transfer.metadata.remote_node_id == node_hdl.getNodeId())
  {
    /* Capture the angle offset. */
    a_angle_offset_deg = a_angle_deg;
    b_angle_offset_deg = b_angle_deg;

    DBG_INFO("onExecuteCommand_1_0_Request_Received:\n\toffset femur: %0.2f\n\toffset tibia: %0.2f", a_angle_offset_deg, b_angle_offset_deg);

    /* Send the response. */
    ExecuteCommand_1_0::Response<> rsp;
    rsp = ExecuteCommand_1_0::Response<>::Status::SUCCESS;
    node_hdl.respond(rsp, transfer.metadata.remote_node_id, transfer.metadata.transfer_id);
  }
}
