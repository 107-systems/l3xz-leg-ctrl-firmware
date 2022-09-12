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

/**************************************************************************************
 * DEFINES
 **************************************************************************************/

#define LED1_PIN 2
#define LED2_PIN A7
#define LED3_PIN A6
#define BUMPER 6
#define ANALOG_PIN A1

#define ATSAMD21G18_SERIAL_NUMBER_WORD_0  *(volatile uint32_t*)(0x0080A00C)
#define ATSAMD21G18_SERIAL_NUMBER_WORD_1  *(volatile uint32_t*)(0x0080A040)
#define ATSAMD21G18_SERIAL_NUMBER_WORD_2  *(volatile uint32_t*)(0x0080A044)
#define ATSAMD21G18_SERIAL_NUMBER_WORD_3  *(volatile uint32_t*)(0x0080A048)

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::node;
using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

union UniqueId
{
  struct __attribute__((packed))
  {
    uint32_t w0, w1, w2, w3;
  } word_buf;
  uint8_t byte_buf[16];
};

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

UniqueId const UNIQUE_ID = []()
{
  UniqueId uid;
  uid.word_buf.w0 = ATSAMD21G18_SERIAL_NUMBER_WORD_0;
  uid.word_buf.w1 = ATSAMD21G18_SERIAL_NUMBER_WORD_1;
  uid.word_buf.w2 = ATSAMD21G18_SERIAL_NUMBER_WORD_2;
  uid.word_buf.w3 = ATSAMD21G18_SERIAL_NUMBER_WORD_3;
  return uid;
} ();

static int          const MKRCAN_MCP2515_CS_PIN  = 3;
static int          const MKRCAN_MCP2515_INT_PIN = 9;
static int          const AS504x_A_CS_PIN        = 4;
static int          const AS504x_B_CS_PIN        = 5;

static CanardPortID const ID_INPUT_VOLTAGE       = 1001U;
static CanardPortID const ID_AS5048_A            = 1002U;
static CanardPortID const ID_AS5048_B            = 1003U;
static CanardPortID const ID_BUMPER              = 1004U;
static CanardPortID const ID_LED1                = 1005U;

static SPISettings  const MCP2515x_SPI_SETTING{1000000, MSBFIRST, SPI_MODE0};
static SPISettings  const AS504x_SPI_SETTING{1000000, MSBFIRST, SPI_MODE1};

static const uavcan_node_GetInfo_Response_1_0 NODE_INFO = {
    /// uavcan.node.Version.1.0 protocol_version
    {1, 0},
    /// uavcan.node.Version.1.0 hardware_version
    {1, 0},
    /// uavcan.node.Version.1.0 software_version
    {0, 1},
    /// saturated uint64 software_vcs_revision_id
    NULL,
    /// saturated uint8[16] unique_id
    {
      UNIQUE_ID.byte_buf[ 0], UNIQUE_ID.byte_buf[ 1], UNIQUE_ID.byte_buf[ 2], UNIQUE_ID.byte_buf[ 3],
      UNIQUE_ID.byte_buf[ 4], UNIQUE_ID.byte_buf[ 5], UNIQUE_ID.byte_buf[ 6], UNIQUE_ID.byte_buf[ 7],
      UNIQUE_ID.byte_buf[ 8], UNIQUE_ID.byte_buf[ 9], UNIQUE_ID.byte_buf[10], UNIQUE_ID.byte_buf[11],
      UNIQUE_ID.byte_buf[12], UNIQUE_ID.byte_buf[13], UNIQUE_ID.byte_buf[14], UNIQUE_ID.byte_buf[15]
    },
    /// saturated uint8[<=50] name
    {
        "107-systems.l3xz-fw_leg-controller",
        strlen("107-systems.l3xz-fw_leg-controller")},
};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onReceiveBufferFull(CanardFrame const &);
void onLed1_Received (CanardRxTransfer const &, Node &);
void onGetInfo_1_0_Request_Received(CanardRxTransfer const &, Node &);

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
    Serial.println("ERROR: Can't find eeprom\nstopped...");
    while (1);
  }
  uint8_t const eeNodeID=ee.readByte(0);
  Serial.print("Node-ID from eeprom: ");
  Serial.println(eeNodeID);

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
  mcp2515.setBitRate(CanBitRate::BR_1000kBPS_16MHZ);
  mcp2515.setNormalMode();

  /* Configure initial heartbeat */
  hb.data.uptime = 0;
  hb = Heartbeat_1_0<>::Health::NOMINAL;
  hb = Heartbeat_1_0<>::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 0;

  /* Subscribe to the GetInfo request */
  node_hdl.subscribe<GetInfo_1_0::Request<>>(onGetInfo_1_0_Request_Received);
  /* Subscribe to the reception of Bit message. */
  node_hdl.subscribe<Bit_1_0<ID_LED1>>(onLed1_Received);
  Serial.println("init finished");

  /* Feed the watchdog to keep it from biting. */
  Watchdog.reset();
}

void loop()
{
  /* toggle LEDS */
  static bool flag_led=0;
  if((millis()%200)==0)
  {
    if(flag_led==0) // execute only once
    {
      if(digitalRead(LED2_PIN)==LOW)
      {
        digitalWrite(LED2_PIN, HIGH);
        digitalWrite(LED3_PIN, LOW);
      }
      else
      {
        digitalWrite(LED2_PIN, LOW);
        digitalWrite(LED3_PIN, HIGH);
      }
    }
    flag_led=1;
  }
  else flag_led=0;

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
     hb.data.uptime = millis() / 1000;
     hb = Heartbeat_1_0<>::Mode::OPERATIONAL;
     Serial.println(hb.data.uptime);
     node_hdl.publish(hb);
     prev_heartbeat = now;
   }

  if((now - prev_bumper) > 100)
  {
    Bit_1_0<ID_BUMPER> uavcan_bumper;
    uavcan_bumper.data.value = digitalRead(BUMPER);
    node_hdl.publish(uavcan_bumper);
    prev_bumper = now;
  }

  if((now - prev_angle_sensor) > 50)
  {
    float const a_angle_raw = angle_A_pos_sensor.angle_raw();
    float const a_angle_deg = (a_angle_raw * 360.0) / 16384.0f; /* 2^14 */
    Serial.println(a_angle_deg);
    Real32_1_0<ID_AS5048_A> uavcan_as5048_a;
    uavcan_as5048_a.data.value = a_angle_deg;
    node_hdl.publish(uavcan_as5048_a);

    float const b_angle_raw = angle_B_pos_sensor.angle_raw();
    float const b_angle_deg = (b_angle_raw * 360.0) / 16384.0f; /* 2^14 */
    Serial.println(b_angle_deg);
    Real32_1_0<ID_AS5048_B> uavcan_as5048_b;
    uavcan_as5048_b.data.value = b_angle_deg;
    node_hdl.publish(uavcan_as5048_b);

    prev_angle_sensor = now;
  }

  if((now - prev_battery_voltage) > (10*1000))
  {
    float const analog = analogRead(ANALOG_PIN)*3.3*11.0/1023.0;
    Serial.print("Analog Pin: ");
    Serial.println(analog);
    Real32_1_0<ID_INPUT_VOLTAGE> uavcan_input_voltage;
    uavcan_input_voltage.data.value = analog;
    node_hdl.publish(uavcan_input_voltage);
    prev_battery_voltage = now;
  }

  /* Transmit all enqeued CAN frames */
  while(node_hdl.transmitCanFrame()) { }

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

  if(uavcan_led1.data.value)
  {
    digitalWrite(LED1_PIN, HIGH);
    Serial.println("Received Bit1: true");
  }
  else
  {
    digitalWrite(LED1_PIN, LOW);
    Serial.println("Received Bit1: false");
  }
}

void onGetInfo_1_0_Request_Received(CanardRxTransfer const &transfer, Node & node_hdl)
{
  GetInfo_1_0::Response<> rsp = GetInfo_1_0::Response<>();
  memcpy(&rsp.data, &NODE_INFO, sizeof(uavcan_node_GetInfo_Response_1_0));
  node_hdl.respond(rsp, transfer.metadata.remote_node_id, transfer.metadata.transfer_id);
}
