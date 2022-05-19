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


#include <ArduinoUAVCAN.h>
#include <ArduinoMCP2515.h>
#include <107-Arduino-AS504x.h>
#include <I2C_eeprom.h>

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

static CanardPortID const ID_INPUT_VOLTAGE       = 1001U;
static CanardPortID const ID_AS5048_A            = 1002U;
static CanardPortID const ID_AS5048_B            = 1003U;
static CanardPortID const ID_BUMPER              = 1004U;
static CanardPortID const ID_LED1                = 1005U;

static SPISettings  const MCP2515x_SPI_SETTING{1000000, MSBFIRST, SPI_MODE0};
static SPISettings  const AS504x_SPI_SETTING{1000000, MSBFIRST, SPI_MODE1};

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void onLed1_Received (CanardTransfer const &, ArduinoUAVCAN &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static ArduinoUAVCAN * uc = nullptr;

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
                       [](CanardFrame const & f) { uc->onCanFrameReceived(f); },
                       nullptr);

Heartbeat_1_0<> hb;
Bit_1_0<ID_BUMPER> uavcan_bumper;
Real32_1_0<ID_INPUT_VOLTAGE> uavcan_input_voltage;
Real32_1_0<ID_AS5048_A> uavcan_as5048_a;
Real32_1_0<ID_AS5048_B> uavcan_as5048_b;

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
  uc = new ArduinoUAVCAN(eeNodeID, [](CanardFrame const & frame) -> bool { return mcp2515.transmit(frame); });

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

  /* Configure initial values */
  uavcan_bumper.data.value = false;
  uavcan_input_voltage.data.value = 0.0;
  uavcan_as5048_a.data.value = 0.0;
  uavcan_as5048_b.data.value = 0.0;
  /* Configure initial heartbeat */
  hb.data.uptime = 0;
  hb = Heartbeat_1_0<>::Health::NOMINAL;
  hb = Heartbeat_1_0<>::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 0;

  /* Subscribe to the reception of Bit message. */
  uc->subscribe<Bit_1_0<ID_LED1>>(onLed1_Received);
  Serial.println("init finished");
}

void loop()
{
  /* check switch */
  static bool bumper_old=0;
  static bool flag_led=0;
  bool bumper_in;
  bumper_in=digitalRead(BUMPER);
  if(bumper_old!=bumper_in)
  {
    uavcan_bumper.data.value = bumper_in;
    uc->publish(uavcan_bumper);
    Serial.print("send bit: ");
    Serial.println(bumper_in);
  }
  bumper_old=bumper_in;

  /* toggle LEDS */
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
  static unsigned long prev_angle_sensor = 0;
  static unsigned long prev_battery_voltage = 0;

  unsigned long const now = millis();

  if((now - prev_heartbeat) > 1000)
  {
     hb.data.uptime = millis() / 1000;
     hb = Heartbeat_1_0<>::Mode::OPERATIONAL;
     Serial.println(hb.data.uptime);
     uc->publish(hb);
     prev_heartbeat = now;
   }

  if((now - prev_angle_sensor) > 50)
  {
    float const a_angle_raw = angle_A_pos_sensor.angle_raw();
    float const a_angle_deg = (a_angle_raw * 360.0) / 16384.0f; /* 2^14 */
    Serial.println(a_angle_deg);
    uavcan_as5048_a.data.value = a_angle_deg;
    uc->publish(uavcan_as5048_a);

    float const b_angle_raw = angle_B_pos_sensor.angle_raw();
    float const b_angle_deg = (b_angle_raw * 360.0) / 16384.0f; /* 2^14 */
    Serial.println(b_angle_deg);
    uavcan_as5048_b.data.value = b_angle_deg;
    uc->publish(uavcan_as5048_b);

    prev_angle_sensor = now;
  }

  if((now - prev_battery_voltage) > (10*1000))
  {
    float const analog = analogRead(ANALOG_PIN)*3.3*11.0/1023.0;
    Serial.print("Analog Pin: ");
    Serial.println(analog);
    uavcan_input_voltage.data.value = analog;
    uc->publish(uavcan_input_voltage);
    prev_battery_voltage = now;
  }

  /* Transmit all enqeued CAN frames */
  while(uc->transmitCanFrame()) { }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void onLed1_Received(CanardTransfer const & transfer, ArduinoUAVCAN & /* uavcan */)
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
