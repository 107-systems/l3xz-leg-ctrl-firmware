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

#include <ArduinoUAVCAN.h>
#include <ArduinoMCP2515.h>

/**************************************************************************************
 * DEFINES
 **************************************************************************************/

#define LED1 2
#define LED2 A7
#define LED3 A6
#define BUMPER 6
#define ANALOG_PIN A1

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace uavcan::primitive::scalar;

/**************************************************************************************
 * CONSTANTS
 **************************************************************************************/

static int const MKRCAN_MCP2515_CS_PIN  = 3;
static int const MKRCAN_MCP2515_INT_PIN = 7;
static CanardPortID const ID_INPUT_VOLTAGE = 1001U;
static CanardPortID const ID_AS5048_A      = 1002U;
static CanardPortID const ID_AS5048_B      = 1003U;
static CanardPortID const ID_BUMPER        = 1004U;
static CanardPortID const ID_LED1          = 1005U;

/**************************************************************************************
 * FUNCTION DECLARATION
 **************************************************************************************/

void    spi_select        ();
void    spi_deselect       ();
uint8_t spi_transfer       (uint8_t const);
void    onExternalEvent    ();
bool    transmitCanFrame   (CanardFrame const &);
void    onReceiveBufferFull(CanardFrame const &);
void    onLed1_Received (CanardTransfer const &, ArduinoUAVCAN &);

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

ArduinoMCP2515 mcp2515(spi_select,
                       spi_deselect,
                       spi_transfer,
                       micros,
                       onReceiveBufferFull,
                       nullptr);

ArduinoUAVCAN uc(15, transmitCanFrame);

Heartbeat_1_0 hb;
Bit_1_0<ID_BUMPER> uavcan_bumper;
Real32_1_0<ID_INPUT_VOLTAGE> uavcan_input_voltage;
Real32_1_0<ID_AS5048_A> uavcan_as5048_a;
Real32_1_0<ID_AS5048_B> uavcan_as5048_b;

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(9600);
//  while(!Serial) { }

  /* Setup LED pins and initialize */
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, LOW);
  pinMode(BUMPER, INPUT_PULLUP);

  /* Setup SPI access */
  SPI.begin();
  pinMode(MKRCAN_MCP2515_CS_PIN, OUTPUT);
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);

  /* Attach interrupt handler to register MCP2515 signaled by taking INT low */
  pinMode(MKRCAN_MCP2515_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MKRCAN_MCP2515_INT_PIN), onExternalEvent, FALLING);

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
  hb = Heartbeat_1_0::Health::NOMINAL;
  hb = Heartbeat_1_0::Mode::INITIALIZATION;
  hb.data.vendor_specific_status_code = 0;

  /* Subscribe to the reception of Bit message. */
  uc.subscribe<Bit_1_0<ID_LED1>>(onLed1_Received);
  Serial.println("init finished");

  /* init and check temperature sensors */
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");
}

void loop()
{
  /* check switch */
  static bool bumper_old=0;
  bool bumper_in;
  bumper_in=digitalRead(BUMPER);
  if(bumper_old!=switch_in)
  {
    uavcan_bumper.data.value = bumper_in;
    uc.publish(uavcan_bumper);   
    Serial.print("send bit: ");
    Serial.println(bumper_in); 
  }
  bumper_old=bumper_in;
  
  /* Update the heartbeat object */
  hb.data.uptime = millis() / 1000;
  hb = Heartbeat_1_0::Mode::OPERATIONAL;

  /* Publish the heartbeat once/second */
  static unsigned long prev = 0;
  unsigned long const now = millis();
  if(now - prev > 1000) {
  /* toggle bit */
    if(uavcan_toggle.data.value==true) uavcan_toggle.data.value=false;
    else uavcan_toggle.data.value=true;
    uc.publish(uavcan_toggle);
    uc.publish(hb);

  /* read AS5048_A value */
    Serial.print("Requesting AS5048 A angle...");
//    uavcan_as5048_a.data.value = a_angle;
//    uc.publish(uavcan_as5048_a);   

  /* read analog value */
    float analog=analogRead(ANALOG_PIN)/1023.0;
    Serial.print("Analog Pin: ");
    Serial.println(analog);
    uavcan_input_voltage.data.value = analog;
    uc.publish(uavcan_input_voltage);   

    prev = now;
  }

  /* Transmit all enqeued CAN frames */
  while(uc.transmitCanFrame()) { }
}

/**************************************************************************************
 * FUNCTION DEFINITION
 **************************************************************************************/

void spi_select()
{
  digitalWrite(MKRCAN_MCP2515_CS_PIN, LOW);
}

void spi_deselect()
{
  digitalWrite(MKRCAN_MCP2515_CS_PIN, HIGH);
}

uint8_t spi_transfer(uint8_t const data)
{
  return SPI.transfer(data);
}

void onExternalEvent()
{
  mcp2515.onExternalEventHandler();
}

bool transmitCanFrame(CanardFrame const & frame)
{
  return mcp2515.transmit(frame);
}

void onReceiveBufferFull(CanardFrame const & frame)
{
  uc.onCanFrameReceived(frame);
}

void onLed1_Received(CanardTransfer const & transfer, ArduinoUAVCAN & /* uavcan */)
{
  Bit_1_0<ID_LED1> const uavcan_led1 = Bit_1_0<ID_LED1>::deserialize(transfer);

  if(uavcan_led1.data.value)
  {
    digitalWrite(LED1, HIGH);
    Serial.println("Received Bit1: true");
  }
  else
  {
    digitalWrite(LED1, LOW);
    Serial.println("Received Bit1: false");
  }
}
