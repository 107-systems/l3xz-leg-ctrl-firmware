/*
 * Software to program the node-id on the leg controller for the L3X-Z Hexapod
 *
 * Hardware:
 *   - Arduino Nano 33 IoT
 *   - EEprom (address 0x50)
 * https://github.com/107-systems/l3xz-hw_leg-controller
 *
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/
#include <Wire.h>

#include <I2C_eeprom.h>

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

I2C_eeprom ee(0x50, I2C_DEVICESIZE_24LC64);

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  while(!Serial) { } /* only for debug */

  /* Setup I2C Eeprom */
  ee.begin();
  if (! ee.isConnected())
  {
    Serial.println("ERROR: Can't find eeprom\nstopped...");
    while (1);
  }
  uint8_t const eeNodeID_towrite=255;  //  <<<---------------- put node-ID here!!!!!!!!!!!!!!!!!!
  ee.writeByte(0,eeNodeID_towrite);
  Serial.println("Node-ID written!");

  delay(1000);

  uint8_t const eeNodeID=ee.readByte(0);
  Serial.print("Node-ID from eeprom: ");
  Serial.println(eeNodeID);

  Serial.println("init finished");
}

void loop()
{
}
