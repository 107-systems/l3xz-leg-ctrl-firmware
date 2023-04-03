/*
 * "Erases" the whole EEPROM by writing 0xFF into every byte
 * of every page. This may be necessary if the EEPROM is used
 * with a flash file system such as littlefs because those
 * usually depend on the the fact that a erased flash cell
 * holds the value of 0xFF.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Wire.h>

#include <107-Arduino-24LCxx.hpp>

/**************************************************************************************
 * GLOBAL CONSTANTS
 **************************************************************************************/

static uint8_t const EEPROM_I2C_DEV_ADDR = 0x50;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

EEPROM_24LCxx eeprom(EEPROM_24LCxx_Type::LC64,
                     EEPROM_I2C_DEV_ADDR,
                     [](size_t const dev_addr) { Wire.beginTransmission(dev_addr); },
                     [](uint8_t const data) { Wire.write(data); },
                     []() { return Wire.endTransmission(); },
                     [](uint8_t const dev_addr, size_t const len) -> size_t { return Wire.requestFrom(dev_addr, len); },
                     []() { return Wire.available(); },
                     []() { return Wire.read(); });

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin(115200);
  while (!Serial) { }
  delay(1000);

  Wire.begin();

  Serial.println(eeprom);

  Serial.print("Connect to EEPROM ... ");
  if (!eeprom.isConnected()) {
    Serial.println("ERROR");
    return;
  }
  Serial.println("OK");

  Serial.println("Erasing EEPROM ...... ");
  size_t const NUM_PAGES = eeprom.device_size() / eeprom.page_size();
  for(size_t page = 0; page < NUM_PAGES; page++)
  {
    uint16_t const page_addr = page * eeprom.page_size();

    char msg[64] = {0};
    snprintf(msg, sizeof(msg), "Erasing %d of %d ... ", page, NUM_PAGES);
    Serial.println(msg);

    eeprom.fill_page(page_addr, 0xFF);
  }
  Serial.println("Erasing EEPROM ...... OK");
}

void loop()
{

}
