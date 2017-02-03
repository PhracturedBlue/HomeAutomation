#include "OneWire.h"

extern OneWire oneWire;

static bool type_s;
static bool initialized = false;
static uint32_t last_time;
static float fahrenheit;
static byte addr[8];

#ifdef DEBUG_SERIAL
  #define dbg    Serial.println
  #define dbg_ln Serial.println
#else
  #define dbg    if(0) Serial.print
  #define dbg_ln if(0) Serial.println
#endif

void init_ds18b20()
{
  fahrenheit = 0.0;

  if ( !oneWire.search(addr)) {
    oneWire.reset_search();
    dbg_ln("DS18B20: Failed to find sensor");
    dbg_ln();
    return;
  }
  if (OneWire::crc8(addr, 7) != addr[7]) {
      dbg_ln("DS18B20: CRC is not valid!");
      return;
  }

  switch (addr[0]) {
    case 0x10:
      dbg_ln("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      dbg_ln("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      dbg_ln("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      dbg_ln("Device is not a DS18x20 family device.");
      return;
  } 

  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0x44, 1);        // start conversion, with parasite power on at the end
  last_time = millis();
  initialized = true;
  fahrenheit = 0.0;
}

float readTemperature()
{
  byte i;
  byte data[12];
  float celsius;
  uint32_t now = millis();
  if(now - last_time > 1000) {
      if (! initialized) {
          init_ds18b20();
          return fahrenheit;
      }
      oneWire.reset();
      oneWire.select(addr);    
      oneWire.write(0xBE);         // Read Scratchpad
      for ( i = 0; i < 9; i++) {           // we need 9 bytes
          data[i] = oneWire.read();
          dbg(data[i], HEX);
          dbg(" ");
      }
      dbg(" CRC=");
      dbg(OneWire::crc8(data, 8), HEX);
      dbg_ln();

      oneWire.reset();
      oneWire.select(addr);
      oneWire.write(0x44, 1);        // start conversion, with parasite power on at the end


      // Convert the data to actual temperature
      // because the result is a 16 bit signed integer, it should
      // be stored to an "int16_t" type, which is always 16 bits
      // even when compiled on a 32 bit processor.
      int16_t raw = (data[1] << 8) | data[0];
      if (type_s) {
          raw = raw << 3; // 9 bit resolution default
          if (data[7] == 0x10) {
              // "count remain" gives full 12 bit resolution
              raw = (raw & 0xFFF0) + 12 - data[6];
          }
      } else {
          byte cfg = (data[4] & 0x60);
          // at lower res, the low bits are undefined, so let's zero them
          if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
          else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
          else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
          //// default is 12 bit resolution, 750 ms conversion time
      }
      celsius = (float)raw / 16.0;
      fahrenheit = celsius * 1.8 + 32.0;
      dbg("  Temperature = ");
      dbg(celsius);
      dbg(" Celsius, ");
      dbg(fahrenheit);
      dbg_ln(" Fahrenheit");      
  }
  return fahrenheit;
}

