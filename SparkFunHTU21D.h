/* 
 HTU21D Humidity Sensor Library
 By: Nathan Seidle
 SparkFun Electronics
 Date: September 22nd, 2013
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 Get humidity and temperature from the HTU21D sensor.
 
 This same library should work for the other similar sensors including the Si
 
 */
 
#pragma once

#include "mbed.h"

#define HTU21D_ADDRESS 0x80  //8-bit I2C address for the sensor

#define HTU21D_ERROR       997
#define ERROR_I2C_TIMEOUT  998
#define ERROR_BAD_CRC      999

#define TRIGGER_TEMP_MEASURE_HOLD    0xE3
#define TRIGGER_HUMD_MEASURE_HOLD    0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  0xF5

#define HTU21D_SERIAL1_READ1         0xFA      //read 1-st two serial bytes
#define HTU21D_SERIAL1_READ2         0x0F      //read 2-nd two serial bytes
#define HTU21D_SERIAL2_READ1         0xFC      //read 3-rd two serial bytes
#define HTU21D_SERIAL2_READ2         0xC9      //read 4-th two serial bytes

#define HTU21D_FIRMWARE_READ1        0x84      //read firmware revision, 1-st part of the command
#define HTU21D_FIRMWARE_READ2        0xB8      //read firmware revision, 2-nd part of the command

#define HTU21D_FIRMWARE_V1           0xFF      //sensor firmware v1.0
#define HTU21D_FIRMWARE_V2           0x20      //sensor firmware 2.0

#define WRITE_USER_REG  0xE6
#define READ_USER_REG  0xE7
#define SOFT_RESET  0xFE

#define USER_REGISTER_RESOLUTION_MASK 0x81
#define USER_REGISTER_RESOLUTION_RH12_TEMP14 0x00
#define USER_REGISTER_RESOLUTION_RH8_TEMP12 0x01
#define USER_REGISTER_RESOLUTION_RH10_TEMP13 0x80
#define USER_REGISTER_RESOLUTION_RH11_TEMP11 0x81

#define USER_REGISTER_END_OF_BATTERY 0x40
#define USER_REGISTER_HEATER_ENABLED 0x04
#define USER_REGISTER_DISABLE_OTP_RELOAD 0x02

#define SI7013_CHIPID                0x0D      //device id SI7013
#define SI7020_CHIPID                0x14      //device id SI7020
#define SI7021_CHIPID                0x15      //device id SI7021
#define HTU21D_CHIPID                0x32      //device id HTU21D/SHT21 


class HTU21D {

public:
  HTU21D();
  void begin(I2C &_i2c);
  float readHumidity(void);
  float readTemperature(void);
  void setResolution(uint8_t resBits);
  uint16_t readDeviceID(void);
  uint16_t readFirmwareVersion(void);
  uint8_t readUserRegister(void);
  void writeUserRegister(uint8_t val);

private:
  uint8_t checkCRC(uint16_t message_from_sensor, uint8_t check_value_from_sensor);
  uint16_t readValue(uint8_t cmd);

  I2C *_i2c; //The generic connection to user's chosen I2C hardware
  uint8_t _buf[3];
};
