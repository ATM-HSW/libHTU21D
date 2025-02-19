/*
  HTU21D Humidity Sensor Library
  By: Nathan Seidle
  SparkFun Electronics
  Date: September 22nd, 2013
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  This library allows an Arduino to read from the HTU21D low-cost high-precision humidity sensor.

  If you have feature suggestions or need support please use the github support page: https://github.com/sparkfun/HTU21D

  Hardware Setup: The HTU21D lives on the I2C bus. Attach the SDA pin to A4, SCL to A5. If you are using the SparkFun
  breakout board you *do not* need 4.7k pull-up resistors on the bus (they are built-in).

  Link to the breakout board product:

  Software:
  Call HTU21D.Begin() in setup.
  HTU21D.ReadHumidity() will return a float containing the humidity. Ex: 54.7
  HTU21D.ReadTemperature() will return a float containing the temperature in Celsius. Ex: 24.1
  HTU21D.SetResolution(byte: 0b.76543210) sets the resolution of the readings.
  HTU21D.check_crc(message, check_value) verifies the 8-bit CRC generated by the sensor
  HTU21D.read_user_register() returns the user register. Used to set resolution.
*/

#include "mbed.h"
#include "SparkFunHTU21D.h"

HTU21D::HTU21D() {
  //Set initial values for private vars
}

//Begin
/*******************************************************************************************/
//Start I2C communication
void HTU21D::begin(I2C &i2c) {
  _i2c = &i2c; //Grab which port the user wants us to use
}

#define MAX_WAIT 100
#define DELAY_INTERVAL 10
#define MAX_COUNTER (MAX_WAIT/DELAY_INTERVAL)

//Given a command, reads a given 2-byte value with CRC from the HTU21D
uint16_t HTU21D::readValue(uint8_t cmd) {
  int ret;
  
  //Request a humidity reading
  _buf[0] = cmd;
  ret = _i2c->write(HTU21D_ADDRESS, (const char*)_buf, 1);
  
  //Hang out while measurement is taken. datasheet says 50ms, practice may call for more
  bool validResult;
  uint8_t counter;
  for (counter = 0, validResult = 0 ; counter < MAX_COUNTER && !validResult ; counter++) {
    thread_sleep_for(DELAY_INTERVAL);

    //stop the loop when i2c device is readable
    validResult = !_i2c->read(HTU21D_ADDRESS, (char*)_buf, 3);
  }

  if (!validResult) return (ERROR_I2C_TIMEOUT); //Error out

  uint16_t rawValue = ((uint16_t) _buf[0] << 8) | (uint16_t) _buf[1];

  if (checkCRC(rawValue, _buf[2]) != 0) return (ERROR_BAD_CRC); //Error out

  return rawValue & 0xFFFC; // Zero out the status bits
}

//Read the humidity
/*******************************************************************************************/
//Calc humidity and return it to the user
//Returns 998 if I2C timed out
//Returns 999 if CRC is wrong
float HTU21D::readHumidity(void) {
  uint16_t rawHumidity = readValue(TRIGGER_HUMD_MEASURE_NOHOLD);
  
  if(rawHumidity == ERROR_I2C_TIMEOUT || rawHumidity == ERROR_BAD_CRC) return(rawHumidity);

  //Given the raw humidity data, calculate the actual relative humidity
  float tempRH = rawHumidity * (125.0 / 65536.0); //2^16 = 65536
  float rh = tempRH - 6.0; //From page 14

  return (rh);
}

//Read the temperature
/*******************************************************************************************/
//Calc temperature and return it to the user
//Returns 998 if I2C timed out
//Returns 999 if CRC is wrong
float HTU21D::readTemperature(void) {
  uint16_t rawTemperature = readValue(TRIGGER_TEMP_MEASURE_NOHOLD);

  if(rawTemperature == ERROR_I2C_TIMEOUT || rawTemperature == ERROR_BAD_CRC) return(rawTemperature);

  //Given the raw temperature data, calculate the actual temperature
  float tempTemperature = rawTemperature * (175.72 / 65536.0); //2^16 = 65536
  float realTemperature = tempTemperature - 46.85; //From page 14

  return (realTemperature);
}

//Set sensor resolution
/*******************************************************************************************/
//Sets the sensor resolution to one of four levels
//Page 12:
// 0/0 = 12bit RH, 14bit Temp
// 0/1 = 8bit RH, 12bit Temp
// 1/0 = 10bit RH, 13bit Temp
// 1/1 = 11bit RH, 11bit Temp
//Power on default is 0/0

void HTU21D::setResolution(uint8_t resolution) {
  uint8_t userRegister = readUserRegister(); //Go get the current register state
  userRegister &= 0b01111110; //Turn off the resolution bits
  resolution &= 0b10000001; //Turn off all other bits but resolution bits
  userRegister |= resolution; //Mask in the requested resolution bits

  //Request a write to user register
  writeUserRegister(userRegister);
}

/***************************************************************************/
/*
    readDeviceID()
    Reads device id
    NOTE:
    - see p.23 of Si7021 datasheet for details
    - full serial number is {SNA3, SNA2, SNA1, SNA0, SNB3**, SNB2, SNB1, SNB0}
    - **chip ID:
        - 0x0D: Si7013
        - 0x14: Si7020
        - 0x15: Si7021 
        - 0x32: HTU21D & SHT21
*/
/**************************************************************************/
uint16_t HTU21D::readDeviceID(void) {
  int ret;
  uint16_t deviceID = 0;
  uint8_t  checksum = 0;

  /* request serial_2 -> SNB3**, SNB2, SNB1, SNB0 */
  _buf[0] = HTU21D_SERIAL2_READ1;
  _buf[1] = HTU21D_SERIAL2_READ2;
  ret = _i2c->write(HTU21D_ADDRESS, (const char*)_buf, 2);

  /* read serial_2 -> SNB3**, SNB2, CRC */
  ret = _i2c->read(HTU21D_ADDRESS, (char*)_buf, 3);
  deviceID  = _buf[0] << 8;
  deviceID |= _buf[1];
  checksum  = _buf[2];

  if (checkCRC(deviceID, checksum) != 0) return (ERROR_BAD_CRC); //Error out

  deviceID = deviceID >> 8;

  return deviceID;
}

/***************************************************************************/
/*
    readFirmwareVersion()
    Reads firware version
    NOTE:
    - see p.24 of Si7021 datasheet for details
*/
/**************************************************************************/
uint16_t HTU21D::readFirmwareVersion(void) {
  int ret;
  uint16_t firmwareVersion = 0;

  /* request firware version */
  _buf[0] = HTU21D_FIRMWARE_READ1;
  _buf[1] = HTU21D_FIRMWARE_READ2;
  ret = _i2c->write(HTU21D_ADDRESS, (const char*)_buf, 2);

  /* read firware version */
  ret = _i2c->read(HTU21D_ADDRESS, (char*)_buf, 1);
  firmwareVersion  = _buf[0];

  return firmwareVersion;
}

//Read the user register
uint8_t HTU21D::readUserRegister(void) {
  int ret;
  uint8_t userRegister;

  //Request the user register
  _buf[0] = READ_USER_REG;
  ret = _i2c->write(HTU21D_ADDRESS, (const char*)_buf, 1); //Read the user register

  //Read result
  ret = _i2c->read(HTU21D_ADDRESS, (char*)_buf, 1);
  userRegister = _buf[0];

  return (userRegister);
}

void HTU21D::writeUserRegister(uint8_t val) {
  int ret;
  _buf[0] = WRITE_USER_REG; //Write to the user register
  _buf[1] = val;            //Write the new resolution bits
  ret = _i2c->write(HTU21D_ADDRESS, (const char*)_buf, 2);
}

//Give this function the 2 byte message (measurement) and the check_value byte from the HTU21D
//If it returns 0, then the transmission was good
//If it returns something other than 0, then the communication was corrupted
//From: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
//POLYNOMIAL = 0x0131 = x^8 + x^5 + x^4 + 1 : http://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
#define SHIFTED_DIVISOR 0x988000 //This is the 0x0131 polynomial shifted to farthest left of three bytes

uint8_t HTU21D::checkCRC(uint16_t message_from_sensor, uint8_t check_value_from_sensor) {
  //Test cases from datasheet:
  //message = 0xDC, checkvalue is 0x79
  //message = 0x683A, checkvalue is 0x7C
  //message = 0x4E85, checkvalue is 0x6B

  uint32_t remainder = (uint32_t)message_from_sensor << 8; //Pad with 8 bits because we have to add in the check value
  remainder |= check_value_from_sensor; //Add on the check value

  uint32_t divsor = (uint32_t)SHIFTED_DIVISOR;

  for (int i = 0 ; i < 16 ; i++) { //Operate on only 16 positions of max 24. The remaining 8 are our remainder and should be zero when we're done.
    if ( remainder & (uint32_t)1 << (23 - i) ) //Check if there is a one in the left position
      remainder ^= divsor;

    divsor >>= 1; //Rotate the divsor max 16 times so that we have 8 bits left of a remainder
  }

  return (uint8_t)remainder;
}