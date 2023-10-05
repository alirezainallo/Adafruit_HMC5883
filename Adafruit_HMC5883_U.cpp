/*!
 * @file Adafruit_HMC5883_U.cpp
 *
 * @mainpage Adafruit HMC5883 Unified Library
 *
 * @section intro_sec Introduction
 *
 * This is a library for the HMC5883 magnentometer/compass
 *
 * Designed specifically to work with the Adafruit HMC5883 Breakout
 * http://www.adafruit.com/products/1746
 *
 * These displays use I2C to communicate, 2 pins are required to interface.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit andopen-source hardware by purchasing products
 * from Adafruit!
 *
 * @section author Author
 *
 * Written by Kevin Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 */

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifdef __AVR_ATtiny85__
#include "TinyWireM.h"
#define Wire TinyWireM
#else
#include <Wire.h>
#endif

#include <limits.h>

#include "Adafruit_HMC5883_U.h"

static float _hmc5883_Gauss_LSB_XY = 1100.0F; // Varies with gain
static float _hmc5883_Gauss_LSB_Z = 980.0F;   // Varies with gain

/***************************************************************************
 MAGNETOMETER
 ***************************************************************************/
/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Reads the raw data from the sensor
*/
/**************************************************************************/
void Adafruit_HMC5883_Unified::read() {
  Adafruit_BusIO_Register power_mg_out_x =
      Adafruit_BusIO_Register(i2c_dev, HMC5883_REGISTER_MAG_OUT_X_H_M, 2, 1);
  Adafruit_BusIO_Register power_mg_out_y =
      Adafruit_BusIO_Register(i2c_dev, HMC5883_REGISTER_MAG_OUT_Y_H_M, 2, 1);
  Adafruit_BusIO_Register power_mg_out_z =
      Adafruit_BusIO_Register(i2c_dev, HMC5883_REGISTER_MAG_OUT_Z_H_M, 2, 1);
  
  // Read the magnetometer
  int16_t compass_x = 0;
  int16_t compass_y = 0;
  int16_t compass_z = 0;
  power_mg_out_x.read((uint16_t*)&compass_x);
  power_mg_out_y.read((uint16_t*)&compass_y);
  power_mg_out_z.read((uint16_t*)&compass_z);
  
  // Shift values to create properly formed integer (low byte first)
  _magData.x = compass_x;
  _magData.y = compass_y;
  _magData.z = compass_z;

  // ToDo: Calculate orientation
  _magData.orientation = 0.0;
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_HMC5883 class
*/
/**************************************************************************/
Adafruit_HMC5883_Unified::Adafruit_HMC5883_Unified(int32_t sensorID) {
  _sensorID = sensorID;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool Adafruit_HMC5883_Unified::begin(uint8_t i2c_address, TwoWire *wire,
                             int32_t sensor_id) {
  // Enable I2C
  // Wire.begin();
  if(i2c_dev){
    delete i2c_dev;
  }
  i2c_dev = new Adafruit_I2CDevice(i2c_address, wire);

  bool hmc_found = false;
  for (uint8_t tries = 0; tries < 5; tries++) {
    hmc_found = i2c_dev->begin();
    if (hmc_found)
      break;
    delay(10);
  }
  
  if (!hmc_found)
    return false;

  Adafruit_BusIO_Register mag_mr_reg =
      Adafruit_BusIO_Register(i2c_dev, HMC5883_REGISTER_MAG_MR_REG_M, 1);
  
  // Enable the magnetometer
  mag_mr_reg.write(0x00);

  // Set the gain to a known level
  // setMagGain(HMC5883_MAGGAIN_4_0);//HMC5883_MAGGAIN_1_3

  return true;
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's gain
*/
/**************************************************************************/
void Adafruit_HMC5883_Unified::setMagGain(hmc5883MagGain gain) {
  
  Adafruit_BusIO_Register mag_crb_reg =
      Adafruit_BusIO_Register(i2c_dev, HMC5883_REGISTER_MAG_CRB_REG_M, 1);

  mag_crb_reg.write((uint32_t)gain);

  _magGain = gain;

  switch (gain) {
  case HMC5883_MAGGAIN_1_3:
    _hmc5883_Gauss_LSB_XY = 1100;
    _hmc5883_Gauss_LSB_Z = 980;
    break;
  case HMC5883_MAGGAIN_1_9:
    _hmc5883_Gauss_LSB_XY = 855;
    _hmc5883_Gauss_LSB_Z = 760;
    break;
  case HMC5883_MAGGAIN_2_5:
    _hmc5883_Gauss_LSB_XY = 670;
    _hmc5883_Gauss_LSB_Z = 600;
    break;
  case HMC5883_MAGGAIN_4_0:
    _hmc5883_Gauss_LSB_XY = 450;
    _hmc5883_Gauss_LSB_Z = 400;
    break;
  case HMC5883_MAGGAIN_4_7:
    _hmc5883_Gauss_LSB_XY = 400;
    _hmc5883_Gauss_LSB_Z = 255;
    break;
  case HMC5883_MAGGAIN_5_6:
    _hmc5883_Gauss_LSB_XY = 330;
    _hmc5883_Gauss_LSB_Z = 295;
    break;
  case HMC5883_MAGGAIN_8_1:
    _hmc5883_Gauss_LSB_XY = 230;
    _hmc5883_Gauss_LSB_Z = 205;
    break;
  }
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool Adafruit_HMC5883_Unified::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  /* Read new data */
  read();

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_MAGNETIC_FIELD;
  event->timestamp = 0;
  event->magnetic.x =
      _magData.x;
  event->magnetic.y =
      _magData.y;
  event->magnetic.z =
      _magData.z;
  // event->magnetic.x =
  //     _magData.x / _hmc5883_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  // event->magnetic.y =
  //     _magData.y / _hmc5883_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  // event->magnetic.z =
  //     _magData.z / _hmc5883_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Adafruit_HMC5883_Unified::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "HMC5883", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = 0;
  sensor->max_value = 800;  // 8 gauss == 800 microTesla
  sensor->min_value = -800; // -8 gauss == -800 microTesla
  sensor->resolution = 0.2; // 2 milligauss == 0.2 microTesla
}
