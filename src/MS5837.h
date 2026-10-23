/* Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library
------------------------------------------------------------

Title: Blue Robotics Arduino MS5837-30BA Pressure/Temperature Sensor Library

Description: This library provides utilities to communicate with and to
read data from the Measurement Specialties MS5837-30BA pressure/temperature
sensor.

Authors: Rustom Jehangir, Blue Robotics Inc.
         Adam Šimko, Blue Robotics Inc.

-------------------------------
The MIT License (MIT)

Copyright (c) 2015 Blue Robotics Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-------------------------------*/

#ifndef MS5837_H_BLUEROBOTICS
#define MS5837_H_BLUEROBOTICS

#include "Arduino.h"
#include <Wire.h>
#include <cstdint>

class MS5837 {
public:
  static const float Pa;
  static const float bar;
  static const float mbar;

  static const uint8_t MS5837_30BA;
  static const uint8_t MS5837_02BA;
  static const uint8_t MS5837_UNRECOGNISED;

  static const uint16_t MS5837_OSR_256;
  static const uint16_t MS5837_OSR_512;
  static const uint16_t MS5837_OSR_1024;
  static const uint16_t MS5837_OSR_2048;
  static const uint16_t MS5837_OSR_4096;
  static const uint16_t MS5837_OSR_8192;

  MS5837();

  bool init(TwoWire &wirePort = Wire);
  bool begin(TwoWire &wirePort = Wire); // Calls init()

  /** Set model of MS5837 sensor. Valid options are MS5837::MS5837_30BA
   * (default) and MS5837::MS5837_02BA.
   */
  void setModel(uint8_t model);
  uint8_t getModel();

  /** Set the oversampling ratio (OSR) used for temperature conversions.
   * Valid values are 256, 512, 1024, 2048, 4096 and 8192 (the default).
   * A higher OSR gives lower noise but a longer conversion time; lowering
   * the OSR is the main way to speed up read(). Returns true if the value
   * was accepted, or false if it was invalid (in which case the previous
   * setting is kept unchanged).
   */
  bool setTemperatureOSR(uint16_t osr);

  /** Returns the currently configured temperature OSR. */
  uint16_t getTemperatureOSR();

  /** Set the oversampling ratio (OSR) used for pressure conversions.
   * Valid values are 256, 512, 1024, 2048, 4096 and 8192 (the default).
   * A higher OSR gives lower noise but a longer conversion time; lowering
   * the OSR is the main way to speed up read(). Returns true if the value
   * was accepted, or false if it was invalid (in which case the previous
   * setting is kept unchanged).
   */
  bool setPressureOSR(uint16_t osr);

  /** Returns the currently configured pressure OSR. */
  uint16_t getPressureOSR();

  /** Returns the ADC conversion delay, in milliseconds, currently used for
   * pressure conversions. This depends on the configured pressure OSR and is
   * exposed to support building a non-blocking read loop.
   */
  uint32_t getPressureConversionDelayMS();

  /** Returns the ADC conversion delay, in milliseconds, currently used for
   * temperature conversions. This depends on the configured temperature OSR
   * and is exposed to support building a non-blocking read loop.
   */
  uint32_t getTemperatureConversionDelayMS();

  /** Provide the density of the working fluid in kg/m^3. Default is for
   * seawater. Should be 997 for freshwater.
   */
  void setFluidDensity(float density);

  /** The read from I2C takes up to 40 ms at the default (highest) OSR and
   *  less at lower OSR, so use sparingly if possible.
   */
  void read();

  /** Pressure returned in mbar or mbar*conversion rate.
   */
  float pressure(float conversion = 1.0f);

  /** Temperature returned in deg C.
   */
  float temperature();

  /** Depth returned in meters (valid for operation in incompressible
   *  liquids only. Uses density that is set for fresh or seawater.
   */
  float depth();

  /** Altitude returned in meters (valid for operation in air only).
   */
  float altitude();

private:
  // This stores the requested i2c port
  TwoWire *_i2cPort;

  uint16_t C[8];
  uint32_t D1_pres, D2_temp;
  int32_t TEMP;
  int32_t P;
  uint8_t _model;
  uint16_t _D1PressureOSR;
  uint16_t _D2TemperatureOSR;
  uint8_t _D1PressureConvertCmd;
  uint8_t _D2TemperatureConvertCmd;
  uint32_t _D1PressureConversionDelayMS;
  uint32_t _D2TemperatureConversionDelayMS;

  float fluidDensity;

  /** Performs calculations per the sensor data sheet for conversion and
   *  second order compensation.
   */
  void calculate();

  uint8_t crc4(uint16_t n_prom[]);
};

#endif
