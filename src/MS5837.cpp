#include "MS5837.h"
#include <Wire.h>

const uint8_t MS5837_ADDR = 0x76;
const uint8_t MS5837_RESET = 0x1E;
const uint8_t MS5837_ADC_READ = 0x00;
const uint8_t MS5837_PROM_READ = 0xA0;
const uint8_t MS5837_CONVERT_D1_256 = 0x40;
const uint8_t MS5837_CONVERT_D1_512 = 0x42;
const uint8_t MS5837_CONVERT_D1_1024 = 0x44;
const uint8_t MS5837_CONVERT_D1_2048 = 0x46;
const uint8_t MS5837_CONVERT_D1_4096 = 0x48;
const uint8_t MS5837_CONVERT_D1_8192 = 0x4A;
const uint8_t MS5837_CONVERT_D2_256 = 0x50;
const uint8_t MS5837_CONVERT_D2_512 = 0x52;
const uint8_t MS5837_CONVERT_D2_1024 = 0x54;
const uint8_t MS5837_CONVERT_D2_2048 = 0x56;
const uint8_t MS5837_CONVERT_D2_4096 = 0x58;
const uint8_t MS5837_CONVERT_D2_8192 = 0x5A;

const uint32_t MS5837_256_DELAY_MS = 1UL;
const uint32_t MS5837_512_DELAY_MS = 2UL;
const uint32_t MS5837_1024_DELAY_MS = 3UL;
const uint32_t MS5837_2048_DELAY_MS = 5UL;
const uint32_t MS5837_4096_DELAY_MS = 9UL;
const uint32_t MS5837_8192_DELAY_MS = 18UL;

// Maximum conversion times from the datasheet, in microseconds
const uint32_t MS5837_256_DELAY_US = 560UL;
const uint32_t MS5837_512_DELAY_US = 1100UL;
const uint32_t MS5837_1024_DELAY_US = 2170UL;
const uint32_t MS5837_2048_DELAY_US = 4320UL;
const uint32_t MS5837_4096_DELAY_US = 8610UL;
const uint32_t MS5837_8192_DELAY_US = 17200UL;

const uint16_t MS5837::MS5837_OSR_256 = 256;
const uint16_t MS5837::MS5837_OSR_512 = 512;
const uint16_t MS5837::MS5837_OSR_1024 = 1024;
const uint16_t MS5837::MS5837_OSR_2048 = 2048;
const uint16_t MS5837::MS5837_OSR_4096 = 4096;
const uint16_t MS5837::MS5837_OSR_8192 = 8192;

const float MS5837::Pa = 100.0f;
const float MS5837::bar = 0.001f;
const float MS5837::mbar = 1.0f;

const uint8_t MS5837::MS5837_30BA = 0;
const uint8_t MS5837::MS5837_02BA = 1;
const uint8_t MS5837::MS5837_UNRECOGNISED = 255;

// context:
// https://github.com/ArduPilot/ardupilot/pull/29122#issuecomment-2877269114
const uint16_t MS5837_02BA_MAX_SENSITIVITY = 49000;
const uint16_t MS5837_02BA_30BA_SEPARATION = 37000;
const uint16_t MS5837_30BA_MIN_SENSITIVITY = 26000;

MS5837::MS5837() {
  _i2cPort = NULL; // read() and the request/read functions check this
  fluidDensity = 1029;
  this->setTemperatureOSR(8192);
  this->setPressureOSR(8192);
}

bool MS5837::begin(TwoWire &wirePort) { return (init(wirePort)); }

bool MS5837::init(TwoWire &wirePort) {
  _i2cPort = &wirePort; // Grab which port the user wants us to use

  // Reset the MS5837, per datasheet
  _i2cPort->beginTransmission(MS5837_ADDR);
  _i2cPort->write(MS5837_RESET);
  _i2cPort->endTransmission();

  // Wait for reset to complete
  delay(10);

  // Read calibration values and CRC
  for (uint8_t i = 0; i < 7; i++) {
    _i2cPort->beginTransmission(MS5837_ADDR);
    _i2cPort->write(MS5837_PROM_READ + i * 2);
    _i2cPort->endTransmission();

    _i2cPort->requestFrom(MS5837_ADDR, 2);
    C[i] = (_i2cPort->read() << 8) | _i2cPort->read();
  }
  // Verify that data is correct with CRC
  uint8_t crcRead = C[0] >> 12;
  uint8_t crcCalculated = crc4(C);

  if (crcCalculated != crcRead) {
    return false; // CRC fail
  }

  // PROM Word 1 represents the sensor's pressure sensitivity calibration
  // Set _model according to the experimental pressure sensitivity thresholds
  if (C[1] < MS5837_30BA_MIN_SENSITIVITY ||
      C[1] > MS5837_02BA_MAX_SENSITIVITY) {
    _model = MS5837_UNRECOGNISED;
  } else if (C[1] > MS5837_02BA_30BA_SEPARATION) {
    _model = MS5837_02BA;
  } else {
    _model = MS5837_30BA;
  }

  // TODO: extract and store/report sensor package type from bits 11-5 of PROM
  // Word 0, per
  // https://github.com/ArduPilot/ardupilot/pull/29122#pullrequestreview-2837597764

  // The sensor has passed the CRC check, so we should return true even if
  // the sensor version is unrecognised.
  // (The MS5637 has the same address as the MS5837 and will also pass the CRC
  // check) (but will hopefully be unrecognised.)
  return true;
}

void MS5837::setModel(uint8_t model) { _model = model; }

uint8_t MS5837::getModel() { return (_model); }

bool MS5837::setPressureOSR(uint16_t osr) {
  uint8_t cmd;
  uint32_t d_ms;
  uint32_t d_us;
  switch (osr) {
  case 256:
    cmd = MS5837_CONVERT_D1_256;
    d_ms = MS5837_256_DELAY_MS;
    d_us = MS5837_256_DELAY_US;
    break;
  case 512:
    cmd = MS5837_CONVERT_D1_512;
    d_ms = MS5837_512_DELAY_MS;
    d_us = MS5837_512_DELAY_US;
    break;
  case 1024:
    cmd = MS5837_CONVERT_D1_1024;
    d_ms = MS5837_1024_DELAY_MS;
    d_us = MS5837_1024_DELAY_US;
    break;
  case 2048:
    cmd = MS5837_CONVERT_D1_2048;
    d_ms = MS5837_2048_DELAY_MS;
    d_us = MS5837_2048_DELAY_US;
    break;
  case 4096:
    cmd = MS5837_CONVERT_D1_4096;
    d_ms = MS5837_4096_DELAY_MS;
    d_us = MS5837_4096_DELAY_US;
    break;
  case 8192:
    cmd = MS5837_CONVERT_D1_8192;
    d_ms = MS5837_8192_DELAY_MS;
    d_us = MS5837_8192_DELAY_US;
    break;
  default:
    return false; // reject; leave existing setting untouched
  }
  _D1PressureOSR = osr;
  _D1PressureConvertCmd = cmd;
  _D1PressureConversionDelayMS = d_ms;
  _D1PressureConversionDelayUS = d_us;
  return true;
}

uint16_t MS5837::getPressureOSR() { return (_D1PressureOSR); }

bool MS5837::setTemperatureOSR(uint16_t osr) {
  uint8_t cmd;
  uint32_t d_ms;
  uint32_t d_us;
  switch (osr) {
  case 256:
    cmd = MS5837_CONVERT_D2_256;
    d_ms = MS5837_256_DELAY_MS;
    d_us = MS5837_256_DELAY_US;
    break;
  case 512:
    cmd = MS5837_CONVERT_D2_512;
    d_ms = MS5837_512_DELAY_MS;
    d_us = MS5837_512_DELAY_US;
    break;
  case 1024:
    cmd = MS5837_CONVERT_D2_1024;
    d_ms = MS5837_1024_DELAY_MS;
    d_us = MS5837_1024_DELAY_US;
    break;
  case 2048:
    cmd = MS5837_CONVERT_D2_2048;
    d_ms = MS5837_2048_DELAY_MS;
    d_us = MS5837_2048_DELAY_US;
    break;
  case 4096:
    cmd = MS5837_CONVERT_D2_4096;
    d_ms = MS5837_4096_DELAY_MS;
    d_us = MS5837_4096_DELAY_US;
    break;
  case 8192:
    cmd = MS5837_CONVERT_D2_8192;
    d_ms = MS5837_8192_DELAY_MS;
    d_us = MS5837_8192_DELAY_US;
    break;
  default:
    return false; // reject; leave existing setting untouched
  }
  _D2TemperatureOSR = osr;
  _D2TemperatureConvertCmd = cmd;
  _D2TemperatureConversionDelayMS = d_ms;
  _D2TemperatureConversionDelayUS = d_us;
  return true;
}

uint16_t MS5837::getTemperatureOSR() { return (_D2TemperatureOSR); }

uint32_t MS5837::getPressureConversionDelayMS() {
  return (_D1PressureConversionDelayMS);
}

uint32_t MS5837::getTemperatureConversionDelayMS() {
  return (_D2TemperatureConversionDelayMS);
}

uint32_t MS5837::getPressureConversionDelayUS() {
  return (_D1PressureConversionDelayUS);
}

uint32_t MS5837::getTemperatureConversionDelayUS() {
  return (_D2TemperatureConversionDelayUS);
}

void MS5837::setFluidDensity(float density) { fluidDensity = density; }

void MS5837::read() {

  // Check that _i2cPort is not NULL (i.e. has the user forgoten to call .init
  // or .begin?)
  if (_i2cPort == NULL) {
    return;
  }

  requestPressureConversion();
  delay(_D1PressureConversionDelayMS);
  readPressureResult();

  requestTemperatureConversion();
  delay(_D2TemperatureConversionDelayMS);
  readTemperatureResult();

  calculate();
}

bool MS5837::requestPressureConversion() {
  return sendCommand(_D1PressureConvertCmd);
}

bool MS5837::readPressureResult() { return readADC(D1_pres); }

bool MS5837::requestTemperatureConversion() {
  return sendCommand(_D2TemperatureConvertCmd);
}

bool MS5837::readTemperatureResult() { return readADC(D2_temp); }

bool MS5837::sendCommand(uint8_t cmd) {
  if (_i2cPort == NULL) {
    return false;
  }
  _i2cPort->beginTransmission(MS5837_ADDR);
  _i2cPort->write(cmd);
  return _i2cPort->endTransmission() == 0;
}

// Reads the 24-bit ADC result. The sensor returns 0 if the conversion has not
// finished, so 0 is reported as a failure; `result` is still updated.
bool MS5837::readADC(uint32_t &result) {
  if (!sendCommand(MS5837_ADC_READ)) {
    return false;
  }

  if (_i2cPort->requestFrom(MS5837_ADDR, (uint8_t)3) != 3) {
    return false;
  }
  result = _i2cPort->read();
  result = (result << 8) | _i2cPort->read();
  result = (result << 8) | _i2cPort->read();

  return result != 0;
}

void MS5837::calculate() {
  // Given C1-C6 and D1, D2, calculated TEMP and P
  // Do conversion first and then second order temp compensation

  int32_t dT = 0;
  int64_t SENS = 0;
  int64_t OFF = 0;
  int32_t SENSi = 0;
  int32_t OFFi = 0;
  int32_t Ti = 0;
  int64_t OFF2 = 0;
  int64_t SENS2 = 0;

  // Terms called
  dT = D2_temp - uint32_t(C[5]) * 256l;
  if (_model == MS5837_02BA) {
    SENS = int64_t(C[1]) * 65536l + (int64_t(C[3]) * dT) / 128l;
    OFF = int64_t(C[2]) * 131072l + (int64_t(C[4]) * dT) / 64l;
    P = (D1_pres * SENS / (2097152l) - OFF) / (32768l);
  } else {
    SENS = int64_t(C[1]) * 32768l + (int64_t(C[3]) * dT) / 256l;
    OFF = int64_t(C[2]) * 65536l + (int64_t(C[4]) * dT) / 128l;
    P = (D1_pres * SENS / (2097152l) - OFF) / (8192l);
  }

  // Temp conversion
  TEMP = 2000l + int64_t(dT) * C[6] / 8388608LL;

  // Second order compensation
  if (_model == MS5837_02BA) {
    if ((TEMP / 100) < 20) { // Low temp
      Ti = (11 * int64_t(dT) * int64_t(dT)) / (34359738368LL);
      OFFi = (31 * (TEMP - 2000) * (TEMP - 2000)) / 8;
      SENSi = (63 * (TEMP - 2000) * (TEMP - 2000)) / 32;
    }
  } else {
    if ((TEMP / 100) < 20) { // Low temp
      Ti = (3 * int64_t(dT) * int64_t(dT)) / (8589934592LL);
      OFFi = (3 * (TEMP - 2000) * (TEMP - 2000)) / 2;
      SENSi = (5 * (TEMP - 2000) * (TEMP - 2000)) / 8;
      if ((TEMP / 100) < -15) { // Very low temp
        OFFi = OFFi + 7 * (TEMP + 1500l) * (TEMP + 1500l);
        SENSi = SENSi + 4 * (TEMP + 1500l) * (TEMP + 1500l);
      }
    } else if ((TEMP / 100) >= 20) { // High temp
      Ti = 2 * (dT * dT) / (137438953472LL);
      OFFi = (1 * (TEMP - 2000) * (TEMP - 2000)) / 16;
      SENSi = 0;
    }
  }

  OFF2 = OFF - OFFi; // Calculate pressure and temp second order
  SENS2 = SENS - SENSi;

  TEMP = (TEMP - Ti);

  if (_model == MS5837_02BA) {
    P = (((D1_pres * SENS2) / 2097152l - OFF2) / 32768l);
  } else {
    P = (((D1_pres * SENS2) / 2097152l - OFF2) / 8192l);
  }
}

float MS5837::pressure(float conversion) {
  if (_model == MS5837_02BA) {
    return P * conversion / 100.0f;
  } else {
    return P * conversion / 10.0f;
  }
}

float MS5837::temperature() { return TEMP / 100.0f; }

// The pressure sensor measures absolute pressure, so it will measure the
// atmospheric pressure + water pressure We subtract the atmospheric pressure to
// calculate the depth with only the water pressure The average atmospheric
// pressure of 101300 pascal is used for the calcuation, but atmospheric
// pressure varies If the atmospheric pressure is not 101300 at the time of
// reading, the depth reported will be offset In order to calculate the correct
// depth, the actual atmospheric pressure should be measured once in air, and
// that value should subtracted for subsequent depth calculations.
float MS5837::depth() {
  return (pressure(MS5837::Pa) - 101300) / (fluidDensity * 9.80665);
}

float MS5837::altitude() {
  return (1 - pow((pressure() / 1013.25), .190284)) * 145366.45 * .3048;
}

uint8_t MS5837::crc4(uint16_t n_prom[]) {
  uint16_t n_rem = 0;

  n_prom[0] = ((n_prom[0]) & 0x0FFF);
  n_prom[7] = 0;

  for (uint8_t i = 0; i < 16; i++) {
    if (i % 2 == 1) {
      n_rem ^= (uint16_t)((n_prom[i >> 1]) & 0x00FF);
    } else {
      n_rem ^= (uint16_t)(n_prom[i >> 1] >> 8);
    }
    for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & 0x8000) {
        n_rem = (n_rem << 1) ^ 0x3000;
      } else {
        n_rem = (n_rem << 1);
      }
    }
  }

  n_rem = ((n_rem >> 12) & 0x000F);

  return n_rem ^ 0x00;
}
