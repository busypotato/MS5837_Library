# BlueRobotics MS5837 Library ![Test](https://github.com/bluerobotics/BlueRobotics_MS5837_Library/workflows/Test/badge.svg)

Arduino library for the MS5837 pressure sensor. The MS5837 is a tiny SMT pressures sensor from Measurement Specialties that can measure pressure of up to 30 Bar (300m depth) with resolution of 0.2 mbar.

This library also supports the MS5837-02BA which has a much smaller measurement range and is better suited for altitude measurement in air.

# Documentation

Please see the examples for normal operation. Below are the available functions used in the library.

```cpp
MS5837();

/** Must be called before attempting to operate the sensor.
 * Returns true if the sensor was initialized successfully.
 * wirePort is optional and defaults to Wire.
 * You can select a different port by calling (e.g.) init(Wire1)
 */
bool init(TwoWire &wirePort);

/** Calls init.
 * Returns true if the sensor was initialized successfully.
 */
bool begin(TwoWire &wirePort);

/** Set model of MS5837 sensor. Valid options are MS5837::MS5837_30BA (default)
 * and MS5837::MS5837_02BA.
 */
void setModel(uint8_t model);

/** Set the oversampling ratio (OSR) used for pressure or temperature
 * conversions. Valid values are 256, 512, 1024, 2048, 4096 and 8192 (the
 * default). A higher OSR gives lower noise but a longer conversion time, so
 * lowering the OSR is the main way to increase the sample rate. Each setter
 * returns true if the value was accepted, or false if it was invalid (in
 * which case the previous setting is kept).
 */
bool setPressureOSR(uint16_t osr);
bool setTemperatureOSR(uint16_t osr);

/** Return the currently configured OSR for each channel.
 */
uint16_t getPressureOSR();
uint16_t getTemperatureOSR();

/** Return the ADC conversion delay, in milliseconds, currently used for each
 * channel. This depends on the configured OSR and is exposed to help build a
 * non-blocking read loop.
 */
uint32_t getPressureConversionDelayMS();
uint32_t getTemperatureConversionDelayMS();

/** Provide the density of the working fluid in kg/m^3. Default is for
 * seawater. Should be 997 for freshwater.
 */
void setFluidDensity(float density);

/** Reads the sensor. Performs a pressure conversion followed by a temperature
 * conversion and blocks for the conversion time of each, so the duration
 * depends on the configured OSR (up to ~40 ms at the default OSR of 8192, and
 * much less at lower OSR). Use sparingly if possible.
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

```

# Sample rate and oversampling (OSR)

By default the sensor uses the highest oversampling ratio (8192) for both
pressure and temperature, which gives the best resolution but the slowest
conversion. If you need faster readings, lower the OSR with `setPressureOSR()`
and/or `setTemperatureOSR()`. The available values and their approximate
maximum conversion times (per the datasheet) are:

| OSR  | Max conversion time |
|------|---------------------|
| 256  | 0.56 ms             |
| 512  | 1.10 ms             |
| 1024 | 2.17 ms             |
| 2048 | 4.32 ms             |
| 4096 | 8.61 ms             |
| 8192 | 17.2 ms             |

Because `read()` performs a pressure conversion and a temperature conversion
back to back, the total time is roughly the sum of the two channels'
conversion times plus I2C overhead. Since temperature changes slowly, one
common optimisation is to sample temperature less often than pressure.

# Versions

1.1.0 - Expose per-channel oversampling configuration (`setPressureOSR` /
        `setTemperatureOSR`, with matching getters and conversion-delay
        getters) so the sample rate can be increased. Fully backwards
        compatible; no existing calls change.

1.0.0 - First release, included in Arduino Library Manager

# Reference

You can find the [MS5837-30BA datasheet here](http://www.mouser.com/ds/2/418/MS5837-30BA-736494.pdf).
