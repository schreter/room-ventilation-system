/*
 * Copyright (C) 2019 Ivan Schréter (schreter@gmx.net)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * This copyright notice MUST APPEAR in all copies of the software!
 */

/**
 * @file
 * @brief Library for differential pressure sensor Sensirion SDP8xx/SDP6xx for Arduino.
 *
 * Supported Sensor modules:
 * - SDP8xx series from Sensirion - https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/0_Datasheets/Differential_Pressure/Sensirion_Differential_Pressure_Sensors_SDP8xx_Digital_Datasheet.pdf
 * - SDP6xx series from Sensirion (not tested properly yet)
 */


#ifndef SDP8XX_H
#define SDP8XX_H

#include <inttypes.h>

class SDP8xx
{
public:
  /// Supported sensor types.
  enum class type : uint8_t
  {
    invalid,  ///< No sensor detected.
    simulated,///< Sensor is simulated, read does nothing (for debugging/development).
    SDP8xx,   ///< Detected SDP8xx sensor.
    SDP6xx    ///< Detected SDP6xx sensor.
  };

  /// Error code of a command sent to the sensor.
  enum class result : uint8_t
  {
    ok                = 0,  ///< No error, all OK.
    not_implemented   = 1,  ///< Function not implemented for the sensor type.
    invalid_parameter = 2,  ///< Invalid parameter calling the function or called in wrong state.
    no_data           = 3,  ///< No data received from the device, although should receive some.
    short_data        = 4,  ///< Too few bytes received from the device.
    sensor_not_found  = 5,  ///< No sensor at the given I2C address.
    timeout           = 6,  ///< Timed out reading data from the device.
    checksum_error    = 7,  ///< Error computing checksum of the received data.
    unknown_error     = 255 ///< Unknown error during I2C transmit.
  };

  /*!
   * @brief Construct a new sensor handler.
   *
   * You must call begin() to initialize the sensor.
   *
   * @param address I2C address to use (0 for probing default addresses 0x40/0x25/0x26).
   * @param tc_diff_press if set, use differential pressure temperature compensation,
   *    else mass flow temperature compensation (SDP8xx only).
   */
  explicit SDP8xx(uint8_t address = 0, bool tc_diff_press = true);

  /*!
   * @brief Check whether the sensor is present at the address specified in the constructor.
   *
   * The code can detect SDP8xx sensor for sure, since it provides ID command. In that case
   * getType() will return type::SDP8xx. If SDP8xx sensor cannot be detected via ID command,
   * then a read is performed as a fallback to try to detect SDP8xx/SDP6xx sensor.
   *
   * @return @c true, if the sensor is present, @c false otherwise.
   */
  bool begin();

  /*!
   * @brief Initialize the sensor as simulated sensor.
   *
   * To go back to normal mode, use begin() to reinitialize the sensor.
   */
  void beginSimulated();

  /// Get detected sensor type.
  type getType() const { return type_; }

  /*!
   * @brief Set sensor range to handle different sensor types.
   *
   * For SDP8xx, the range is determined automatically based on identified
   * sensor type and doesn't have to be set.
   *
   * For SDP6xx, default range is 125Pa.
   *
   * @param range_pa sensor range in Pa (typically 500, 125 or 25).
   * @return error code.
   */
  result setSensorRange(uint16_t range_pa);

  /*!
   * @brief Set required resolution of sensor readings in bits.
   *
   * This method is only relevant for SDP6xx, SDP8xx always uses 16-bit resolution
   * and returns result::ok here.
   *
   * @param bits resolution in bits (9-16 bits are allowed).
   * @return error code.
   */
  result setSensorResolution(uint8_t bits);

  /*!
   * @brief Start continuous measurement to quickly return measured values (SDP8xx only).
   *
   * @param tc_diff_press if set, use differential pressure temperature compensation,
   *    else mass flow temperature compensation.
   * @param with_average if set, then the measurements will be averaged to produce
   *    more exact data, but the sensor will react slower to changes.
   * @return error code.
   */
  result startContinuousMeasurement(bool tc_diff_press = false, bool with_average = false);

  /*!
   * @brief End continuous measurements, if started, and go back to trigger mode.
   *
   * By default, the sensor is in trigger mode.
   *
   * @param tc_diff_press if set, use differential pressure temperature compensation,
   *    else mass flow temperature compensation (SDP8xx only).
   * @return error code.
   */
  result startTriggerMode(bool tc_diff_press = false);

  /*!
   * @brief Request a new measurement.
   *
   * If in trigger mode, a command with request is sent to the sensor and
   * then the measurement is read back. In continuous measurement mode, simply
   * the data is read from the sensor.
   *
   * @note If continuous measurement was not applied, then the reading takes
   *    up to 50ms, during which the execution of the program is blocked.
   *    Reading with continuous measurement enabled takes typically <1ms
   *    (except possibly the very first reading, which takes up to 8ms).
   *
   * In case simulation was turned on, then the read always succeeds and values
   * referred to are not changed. This is useful when reading sensor into state
   * variables over and over - in simulation mode, those variables simply stay
   * set as they were and can be updated using some external debug mechanism.
   *
   * @param pressure [out] filled by measured pressure differential in Pa.
   * @param temperature [out] filled by measured temperature (centigrade),
   *    if supported (i.e., on SDP8xx), else NaN.
   * @return error code.
   */
  result read(float& pressure_diff_pa, float& temperature_c);

private:
  /// Identify SDP8xx sensor by using ID command.
  bool detectSDP8xx();

  /// Identify SDP8xx sensor by trying to do a read from it.
  bool detectSDP8xxViaRead();

  /// Identify SDP6xx sensor by trying to do a read from it.
  bool detectSDP6xxViaRead();

  /*!
   * @brief Read reply from the sensor.
   *
   * @param dest destination where to store the data.
   * @param size destination size in bytes (multiples of 2).
   * @return error code.
   */
  result readReply(void* dest, uint8_t size);

  uint8_t address_;                   ///< I2C address of the sensor (or 0 if not present).
  type type_ = type::invalid;         ///< SDP sensor type.
  bool is_continuous_ = false;        ///< Flag whether continuous measurement is enabled.
  bool tc_diff_press_ = false;        ///< Flag whether to use differential pressure temperature compensation.
  float scale_factor_ = 1/240.0;      ///< Scale factor for reading data
};

#endif
