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

#include <Wire.h>

#include "SDP8xx.h"
#include "Arduino.h"

namespace
{
  /// Default sensor address on I2C bus.
  static constexpr uint8_t DEFAULT_ADDRESS_SDP8xx = 0x25;
  /// Secondary sensor address on I2C bus.
  static constexpr uint8_t SECONDARY_ADDRESS_SDP8xx = 0x26;
  /// Default sensor address on I2C bus.
  static constexpr uint8_t DEFAULT_ADDRESS_SDP6xx = 0x40;

  /*!
   * @brief Calculate checksum:
   *
   * From sensirion App Note "CRC Checksum": calculates checksum for n uint8_ts of data.
   *
   * @param crc_base base for CRC computation (depends on sensor type).
   * @param data,size block to checksum.
   * @return checksum.
   */
  static uint8_t crc(uint8_t crc_base, const uint8_t* data, uint8_t size)
  {
    // Checksum CRC-8 polynomial generator (to check if the recieved message is OK)
    static constexpr uint8_t POLYNOMIAL = 0x31; // P(x) = x^8 + x^5 + x^4 -1 = 100110001

    uint8_t crc = crc_base;
    for (uint8_t i = 0; i < size; ++i) {
      crc ^= data[i];
      for (uint8_t bit = 8; bit > 0; --bit) {
        if (crc & 0x80)
          crc = uint8_t(crc << 1) ^ POLYNOMIAL;
        else
          crc = uint8_t(crc << 1);
      }
    }
    return crc;
  }

  /// Convert Wire.endTransmission() to result code.
  static SDP8xx::result wire_to_result(uint8_t transmit_rc)
  {
    switch (transmit_rc)
    {
    case 0:
      return SDP8xx::result::ok;
    case 1: // too big data
      return SDP8xx::result::invalid_parameter;
    case 2: // NACK on address
      return SDP8xx::result::sensor_not_found;
    case 3: // NACK on command data
      return SDP8xx::result::invalid_parameter;
    case 4: // other error
    default:
      return SDP8xx::result::unknown_error;
    }
  }

  /// Convert big-endian number from sensor to Arduino's little endian.
  static int16_t to_le(int16_t n)
  {
    uint8_t* data = reinterpret_cast<uint8_t*>(&n);
    return (int16_t(data[0]) << 8) | int16_t(data[1]);
  }

  /// Convert big-endian number from sensor to Arduino's little endian.
  static uint32_t to_le(uint32_t n)
  {
    uint8_t* data = reinterpret_cast<uint8_t*>(&n);
    return (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) | (uint32_t(data[2]) << 8) | uint32_t(data[3]);
  }

  /// Commands for SDP8xx sensor.
  namespace sdp800_command
  {
    static constexpr uint16_t start_cont_mf     = 0x3608; ///< Start continuous reading, no average, tc mass flow.
    static constexpr uint16_t start_cont_mf_avg = 0x3603; ///< Start continuous reading, with average, tc mass flow.
    static constexpr uint16_t start_cont_dp     = 0x361e; ///< Start continuous reading, no average, tc diff press.
    static constexpr uint16_t start_cont_dp_avg = 0x3615; ///< Start continuous reading, with average, tc diff press.
    static constexpr uint16_t stop_cont         = 0x3ff9; ///< Stop continuous reading.

    static constexpr uint16_t trigger_read_mf_h = 0x3726; ///< Trigger one reading, tc mass flow, with clock stretching.
    static constexpr uint16_t trigger_read_dp_h = 0x372d; ///< Trigger one reading, tc diff press, with clock stretching.

    static constexpr uint16_t read_pid1         = 0x367c; ///< Read product identifier (1/2).
    static constexpr uint16_t read_pid2         = 0xe102; ///< Read product identifier (2/2).

#if 0 // other, unused commands:
    static constexpr uint16_t trigger_read_mf   = 0x3624; ///< Trigger one reading, tc mass flow.
    static constexpr uint16_t trigger_read_dp   = 0x362f; ///< Trigger one reading, tc diff press.
    static constexpr uint16_t sleep             = 0x3677; ///< Enter sleep mode.
#endif
  };

  /// Product identifiers.
  namespace sdp800_pid
  {
    static constexpr uint32_t SDP800_500 = 0x03020101;
    static constexpr uint32_t SDP810_500 = 0x03020A01;
    static constexpr uint32_t SDP801_500 = 0x03020401;
    static constexpr uint32_t SDP811_500 = 0x03020D01;
    static constexpr uint32_t SDP800_125 = 0x03020201;
    static constexpr uint32_t SDP810_125 = 0x03020B01;
  }

  /// Commands for SDP6xx sensor.
  namespace sdp600_command
  {
    static constexpr uint8_t trigger_read   = 0xF1; ///< Trigger a pressure measurement.
    static constexpr uint8_t soft_reset     = 0xFE; ///< Soft reset.
    static constexpr uint8_t read_user_reg  = 0xE5; ///< Read advanced user register.
    static constexpr uint8_t write_user_reg = 0xE4; ///< Write advanced user register.
  }

  /// Check if there is any device at specified address.
  static bool detectAnyDevice(uint8_t address)
  {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() != 0)
    {
      // no such device
      Serial.print(F("SDP8xx: Didn't find any device at I2C address 0x"));
      Serial.println(address, HEX);
      return false;
    }
    return true;
  }

  /// Send 1B command for SDP6xx.
  static SDP8xx::result sendCommand(uint8_t address, uint8_t cmd, const void* data = nullptr, uint8_t data_size = 0)
  {
    Wire.beginTransmission(address);
    Wire.write(cmd);
    if (data)
      Wire.write(reinterpret_cast<const uint8_t*>(data), data_size);
    return wire_to_result(Wire.endTransmission());
  }

  /// Send 2B command for SDP8xx.
  static SDP8xx::result sendCommand(uint8_t address, uint16_t cmd)
  {
    Wire.beginTransmission(address);
    Wire.write(uint8_t(cmd >> 8));
    Wire.write(uint8_t(cmd));
    return wire_to_result(Wire.endTransmission());
  }
}

SDP8xx::SDP8xx(uint8_t address, bool tc_diff_press) :
  address_(address),
  tc_diff_press_(tc_diff_press)
{}

bool SDP8xx::begin()
{
  if (address_)
  {
    if (!detectAnyDevice(address_))
      return false;
  }

  if (address_ == 0)
  {
    // try SDP8xx first
    address_ = DEFAULT_ADDRESS_SDP8xx;
    bool has_device_default_sdp8xx = detectAnyDevice(address_);
    if (has_device_default_sdp8xx)
    {
      if (detectSDP8xx())
        return true;
    }
    address_ = SECONDARY_ADDRESS_SDP8xx;
    bool has_device_secondary_sdp8xx = detectAnyDevice(address_);
    if (has_device_secondary_sdp8xx)
    {
      if (detectSDP8xx())
        return true;
    }

    // try SDP6xx as fallback
    address_ = DEFAULT_ADDRESS_SDP6xx;
    if (detectAnyDevice(address_))
    {
      if (detectSDP6xxViaRead())
        return true;
    }

    // didn't find SDP6xx, try with SPD8xx via read
    if (has_device_default_sdp8xx)
    {
      address_ = DEFAULT_ADDRESS_SDP8xx;
      if (detectSDP8xxViaRead())
        return true;
    }
    if (has_device_secondary_sdp8xx)
    {
      address_ = SECONDARY_ADDRESS_SDP8xx;
      if (detectSDP8xxViaRead())
        return true;
    }

    // didn't find anything
    address_ = 0;
  }
  else if (address_ == DEFAULT_ADDRESS_SDP6xx)
  {
    // should be SDP6xx
    if (detectSDP6xxViaRead())
      return true;

    // probably no sensor, but try with SDP8xx detection first
    if (detectSDP8xx() || detectSDP8xxViaRead())
      return true;

    // didn't detect it
  }
  else
  {
    // some other address, assume SDP8xx in the first place
    if (detectSDP8xx() || detectSDP8xxViaRead())
      return true;

    // probably no sensor, but try with SDP6xx detection first
    if (detectSDP6xxViaRead())
      return true;

    // else didn't detect it
  }

  type_ = type::invalid;
  return false;
}

void SDP8xx::beginSimulated()
{
  is_continuous_ = false;
  type_ = type::simulated;
}

SDP8xx::result SDP8xx::setSensorResolution(uint8_t bits)
{
  if (type_ == type::invalid)
    return result::sensor_not_found;

  if (bits < 9 || bits > 16)
    return result::invalid_parameter;

  if (type_ != type::SDP6xx)
    return result::ok; // always 16 bit

  auto rc = sendCommand(address_, sdp600_command::read_user_reg);
  if (rc != result::ok)
    return rc;

  uint16_t user_reg;
  rc = readReply(&user_reg, 2);
  if (rc != result::ok)
    return rc;

  user_reg &= 0xF1FF;
  user_reg |= uint16_t(bits - 9) << 9;
  return sendCommand(address_, sdp600_command::write_user_reg, &user_reg, 2);
}

SDP8xx::result SDP8xx::startContinuousMeasurement(bool tc_diff_press, bool with_average)
{
  if (type_ == type::invalid)
    return result::sensor_not_found;

  if (type_ == type::simulated)
    return result::ok;

  if (type_ != type::SDP8xx)
    return result::not_implemented;

  if (is_continuous_)
    return result::invalid_parameter; // already in continuous reading mode

  auto rc = sendCommand(
        address_,
        tc_diff_press ?
          (with_average ? sdp800_command::start_cont_dp_avg : sdp800_command::start_cont_dp) :
          (with_average ? sdp800_command::start_cont_mf_avg : sdp800_command::start_cont_mf));
  if (rc == result::ok)
  {
    is_continuous_ = true;
    tc_diff_press_ = tc_diff_press;
  }
  return rc;
}

SDP8xx::result SDP8xx::startTriggerMode(bool tc_diff_press)
{
  if (type_ == type::invalid)
    return result::sensor_not_found;

  tc_diff_press_ = tc_diff_press;
  if (!is_continuous_ || type_ != type::SDP8xx)
    return result::ok;  // already off

  auto rc = sendCommand(address_, sdp800_command::stop_cont);
  if (rc == result::ok)
  {
    is_continuous_ = false;
    delayMicroseconds(501); // next command can be sent earliest 500us later
  }
  return rc;
}

SDP8xx::result SDP8xx::read(float& pressure_diff_pa, float& temperature_c)
{
  if (type_ == type::invalid)
  {
    pressure_diff_pa = NAN;
    temperature_c = NAN;
    return result::sensor_not_found;
  }
  if (type_ == type::simulated)
  {
    // do not overwrite the values, simply leave as-is
    return result::ok;
  }

  uint8_t retry_count = 3;
  for (;;)
  {
    pressure_diff_pa = NAN;
    temperature_c = NAN;

    if (type_ == type::SDP6xx)
    {
      // fallback for old SDP6xx sensors
      auto rc = sendCommand(address_, sdp600_command::trigger_read);
      if (rc != result::ok)
      {
        if (retry_count--)
        {
          Serial.print(F("SDP6xx: Retrying reading, could not send read command, rc="));
          Serial.println(int8_t(rc));
          delay(20);
          continue;
        }
        Serial.print(F("SDP6xx: Giving up reading, could not send read command, rc="));
        Serial.println(int8_t(rc));
        return rc;
      }

      // the sensor will hold the bus until the read can be satisfied (for 45ms)
      int16_t value;
      rc = readReply(&value, 2);
      if (rc != result::ok)
      {
        if (retry_count--)
        {
          Serial.print(F("SDP6xx: Retrying reading, could not read data, rc="));
          Serial.println(int8_t(rc));
          delay(20);
          continue;
        }
        Serial.print(F("SDP6xx: Giving up reading, could not read data, rc="));
        Serial.println(int8_t(rc));
        return rc;
      }
      pressure_diff_pa = float(value) * scale_factor_;
      return result::ok;
    }
    // else assume SDP8xx

    struct {
      int16_t pressure_diff;
      int16_t temperature;
      int16_t scale_factor;
    } data;
    const bool read_scale_factor = isnanf(scale_factor_);
    if (!is_continuous_)
    {
      // trigger read
      auto rc = sendCommand(
            address_,
            tc_diff_press_ ? sdp800_command::trigger_read_dp_h : sdp800_command::trigger_read_mf_h);
      if (rc != result::ok)
      {
        if (retry_count--)
        {
          Serial.print(F("SDP8xx: Retrying reading, could not send read command, rc="));
          Serial.println(int8_t(rc));
          delay(20);
          continue;
        }
        return rc;
      }
      // The sensor will hold the bus until the read can be satisfied.
    }
    else
    {
      // Read can be done at any time, the soonest 8ms after the command was sent,
      // proper values are available after 20ms. The sensor will return no data
      // until it's available, so we can safely loop on it.
    }

    unsigned long start_time = millis();
    result rc;
    for (;;)
    {
      rc = readReply(&data, read_scale_factor ? 6 : 4);
      if (rc == result::ok)
        break;
      if (rc == result::no_data && is_continuous_)
      {
        // we didn't get data yet, wait for it up to 50ms (it should read within 8ms)
        if ((millis() - start_time) > 50)
          return result::timeout;
        delay(1);
        continue;
      }
    }
    if (rc != result::ok)
    {
      if (retry_count--)
      {
        Serial.print(F("SDP8xx: Retrying reading, could not read data, rc="));
        Serial.println(int8_t(rc));
        delay(20);
        continue;
      }
      return rc;
    }

    if (read_scale_factor)
    {
      Serial.print(F("SDP8xx: Read scale factor from sensor: "));
      Serial.println(to_le(data.scale_factor));
      scale_factor_ = 1.0f / to_le(data.scale_factor);
    }
    pressure_diff_pa = to_le(data.pressure_diff) * scale_factor_;
    temperature_c = to_le(data.temperature) / 200.0f;
    return result::ok;
  }
}

bool SDP8xx::detectSDP8xx()
{
  // Note: the sensor could be in continuous measurement mode, so first try to stop it.
  delayMicroseconds(501); // previous command could've been stop
  sendCommand(address_, sdp800_command::stop_cont);
  delayMicroseconds(501); // next command can be sent earliest after 500us

  // Now detect sensor type by sending identification commands.
  auto rc = sendCommand(address_, sdp800_command::read_pid1);
  if (rc != result::ok)
  {
    // unknown command
    Serial.print(F("SDP8xx: Couldn't send ID1 command to I2C address 0x"));
    Serial.print(address_, HEX);
    Serial.print(F(", rc="));
    Serial.println(uint8_t(rc));
    return false;
  }
  rc = sendCommand(address_, sdp800_command::read_pid2);
  if (rc != result::ok)
  {
    Serial.print(F("SDP8xx: Couldn't send ID2 command to I2C address 0x"));
    Serial.print(address_, HEX);
    Serial.print(F(", rc="));
    Serial.println(uint8_t(rc));
    return false;
  }

  // assume SDP8xx, now read identification
  type_ = type::SDP8xx;
  struct
  {
    uint32_t prod_nr;
    uint8_t serial[8];
  } data;
  rc = readReply(&data, sizeof(data));
  if (rc != result::ok)
  {
    if (rc == result::no_data)
    {
      Serial.print(F("SDP8xx: Empty reply for ID command, maybe dead SDP8xx at I2C address 0x"));
      Serial.println(address_, HEX);
      return false;
    }
    else
    {
      Serial.print(F("SDP8xx: Couldn't read reply of ID command, maybe dead SDP8xx at I2C address 0x"));
      Serial.print(address_, HEX);
      Serial.print(F(", rc="));
      Serial.println(uint8_t(rc));
      return false;
    }
  }
  Serial.print(F("SDP8xx: found SDP8"));
  switch (to_le(data.prod_nr))
  {
  case sdp800_pid::SDP800_125:
    Serial.print(F("00-125"));
    scale_factor_ = 1/240.0;
    break;
  case sdp800_pid::SDP810_125:
    Serial.print(F("10-125"));
    scale_factor_ = 1/240.0;
    break;
  case sdp800_pid::SDP800_500:
    Serial.print(F("00-500"));
    scale_factor_ = 1/60.0;
    break;
  case sdp800_pid::SDP810_500:
    Serial.print(F("10-500"));
    scale_factor_ = 1/60.0;
    break;
   case sdp800_pid::SDP801_500:
    Serial.print(F("01-500"));
    scale_factor_ = 1/60.0;
    break;
  case sdp800_pid::SDP811_500:
    Serial.print(F("11-500"));
    scale_factor_ = 1/60.0;
    break;
  default:
    Serial.print(F("?? (unknown 0x"));
    Serial.print(to_le(data.prod_nr), HEX);
    Serial.print(')');
    scale_factor_ = NAN; // autodetect
    break;
  }
  Serial.print(F(" at I2C address 0x"));
  Serial.print(address_, HEX);
  Serial.print(F(", serial "));
  for (uint8_t i = 0; i < 8; ++i)
  {
    if (i)
      Serial.print(':');
    Serial.print(data.serial[i], HEX);
  }
  Serial.println();

  return true;
}

SDP8xx::result SDP8xx::setSensorRange(uint16_t range_pa)
{
  if (range_pa < 25)
    return result::invalid_parameter;
  else if (range_pa > 500)
    return result::invalid_parameter;
  scale_factor_ = range_pa / 30000.0f;
  return result::ok;
}

bool SDP8xx::detectSDP8xxViaRead()
{
  // Note: the sensor could be in continuous measurement mode, so first try to stop it.
  delayMicroseconds(501); // previous command could've been stop
  sendCommand(address_, sdp800_command::stop_cont);
  delayMicroseconds(501); // next command can be sent earliest after 500us

  type_ = type::SDP8xx;
  scale_factor_ = NAN;
  float diff, temp;
  if (read(diff, temp) == result::ok)
  {
    Serial.print(F("SDP8xx: Detected SDP8xx sensor via read at address 0x"));
    Serial.println(address_, HEX);
    return true;
  }

  type_ = type::invalid;
  return false;
}

bool SDP8xx::detectSDP6xxViaRead()
{
  // TODO specs don't say when we can read, but SDP8xx uses 500us, so let's wait 1ms
  // to be on the safe side
  delay(1);
  sendCommand(address_, sdp600_command::soft_reset);
  delay(1);

  type_ = type::SDP6xx;
  scale_factor_ = 1/240.0;
  float diff, temp;
  if (read(diff, temp) == result::ok)
  {
    Serial.print(F("SDP8xx: Detected SDP6xx sensor via read at address 0x"));
    Serial.println(address_, HEX);
    return true;
  }

  type_ = type::invalid;
  return false;
}

SDP8xx::result SDP8xx::readReply(void* dest, const uint8_t size)
{
  if (type_ == type::invalid)
    return result::sensor_not_found;
  if (size & 1)
    return result::invalid_parameter;
  const uint8_t rsize = size + (size >> 1);

  Wire.requestFrom(address_, rsize);
  if (rsize > 0 && Wire.available() == 0)
  {
    // we didn't get any data
    return result::no_data;
  }
  if (Wire.available() < rsize)
  {
    // we didn't get enough data
    return result::short_data;
  }

  // store the result and verify checksum
  uint8_t* res = reinterpret_cast<uint8_t*>(dest);
  uint8_t crc_base = (type_ == type::SDP8xx) ? 0xff : 0;
  for (uint8_t i = 0; i < (size >> 1); ++i)
  {
    *res++ = uint8_t(Wire.read());
    *res++ = uint8_t(Wire.read());
    auto checksum = crc(crc_base, res - 2, 2);
    auto expected = uint8_t(Wire.read());
    if (checksum != expected)
    {
      Serial.print(F("SDP8xx: CRC error reading I2C data, checksum: "));
      Serial.print(checksum);
      Serial.print(F(", expected: "));
      Serial.print(expected);
      Serial.print(F(", at word "));
      Serial.println(i);
      return result::checksum_error;
    }
  }
  return result::ok;
}
