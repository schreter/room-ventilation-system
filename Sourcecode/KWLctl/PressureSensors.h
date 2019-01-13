/*
 * Copyright (C) 2018 Sven Just (sven@familie-just.de)
 * Copyright (C) 2018 Ivan Schréter (schreter@gmx.net)
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

#include "Arduino.h"

// IMPORTANT: Must define PLUGGIT_PRESSURE_SENSORS in UserConfig.h to activate the sensor reading!

// Config values:
// KWLConfig::PinPressureSensor1
// KWLConfig::PinPressureSensor2
// KWLConfig::PressureSensorScale
// KWLConfig::PressureSensorOffset
// KWLConfig::PressureSensorAdScale 
// KWLConfig::RetainPressureSensors

namespace PressureSensorConfig {
 constexpr auto  PinPressureSensor1 = A2;
 constexpr auto  PinPressureSensor2 = A3;
 constexpr auto  PressureSensorScale = 10.0;
 constexpr auto  PressureSensorOffset = 2.0;
 constexpr auto  PressureSensorAdScale = 2.0;
 constexpr auto  RetainPressureSensors = true;
};

/*!
 * @file
 * @brief Pressure sensors of the ventilation system (optional).
 */
#pragma once

#include "TimeScheduler.h"

/*!
 * @brief Pressure sensors of the ventilation system (optional).
 *
 * This class reads and publishes values of the Pluggit pressure sensors.
 */
class PressureSensors
{
public:
  PressureSensors();

  /// Initialize sensors.
  void begin(Print& initTracer);

  /// Force sending values via MQTT on the next MQTT run.
  void forceSend() noexcept;

  /// Check if pressure sensor S1 is present.
  bool hasS1() const noexcept { return S1_available_; }

  /// Get S1 pressure value.
  float getS1Press() const noexcept { return s1_press_; }

  /// Get S1 voltage.
  float getS1Volt() const noexcept { return s1_volt_; }

  /// Check if pressure sensor S2 sensor is present.
  bool hasS2() const noexcept { return S2_available_; }

  /// Get S2 pressure value.
  float getS2Press() const noexcept { return s2_press_; }

  /// Get S2 voltage.
  float getS2Volt() const noexcept { return s2_volt_; }

private:
  /// Set up S1 sensor.
  bool setupS1();
  /// Set up S2 sensor.
  bool setupS2();

  /// Read value of pressure sensors.
  void readPressure();
  /// Read value of single pressure sensor.
  void read(const char* sensorName, int pin, float& voltage, float& pressure);

  /// Schedule sending pressure values now.
  void sendPressue(bool force) noexcept;

  // sensor availability
  bool S1_available_ = false;
  bool S2_available_ = false;

  // current values
  float s1_press_ = 0;
  float s2_press_ = 0;
  float s1_volt_ = 0;
  float s2_volt_ = 0;

  // last sent values per MQTT
  float s1_last_sent_volt_ = 0;
  float s2_last_sent_volt_ = 0;

  // Tasks running on timeout
  Scheduler::TaskTimingStats stats_;
  Scheduler::TimedTask<PressureSensors> read_task_;
  Scheduler::TimedTask<PressureSensors, bool> send_task_;
};
