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

/*!
 * @file
 * @brief Controller for the ventilation system.
 */
#pragma once

#include <MicroNTP.h>

#include "NetworkClient.h"
#include "TempSensors.h"
#include "FanControl.h"
#include "Antifreeze.h"
#include "ProgramManager.h"
#include "SummerBypass.h"
#include "AdditionalSensors.h"
#ifndef NO_TFT 
#include "TFT.h"
#endif
#ifdef PLUGGIT_PRESSURE_SENSORS
#include "PressureSensors.h"
#endif

/*!
 * @brief Controller for the ventilation system.
 *
 * This class comprises all modules for the control of the ventilation system.
 */
class KWLControl : private FanControl::SetSpeedCallback, private MessageHandler
{
public:
  /// Fan 1 is not working.
  static constexpr unsigned ERROR_BIT_FAN1    = 0x0001;
  /// Fan 2 is not working.
  static constexpr unsigned ERROR_BIT_FAN2    = 0x0002;
  /// A crash report is present (restarted by watchdog).
  static constexpr unsigned ERROR_BIT_CRASH   = 0x0004;
  /// No NTP time yet.
  static constexpr unsigned ERROR_BIT_NTP     = 0x0008;
  /// T1 sensor is not working.
  static constexpr unsigned ERROR_BIT_T1      = 0x0010;
  /// T2 sensor is not working.
  static constexpr unsigned ERROR_BIT_T2      = 0x0020;
  /// T3 sensor is not working.
  static constexpr unsigned ERROR_BIT_T3      = 0x0040;
  /// T4 sensor is not working.
  static constexpr unsigned ERROR_BIT_T4      = 0x0080;  

  /// Mask to extract information type from info bits.
  static constexpr unsigned INFO_TYPE_MASK    = 0xff00;
  /// Mask to extract information value from info bits.
  static constexpr unsigned INFO_VALUE_MASK   = 0x00ff;
  /// Calibration in progress, value == calibration mode.
  static constexpr unsigned INFO_CALIBRATION  = 0x0100;
  /// Preheater in use, value == strength in %.
  static constexpr unsigned INFO_PREHEATER    = 0x0200;
  /// Antifreeze in use, fan is off, value == 0 normal, 1 == fireplace (both off).
  static constexpr unsigned INFO_ANTIFREEZE   = 0x0300;
  /// Bypass is opening or closing, value == 0 for closing, 1 for opening.
  static constexpr unsigned INFO_BYPASS       = 0x0400;

  KWLControl();

  /// Start the controller.
  void begin(Print& initTracer);

  /// Get persistent configuration.
  KWLPersistentConfig& getPersistentConfig() { return persistent_config_; }

  /// Get network client.
  NetworkClient& getNetworkClient() { return network_client_; }

  /// Get temperature sensor array.
  TempSensors& getTempSensors() { return temp_sensors_; }

  /// Get additional sensor array (optional ones).
  AdditionalSensors& getAdditionalSensors() { return add_sensors_; }

#ifdef PLUGGIT_PRESSURE_SENSORS
  /// Get pressure sensor array (optional ones).
  PressureSensors& getPressureSensors() { return press_sensors_; }
#endif
  
  /// Get fan controlling object.
  FanControl& getFanControl() { return fan_control_; }

  /// Get bypass controlling object.
  SummerBypass& getBypass() { return bypass_; }

  /// Get antifreeze control.
  Antifreeze& getAntifreeze() { return antifreeze_; }

  /// Get NTP client.
  MicroNTP& getNTP() { return ntp_; }

#ifndef NO_TFT 
  /// Get TFT controller.
  TFT& getTFT() { return tft_; }
#endif

  /// Get set of ERROR_BIT_* bits to describe any error situations.
  unsigned getErrors() const { return errors_; }

  /*!
   * @brief Materialize error message to the provided buffer.
   *
   * @param buffer,size buffer where to materialize the message.
   */
  void errorsToString(char* buffer, size_t size);

  /// Get set of INFO_* status to describe any additional information.
  unsigned getInfos() const { return info_; }

  /*!
   * @brief Materialize info message to the provided buffer.
   *
   * @param buffer,size buffer where to materialize the message.
   */
  void infosToString(char* buffer, size_t size);

  /// Called from main loop to run everything.
  void loop();

private:
  virtual void fanSpeedSet() override;

  virtual bool mqttReceiveMsg(const StringView& topic, const StringView& s) override;

  void run();

  /// Send status bits.
  void mqttSendStatus();

  /// Called by watchdog to report deadlock.
  static void deadlockDetected(unsigned long pc, unsigned sp, void* arg);

  /// Scheduler for running tasks.
  Scheduler::PollingScheduler scheduler_;
  /// Persistent configuration.
  KWLPersistentConfig persistent_config_;
  /// UDP handler for NTP.
  EthernetUDP udp_;
  /// NTP protocol.
  MicroNTP ntp_;
  /// Global MQTT client.
  NetworkClient network_client_;
  /// Set of temperature sensors.
  TempSensors temp_sensors_;
  /// Additional sensors (humidity, CO2, VOC).
  AdditionalSensors add_sensors_;
#ifdef PLUGGIT_PRESSURE_SENSORS  
  /// Pressure sensors (from Pluggit).
  PressureSensors press_sensors_;
#endif  
  /// Fan control.
  FanControl fan_control_;
  /// Summer bypass object.
  SummerBypass bypass_;
  /// Antifreeze/preheater control.
  Antifreeze antifreeze_;
  /// Program manager to set daily/weekly programs.
  ProgramManager program_manager_;
#ifndef NO_TFT 
  /// Display control.
  TFT tft_;
#endif
  /// Task to send all scheduler infos reliably.
  PublishTask scheduler_publish_;
  /// Task to send errors.
  PublishTask error_publish_;
  /// Current error state.
  unsigned errors_ = 0;
  /// Current info state.
  unsigned info_ = 0;
  /// Main control timing statistics.
  Scheduler::TaskTimingStats control_stats_;
  /// Timer firing checks.
  Scheduler::TimedTask<KWLControl> control_timer_;
};
