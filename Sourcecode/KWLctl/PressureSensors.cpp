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

#include "PressureSensors.h"
#include "KWLConfig.h"
#include "MessageHandler.h"
#include "MQTTTopic.hpp"

// Definitionen für das Scheduling

/// Interval between two pressure sensor readings (1s).
static constexpr unsigned long INTERVAL_PRESSURE_READ         = 1000000;
/// Minimum time between communicating pressure sensor values.
static constexpr unsigned long INTERVAL_MQTT_PRESSURE         = 5000000;

// ----------------------------- TGS2600 END --------------------------------

PressureSensors::PressureSensors() :
  stats_(F("PressureSensors")),
  read_task_(stats_, &PressureSensors::readPressure, *this),
  send_task_(stats_, &PressureSensors::sendPressue, *this, false)
{}

bool PressureSensors::setupS1()
{
  pinMode(PressureSensorConfig::PinPressureSensor1, INPUT); 
  read("S1 pressure analogVal: ", PressureSensorConfig::PinPressureSensor1, s1_volt_, s1_press_);
  if (s1_volt_ < 9.5) {
    S1_available_ = true;
  } else {
    S1_available_ = false;
  }
  return S1_available_;
}

bool PressureSensors::setupS2()
{
  pinMode(PressureSensorConfig::PinPressureSensor2, INPUT);
  read("S2 pressure analogVal: ", PressureSensorConfig::PinPressureSensor2, s2_volt_, s2_press_);
  if (s2_volt_ < 9.5) {    
    S2_available_ = true;
  } else {
    S2_available_ = false;
  }
  return S2_available_;
}

void PressureSensors::readPressure()
{
  read("S1 pressure analogVal: ", PressureSensorConfig::PinPressureSensor1, s1_volt_, s1_press_);
  // if voltage < 2.0V -> sensor is not available
  //if (s1_volt_ < 2.0)
  //  S1_available_ = false;
  read("S2 pressure analogVal: ", PressureSensorConfig::PinPressureSensor2, s2_volt_, s2_press_);
}

void PressureSensors::read(const char* sensorName, int pin, float& voltage, float& pressure)
{
  analogRead(pin);  // discard a read to get more stable reading
  int analogVal = analogRead(pin);
  if (KWLConfig::serialDebugSensor) {
    Serial.print(sensorName); Serial.print(": ");
    Serial.println(analogVal);
  }
  voltage = PressureSensorConfig::PressureSensorAdScale * analogVal * (5.0 / 1023.0);
  // if voltage < 2.0V -> sensor is not available
  if (voltage < 2.0)
    pressure = 0.0;
  else
    pressure = (voltage-PressureSensorConfig::PressureSensorOffset)*PressureSensorConfig::PressureSensorScale;
}

void PressureSensors::begin(Print& initTracer)
{
  initTracer.print(F("Initialisierung Drucksensoren:"));
  // S1 pressure Sensor
  if (setupS1()) {
    initTracer.print(F(" S1"));
  }  
  // S2 pressure Sensor_read
  if (setupS2()) {
    initTracer.print(F(" S2"));    
  }  
  if (!S1_available_ && !S2_available_) {
    initTracer.println(F(" keine Sensoren"));
  } else {
    initTracer.println();
    read_task_.runRepeated(0, INTERVAL_PRESSURE_READ);
    send_task_.runRepeated(1500, INTERVAL_MQTT_PRESSURE);
  }
}

void PressureSensors::forceSend() noexcept
{
  if (S1_available_ && S2_available_)
    sendPressue(true);
}

void PressureSensors::sendPressue(bool force) noexcept
{
  static int send_suppressed = 0;
  if ((send_suppressed < 10) && !force
      && (abs(s1_volt_ - s1_last_sent_volt_) < 0.1f) && (abs(s2_volt_ - s2_last_sent_volt_) < 0.1f)
      ) {
    // not enough change, no point to send
    send_suppressed++;
    return;
  }
  s1_last_sent_volt_ = s1_volt_;
  s2_last_sent_volt_  = s2_volt_;
  MessageHandler::publish(MQTTTopic::KwlPressure1Voltage,   s1_volt_,   1, PressureSensorConfig::RetainPressureSensors);
  MessageHandler::publish(MQTTTopic::KwlPressure1Pressure,  s1_press_,  1, PressureSensorConfig::RetainPressureSensors);
  MessageHandler::publish(MQTTTopic::KwlPressure2Voltage,   s2_volt_,   1, PressureSensorConfig::RetainPressureSensors);
  MessageHandler::publish(MQTTTopic::KwlPressure2Pressure,  s2_press_,  1, PressureSensorConfig::RetainPressureSensors);  
  send_task_.runRepeated(INTERVAL_MQTT_PRESSURE);
}
