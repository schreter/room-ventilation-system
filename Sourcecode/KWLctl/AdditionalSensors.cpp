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

#include "AdditionalSensors.h"
#include "KWLConfig.h"
#include "MessageHandler.h"
#include "MQTTTopic.hpp"
#include "SDP8xx.h"

#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>

namespace
{
  /// Address of the I2C multiplexer.
  static constexpr uint8_t TCAADDR = 0x70;

  /// Select channel on I2C multiplexer.
  static bool tcaselect(uint8_t chNum)
  {
    if (chNum > 3)
      return false;
    byte ctrl = 0x04 | (chNum & 0x7);
    Wire.beginTransmission(TCAADDR);
    Wire.write(ctrl);
    return Wire.endTransmission() == 0;
  }
}

// Definitionen für das Scheduling

/// Interval between two DHT sensor readings (10s).
static constexpr unsigned long INTERVAL_DHT_READ              = 10000000;
/// Interval between two CO2 sensor readings (10s).
static constexpr unsigned long INTERVAL_MHZ14_READ            = 10000000;
/// Time between VOC sensor readings (1s).
static constexpr unsigned long INTERVAL_TGS2600_READ          =  1000000;
/// Time between pressure sensor readings (1s).
static constexpr unsigned long INTERVAL_DP_READ               =  1000000;

/// Minimum time between communicating DHT values.
static constexpr unsigned long INTERVAL_MQTT_DHT              =  5000000;
/// Maximum time between communicating DHT values.
static constexpr unsigned long INTERVAL_MQTT_DHT_FORCE        = 300000000; // 5 * 60 * 1000; 5 Minuten
/// Minimum time between communicating differential pressure values.
static constexpr unsigned long INTERVAL_MQTT_DP               =  5000000;
/// Maximum time between communicating differential pressure values.
static constexpr unsigned long INTERVAL_MQTT_DP_FORCE         = 300000000; // 5 * 60 * 1000; 5 Minuten
/// Minimum time between communicating CO2 values.
static constexpr unsigned long INTERVAL_MQTT_MHZ14            = 60000000;
/// Maximum time between communicating CO2 values.
static constexpr unsigned long INTERVAL_MQTT_MHZ14_FORCE      = 300000000; // 5 * 60 * 1000; 5 Minuten
/// Minimum time between communicating VOC values.
static constexpr unsigned long INTERVAL_MQTT_TGS2600          =  5000000;
/// Maximum time between communicating VOC values.
static constexpr unsigned long INTERVAL_MQTT_TGS2600_FORCE    = 30000000;

// DHT Sensoren
static DHT_Unified dht1(KWLConfig::PinDHTSensor1, DHT22);
static DHT_Unified dht2(KWLConfig::PinDHTSensor2, DHT22);

// Differential pressure sensors (SDP8xx/SDP6xx on I2C)
static SDP8xx dp1;
static SDP8xx dp2;

// TGS2600
static constexpr float  TGS2600_DEFAULTPPM        = 10;           //default ppm of CO2 for calibration
static constexpr long   TGS2600_DEFAULTRO         = 45000;        //default Ro for TGS2600_DEFAULTPPM ppm of CO2
static constexpr double TGS2600_SCALINGFACTOR     = 0.3555567714; //CO2 gas value
static constexpr double TGS2600_EXPONENT          = -3.337882361; //CO2 gas value
//static constexpr double TGS2600_MAXRSRO           = 2.428;        //for CO2
//static constexpr double TGS2600_MINRSRO           = 0.358;        //for CO2
static constexpr double TGS2600_RL                = 1000;

// **************************** CO2 Sensor MH-Z14 ******************************************
static const uint8_t cmdReadGasPpm[9]   = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
//static const uint8_t cmdCalZeroPoint[9] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};
//static constexpr int Co2Min = 402;

//static char getChecksum(char *packet) {
//  char i, checksum;
//  checksum = 0;
//  for (i = 1; i < 8; i++) {
//    checksum += packet[i];
//  }
//  checksum = 0xff - checksum;
//  checksum += 1;
//  return checksum;
//}

// ----------------------------- TGS2600 ------------------------------------

// TODO there are many warnings when computing the value. For now, added
// explicit casts to proper types and/or cleaned up types, but it needs
// to be checked.

/*
   get the calibrated ro based upon read resistance, and a know ppm
*/
static double TGS2600_getro(double resvalue, float ppm) {
  return resvalue * exp(log(TGS2600_SCALINGFACTOR / double(ppm)) / TGS2600_EXPONENT);
}

/*
   get the ppm concentration
*/
static double TGS2600_getppm(double resvalue, long ro) {
  return TGS2600_SCALINGFACTOR * pow((resvalue / ro), TGS2600_EXPONENT);
}

static int calcSensor_VOC(int valr)
{
  double val;
  double TGS2600_ro = 0;  // unused except for debugging
  double val_voc = 0;

  //Serial.println(valr);
  if (valr > 0) {
    val =  (TGS2600_RL * (1024 - valr) / valr);
    TGS2600_ro = TGS2600_getro(val, TGS2600_DEFAULTPPM);
    //convert to ppm (using default ro)
    val_voc = TGS2600_getppm(val, TGS2600_DEFAULTRO);
  }
  if (KWLConfig::serialDebugSensor) {
    Serial.print ( F("Vrl / Rs / ratio:"));
    Serial.print ( val);
    Serial.print ( F(" / "));
    Serial.print ( TGS2600_ro);
    Serial.print ( F(" / "));
    Serial.println (val_voc);
  }
  return int(val_voc);
}

// ----------------------------- TGS2600 END --------------------------------


AdditionalSensors::AdditionalSensors() :
  MessageHandler(F("AdditionalSensors")),
  stats_(F("AdditionalSensors")),
  dht1_read_(stats_, &AdditionalSensors::readDHT1, *this),
  dht2_read_(stats_, &AdditionalSensors::readDHT2, *this),
  mhz14_read_(stats_, &AdditionalSensors::readMHZ14, *this),
  voc_read_(stats_, &AdditionalSensors::readVOC, *this),
  dp_read_(stats_, &AdditionalSensors::readDP, *this),
  dht_send_task_(stats_, &AdditionalSensors::sendDHT, *this, false),
  dht_send_oversample_task_(stats_, &AdditionalSensors::sendDHT, *this, true),
  co2_send_task_(stats_, &AdditionalSensors::sendCO2, *this, false),
  co2_send_oversample_task_(stats_, &AdditionalSensors::sendCO2, *this, true),
  voc_send_task_(stats_, &AdditionalSensors::sendVOC, *this, false),
  voc_send_oversample_task_(stats_, &AdditionalSensors::sendVOC, *this, true),
  dp_send_task_(stats_, &AdditionalSensors::sendDP, *this, false),
  dp_send_oversample_task_(stats_, &AdditionalSensors::sendDP, *this, true)
{}

bool AdditionalSensors::setupMHZ14()
{
  MHZ14_available_ = false;

  uint8_t response[9];

  KWLConfig::SerialMHZ14.begin(9600);
  delay(100);
  // Folgende zwei Zeilen sind notwendig, damit der Sensor nach Anlegen der Spannung erkannt wird, ansonsten wird er nur nach Reset erkannt.
  KWLConfig::SerialMHZ14.write(cmdReadGasPpm, 9);
  KWLConfig::SerialMHZ14.readBytes(response, 9);
  delay(100);
  KWLConfig::SerialMHZ14.write(cmdReadGasPpm, 9);
  if (KWLConfig::SerialMHZ14.readBytes(response, 9) == 9) {
    int responseHigh = response[2];
    int responseLow = response[3];
    int ppm = (256 * responseHigh) + responseLow;
    if (KWLConfig::serialDebugSensor) {
      Serial.print(F("CO2 ppm: "));
      Serial.println(ppm);
    }
    if (ppm > 0) {
      mhz14_read_.runRepeated(INTERVAL_MHZ14_READ);
      MHZ14_available_ = true;
    }
  }
  return MHZ14_available_;
}

bool AdditionalSensors::setupTGS2600()
{
  pinMode(KWLConfig::PinVocSensor, INPUT_PULLUP);
  analogRead(KWLConfig::PinVocSensor);  // discard a read to get more stable reading
  int analogVal = analogRead(KWLConfig::PinVocSensor);
  if (KWLConfig::serialDebugSensor) Serial.println(analogVal);
  if (analogVal < 1020 /* some reserve for not exact analog read of empty pin */) {
    voc_read_.runRepeated(INTERVAL_TGS2600_READ, INTERVAL_TGS2600_READ);
    TGS2600_available_ = true;
  } else {
    TGS2600_available_ = false;
  }
  return TGS2600_available_;
}

void AdditionalSensors::readDHT1()
{
  sensors_event_t event;

  dht1.temperature().getEvent(&event);
  if (isnanf(event.temperature)) {
    if (!KWLConfig::SendErroneousMeasurement)
      dht1_temp_ = event.temperature;
    Serial.println(F("Failed reading temperature from DHT1"));
  } else if (event.temperature != dht1_temp_) {
    dht1_temp_ = event.temperature;
    if (KWLConfig::serialDebugSensor) {
      Serial.print(F("DHT1 T: "));
      Serial.println(dht1_temp_);
    }
  }

  dht1.humidity().getEvent(&event);
  if (isnanf(event.relative_humidity)) {
    if (!KWLConfig::SendErroneousMeasurement)
      dht1_hum_ = event.relative_humidity;
    Serial.println(F("Failed reading humidity from DHT1"));
  } else if (event.relative_humidity != dht1_hum_) {
    dht1_hum_ = event.relative_humidity;
    if (KWLConfig::serialDebugSensor) {
      Serial.print(F("DHT1 H: "));
      Serial.println(dht1_hum_);
    }
  }
}

void AdditionalSensors::readDHT2()
{
  sensors_event_t event;

  dht2.temperature().getEvent(&event);
  if (isnanf(event.temperature)) {
    if (!KWLConfig::SendErroneousMeasurement)
      dht2_temp_ = event.temperature;
    Serial.println(F("Failed reading temperature from DHT2"));
  } else if (event.temperature != dht2_temp_) {
    dht2_temp_ = event.temperature;
    if (KWLConfig::serialDebugSensor) {
      Serial.print(F("DHT2 T: "));
      Serial.println(dht2_temp_);
    }
  }

  dht2.humidity().getEvent(&event);
  if (isnanf(event.relative_humidity)) {
    if (!KWLConfig::SendErroneousMeasurement)
      dht2_hum_ = event.relative_humidity;
    Serial.println(F("Failed reading humidity from DHT2"));
  } else if (event.relative_humidity != dht2_hum_) {
    dht2_hum_ = event.relative_humidity;
    if (KWLConfig::serialDebugSensor) {
      Serial.print(F("DHT2 H: "));
      Serial.println(dht2_hum_);
    }
  }
}

void AdditionalSensors::readMHZ14()
{
  uint8_t response[9];
  KWLConfig::SerialMHZ14.write(cmdReadGasPpm, 9);
  if (KWLConfig::SerialMHZ14.readBytes(response, 9) == 9) {
    int responseHigh = response[2];
    int responseLow = response[3];
    int ppm = (256 * responseHigh) + responseLow;

    if (KWLConfig::serialDebugSensor) {
      Serial.print(F("CO2 ppm: "));
      Serial.println(ppm);
    }
    // Automatische Kalibrieren des Nullpunktes auf den kleinstmöglichen Wert
    //if (ppm < Co2Min)
    //  KWLConfig::SerialMHZ14.write(cmdCalZeroPoint, 9);

    co2_ppm_ = ppm;
  } else {
    co2_ppm_ = -1000;
  }
}

void AdditionalSensors::readVOC()
{
  analogRead(KWLConfig::PinVocSensor);  // discard a read to get more stable reading
  int analogVal = analogRead(KWLConfig::PinVocSensor);
  voc_ = calcSensor_VOC(analogVal);
  if (KWLConfig::serialDebugSensor) {
    Serial.print(F("VOC analogVal: "));
    Serial.print(analogVal);
    Serial.print(F(", ppm="));
    Serial.println(voc_);
  }
}

void AdditionalSensors::readDP()
{
  float temp1, temp2;
  tcaselect(0);
  dp1.read(dp1_, temp1);
  tcaselect(1);
  dp2.read(dp2_, temp2);
  if (KWLConfig::serialDebugSensor) {
    Serial.print(F("DP1 P="));
    Serial.print(dp1_);
    Serial.print(F(", T="));
    Serial.print(temp1);
    Serial.print(F("; DP2 P="));
    Serial.print(dp1_);
    Serial.print(F(", T="));
    Serial.println(temp2);
  }
}

void AdditionalSensors::begin(Print& initTracer)
{
  initTracer.print(F("Initialisierung Sensoren:"));
  // DHT Sensoren
  dht1.begin();
  dht2.begin();
  delay(1500);
  sensors_event_t event;
  dht1.temperature().getEvent(&event);
  if (!isnan(event.temperature)) {
    DHT1_available_ = true;
    initTracer.print(F(" DHT1"));
    dht1_read_.runRepeated(INTERVAL_DHT_READ);
  }
  dht2.temperature().getEvent(&event);
  if (!isnan(event.temperature)) {
    DHT2_available_ = true;
    initTracer.print(F(" DHT2"));
    dht2_read_.runRepeated(INTERVAL_DHT_READ);
  }
  if (DHT1_available_ || DHT2_available_) {
    dht_send_task_.runRepeated(INTERVAL_DHT_READ + 1000000, INTERVAL_MQTT_DHT);
    dht_send_oversample_task_.runRepeated(INTERVAL_DHT_READ + 1000000, INTERVAL_MQTT_DHT_FORCE);
  }

  // MH-Z14 CO2 Sensor
  if (setupMHZ14()) {
    initTracer.print(F(" CO2"));
    co2_send_task_.runRepeated(INTERVAL_MHZ14_READ + 1000000, INTERVAL_MQTT_MHZ14);
    co2_send_oversample_task_.runRepeated(INTERVAL_MHZ14_READ + 1000000, INTERVAL_MQTT_MHZ14_FORCE);
  }

  // TGS2600 VOC Sensor
  if (setupTGS2600()) {
    initTracer.print(F(" VOC"));
    voc_send_task_.runRepeated(INTERVAL_MQTT_TGS2600 + 1000000, INTERVAL_MQTT_TGS2600);
    voc_send_oversample_task_.runRepeated(INTERVAL_MQTT_TGS2600 + 1000000, INTERVAL_MQTT_TGS2600_FORCE);
  }

  // Differential pressure sensors
  if (tcaselect(0))
  {
    bool has_dp1 = dp1.begin();
    tcaselect(1);
    bool has_dp2 = dp2.begin();
    if (has_dp1 && has_dp2)
    {
      initTracer.print(F(" DP"));
      dp_read_.runRepeated(INTERVAL_DP_READ, INTERVAL_DP_READ);
      dp_send_task_.runRepeated(INTERVAL_DP_READ + 1000000, INTERVAL_MQTT_DP);
      dp_send_oversample_task_.runRepeated(INTERVAL_DP_READ + 1000000, INTERVAL_MQTT_DP_FORCE);
    }
    else if (has_dp1 || has_dp2)
    {
      if (has_dp1)
        initTracer.print(F(" DP1 !DP2"));
      else
        initTracer.print(F(" !DP1 DP2"));
    }
    else
    {
      initTracer.print(F(" !DP"));
    }
  }

  if (!DHT1_available_ && !DHT2_available_ && !MHZ14_available_ && !TGS2600_available_ &&
      dp1.getType() == SDP8xx::type::invalid && dp2.getType() == SDP8xx::type::invalid) {
    initTracer.println(F(" keine Sensoren"));
  } else {
    initTracer.println();
  }
}

void AdditionalSensors::forceSend() noexcept
{
  if (DHT1_available_ || DHT2_available_)
    sendDHT(true);
  if (MHZ14_available_)
    sendCO2(true);
  if (TGS2600_available_)
    sendVOC(true);
  if (hasDP())
    sendDP(true);
}

bool AdditionalSensors::hasDP() const noexcept
{
  return (dp1.getType() != SDP8xx::type::invalid && dp2.getType() != SDP8xx::type::invalid);
}

bool AdditionalSensors::updateDP() noexcept
{
  readDP();
  return !isnanf(dp1_) && !isnanf(dp2_);
}

void AdditionalSensors::sendDHT(bool force) noexcept
{
  if (!force
      && (abs(dht1_temp_ - dht1_last_sent_temp_) < 0.1f) && (abs(dht2_temp_ - dht2_last_sent_temp_) < 0.1f)
      && (abs(dht1_hum_ - dht2_last_sent_hum_) < 1) && (abs(dht2_hum_ - dht2_last_sent_hum_) < 1)) {
    // not enough change, no point to send
    return;
  }
  dht1_last_sent_temp_ = dht1_temp_;
  dht1_last_sent_hum_  = dht1_hum_;
  dht2_last_sent_temp_ = dht2_temp_;
  dht2_last_sent_hum_  = dht2_hum_;
  uint8_t bitmap = (DHT1_available_ ? 0x3 : 0) | (DHT2_available_ ? 0xc: 0);
  publish_dht_.publish([this, bitmap]() mutable {
    uint8_t bit = 1;
    while (bit < 16) {
      if (bitmap & bit) {
        bool res = true;
        switch (bit) {
          case 1: res = isnanf(dht1_temp_) || MessageHandler::publish(MQTTTopic::KwlDHT1Temperatur, dht1_temp_, 1, KWLConfig::RetainAdditionalSensors); break;
          case 2: res = isnanf(dht1_hum_)  || MessageHandler::publish(MQTTTopic::KwlDHT1Humidity, dht1_hum_, 1, KWLConfig::RetainAdditionalSensors); break;
          case 4: res = isnanf(dht2_temp_) || MessageHandler::publish(MQTTTopic::KwlDHT2Temperatur, dht2_temp_, 1, KWLConfig::RetainAdditionalSensors); break;
          case 8: res = isnanf(dht2_hum_)  || MessageHandler::publish(MQTTTopic::KwlDHT2Humidity, dht2_hum_, 1, KWLConfig::RetainAdditionalSensors); break;
          default: return true; // paranoia
        }
        if (!res)
          return false; // will retry later
        bitmap &= ~bit;
      }
      bit <<= 1;
    }
    return true;  // all sent
  });
  dht_send_task_.runRepeated(INTERVAL_MQTT_DHT);
  dht_send_oversample_task_.runRepeated(INTERVAL_MQTT_DHT_FORCE);
}

void AdditionalSensors::sendCO2(bool force) noexcept
{
  if (!force && (abs(co2_ppm_ - co2_last_sent_ppm_) < 20)) {
    // not enough change
    return;
  }
  if (co2_ppm_ >= 0)
    publish_co2_.publish(MQTTTopic::KwlCO2Abluft, co2_ppm_, KWLConfig::RetainAdditionalSensors);
  else if (KWLConfig::SendErroneousMeasurement)
    publish_co2_.publish(MQTTTopic::KwlCO2Abluft, -1, KWLConfig::RetainAdditionalSensors);
  co2_send_task_.runRepeated(INTERVAL_MQTT_MHZ14);
  co2_send_oversample_task_.runRepeated(INTERVAL_MQTT_MHZ14_FORCE);
}

void AdditionalSensors::sendVOC(bool force) noexcept
{
  if (!force && (abs(voc_ - voc_last_sent_) < 20)) {
    // not enough change
    return;
  }
  publish_voc_.publish(MQTTTopic::KwlVOCAbluft, voc_, 1, KWLConfig::RetainAdditionalSensors);
  voc_send_task_.runRepeated(INTERVAL_MQTT_TGS2600);
  voc_send_oversample_task_.runRepeated(INTERVAL_MQTT_TGS2600_FORCE);
}

void AdditionalSensors::sendDP(bool force) noexcept
{
  if (!force && (isnanf(dp1_last_sent_) || isnanf(dp2_last_sent_)))
    force = true;
  if (!force && (abs(dp1_ - dp1_last_sent_) < 1) && (abs(dp2_ - dp2_last_sent_) < 1)) {
    // not enough change
    return;
  }
  dp1_last_sent_ = dp1_;
  dp2_last_sent_ = dp2_;
  uint8_t bitmap = 3;
  publish_dp_.publish([this, bitmap]() mutable {
    uint8_t bit = 1;
    while (bit < 4) {
      if (bitmap & bit) {
        bool res = true;
        switch (bit) {
          case 1: res = isnanf(dp1_) || MessageHandler::publish(MQTTTopic::KwlDP1Pressure, dp1_, 1, KWLConfig::RetainAdditionalSensors); break;
          case 2: res = isnanf(dp2_) || MessageHandler::publish(MQTTTopic::KwlDP2Pressure, dp2_, 1, KWLConfig::RetainAdditionalSensors); break;
          default: return true; // paranoia
        }
        if (!res)
          return false; // will retry later
        bitmap &= ~bit;
      }
      bit <<= 1;
    }
    return true;  // all sent
  });
  dp_send_task_.runRepeated(INTERVAL_MQTT_DP);
  dp_send_oversample_task_.runRepeated(INTERVAL_MQTT_DP_FORCE);
}

bool AdditionalSensors::mqttReceiveMsg(const StringView& topic, const StringView& s)
{
#ifdef DEBUG
  if (topic == MQTTTopic::KwlDebugsetDP1) {
    if (s == MQTTTopic::ValueMeasure) {
      if (tcaselect(0)) {
        Serial.println(F("Restarting measurement on DP1 sensor"));
        dp1.begin();
      } else {
        Serial.println(F("Cannot restart DP1 sensor, I2C switch not available"));
      }
    } else {
      dp1.beginSimulated();
      Serial.print(F("Starting simulation of DP1 sensor with pressure "));
      if (s != MQTTTopic::ValueSimulate)
        dp1_ = s.toFloat();
      Serial.println(double(dp1_));
    }
    sendDP(true);
  } else if (topic == MQTTTopic::KwlDebugsetDP2) {
    if (s == MQTTTopic::ValueMeasure) {
      if (tcaselect(1)) {
        Serial.println(F("Restarting measurement on DP2 sensor"));
        dp2.begin();
      } else {
        Serial.println(F("Cannot restart DP2 sensor, I2C switch not available"));
      }
    } else {
      dp2.beginSimulated();
      Serial.print(F("Starting simulation of DP2 sensor with pressure "));
      if (s != MQTTTopic::ValueSimulate)
        dp2_ = s.toFloat();
      Serial.println(double(dp2_));
    }
    sendDP(true);
  } else {
    return false;
  }
  return true;
#else
  return false;
#endif
}
