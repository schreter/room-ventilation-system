/*
  ################################################################
  #
  #   Copyright notice
  #
  #   Control software for a Room Ventilation System
  #   https://github.com/svenjust/room-ventilation-system
  #
  #   Copyright (C) 2018  Sven Just (sven@familie-just.de)
  #
  #   This program is free software: you can redistribute it and/or modify
  #   it under the terms of the GNU General Public License as published by
  #   the Free Software Foundation, either version 3 of the License, or
  #   (at your option) any later version.
  #
  #   This program is distributed in the hope that it will be useful,
  #   but WITHOUT ANY WARRANTY; without even the implied warranty of
  #   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  #   GNU General Public License for more details.
  #
  #   You should have received a copy of the GNU General Public License
  #   along with this program.  If not, see <https://www.gnu.org/licenses/>.
  #
  #   This copyright notice MUST APPEAR in all copies of the script!
  #
  ################################################################
*/

#include "KWLConfig.h"
#include "KWLControl.hpp"

#include <Adafruit_GFX.h>       // TFT
#include <MCUFRIEND_kbv.h>      // TFT
#include <TouchScreen.h>        // Touch

// Font einbinden
#include <Fonts/FreeSans9pt7b.h>  // Font
#include <Fonts/FreeSans12pt7b.h>  // Font
#include <Fonts/FreeSansBold24pt7b.h>

// ***************************************************  T T F   U N D   T O U C H  ********************************************************
// *** TFT
// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GREY    0x7BEF
uint16_t ID;
MCUFRIEND_kbv tft;

// Definition for Touch
// most mcufriend shields use these pins and Portrait mode:
// TODO move to KWLConfig
uint8_t YP = A1;  // must be an analog pin, use "An" notation!
uint8_t XM = A2;  // must be an analog pin, use "An" notation!
uint8_t YM = 7;   // can be a digital pin
uint8_t XP = 6;   // can be a digital pin

uint16_t TS_LEFT = KWLConfig::TouchLeft;
uint16_t TS_RT   = KWLConfig::TouchRight;
uint16_t TS_TOP  = KWLConfig::TouchTop;
uint16_t TS_BOT  = KWLConfig::TouchBottom;

const char *name = "Unknown controller";

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// For the one we're using, its 300 ohms across the X plate
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
TSPoint tp;
#define MINPRESSURE 20
#define MAXPRESSURE 1000
#define SWAP(a, b) {int16_t tmp = a; a = b; b = tmp;}
// ***************************************************  E N D E   T T F   U N D   T O U C H  ********************************************************


/* FARBEN


*/
/*
  #define colButton-BackColor-Pressed 0x41B3D6 //  65 179 214
  #define colButton-BackColor         0x3A9FBE //  58 159 190
  //#define colBackColor                0x000BFF //  45 127 151
  #define colBackColor                0xFFFFFF //  45 127 151
  //#define colBackColor                0x2D7F97 //  45 127 151
  #define colWindow-Title             0x42B6DA //  66 182 218
  #define colFontColor                0x000000 // 255 255 255
  //#define colFontColor                0xFFFFFF // 255 255 255
*/

// RGB in 565 Kodierung
// Colorpicker http://www.barth-dev.de/online/rgb565-color-picker/
// Weiss auf Schwarz
#define colBackColor                0x0000 //  45 127 151
#define colWindowTitleBackColor     0xFFFF //  66 182 218
#define colWindowTitleFontColor     0x0000 //  66 182 218
#define colFontColor                0xFFFF // 255 255 255
#define colErrorBackColor           0xF800 // rot
#define colInfoBackColor            0xFFE0 // gelb
#define colErrorFontColor           0xFFFF // weiss
#define colInfoFontColor            0x0000 // schwarz
#define colMenuBtnFrame             0x0000
#define colMenuBackColor            0xFFFF
#define colMenuFontColor            0x0000
#define colMenuBtnFrameHL           0xF800

/*
  // Schwarz auf weiss
  #define colBackColor                0xFFFFFF //  45 127 151
  #define colWindowTitleBackColor     0x000000 //  66 182 218
  #define colWindowTitleFontColor     0xFFFFFF //  66 182 218
  #define colFontColor                0x000000 // 255 255 255
*/

uint16_t identifier;
uint8_t Orientation = 3;    //PORTRAIT

#define       fontFactorSmall               1
#define       fontFactorBigNumber           3
int           baselineSmall               = 0;
int           baselineMiddle              = 0;
int           baselineBigNumber           = 0;
int           numberfieldheight           = 0;

char strPrint[50];

unsigned long previousMillisDisplayUpdate = 0;
unsigned long intervalDisplayUpdate       = 5000;
boolean       updateDisplayNow            = false;
int           LastDisplaySpeedTachoFan1   = 0;
int           LastDisplaySpeedTachoFan2   = 0;
int           LastDisplaykwlMode          = 0;
int           LastEffiencyKwl             = 0;
unsigned      LastErrorBits               = 0;
unsigned      LastInfoBits                = 0;
double LastDisplayT1 = 0, LastDisplayT2 = 0, LastDisplayT3 = 0, LastDisplayT4 = 0;
float LastDisplayDHT1Temp = 0, LastDisplayDHT2Temp = 0;
int LastDisplayMHZ14_CO2_ppm = 0, LastDisplayTGS2600 = 0;

byte          LastLanOk                   = true;
byte          LastMqttOk                  = true;

byte          touchBtnWidth               = 60;
byte          touchBtnHeight              = 45;
byte          touchBtnYOffset             = 30;


int16_t  x1, y1;
uint16_t w, h;

boolean       menuBtnActive[10];
byte          menuScreen                  = 0;
byte          menuBtnPressed              = -1;
byte          LastMenuBtn                 = 0;
unsigned long millisLastMenuBtnPress      = 0;
unsigned long intervalMenuBtn             = 500;
unsigned long intervalLastMillisTouch     = 60000; // eine Minute
unsigned long LastMillisTouch             = 0;

unsigned inputStandardSpeedSetpointFan1 = 0;
unsigned inputStandardSpeedSetpointFan2 = 0;

// External references; TODO refactor via explicit references to respective components

extern KWLControl kwlControl;

// forwards
void PrintScreenTitle(char* title);
void NewMenuEntry(byte mnuBtn, String mnuTxt);
void PrintMenuBtn(byte mnuBtn, String mnuTxt, long colFrame);
void gotoScreen(byte ScreenNo);
void ShowMenu();
void show_Touch_Serial();


/******************************************* Seitenverwaltung ********************************************
  Für jeden Screen müssen die folgenden Funktionen implementiert werden:
   - SetupBackgroundScreen       Zeichnet den statischen Anteil des Screens und wird nur einmal aufgerufen
   - loopDisplayUpdateScreen     Zeichnet den dynamischen Anteil und wird regelmäßig aufgerufen
   - NewMenuScreen               Definiert das Menü des Screens
   - DoMenuActionScreen          Definiert die Aktion des Menüs
*/

// ****************************************** Screen 0: HAUPTSEITE ******************************************
void SetupBackgroundScreen0() {
  // Menu Screen 0 Standardseite

  char title[] = "Lueftungsstufe";
  PrintScreenTitle(title);

  tft.setCursor(150, 140 + baselineSmall);
  tft.print(F("Temperatur"));

  tft.setCursor(270, 140 + baselineSmall);
  tft.print(F("Luefterdrehzahl"));

  tft.setCursor(18, 166 + baselineMiddle);
  tft.print(F("Aussenluft"));

  tft.setCursor(18, 192 + baselineMiddle);
  tft.print(F("Zuluft"));

  tft.setCursor(18, 218 + baselineMiddle);
  tft.print(F("Abluft"));

  tft.setCursor(18, 244 + baselineMiddle);
  tft.print(F("Fortluft"));

  tft.setCursor(18, 270 + baselineMiddle);
  tft.print(F("Wirkungsgrad"));
  // Ende Menu Screen 1
}

void loopDisplayUpdateScreen0() {
  // Menu Screen 0
  if (LastDisplaykwlMode != kwlControl.getFanControl().getVentilationMode() || updateDisplayNow) {
    // KWL Mode
    tft.setFont(&FreeSansBold24pt7b);
    tft.setTextColor(colFontColor, colBackColor);
    tft.setTextSize(2);

    tft.setCursor(200, 55 + 2 * baselineBigNumber);
    sprintf(strPrint, "%-1i", (int)kwlControl.getFanControl().getVentilationMode());
    tft.fillRect(200, 55, 60, 80, colBackColor);
    tft.print(strPrint);
    LastDisplaykwlMode = kwlControl.getFanControl().getVentilationMode();
    tft.setTextSize(1);
  }
  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(colFontColor, colBackColor);

  // Speed Fan1
  int speed1 = kwlControl.getFanControl().getFan1().getSpeed();
  if (abs(LastDisplaySpeedTachoFan1 - speed1) > 10 || updateDisplayNow) {
    LastDisplaySpeedTachoFan1 = speed1;
    tft.fillRect(280, 192, 80, numberfieldheight, colBackColor);
    sprintf(strPrint, "%5i", speed1);
    tft.getTextBounds(strPrint, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(340 - w, 192 + baselineMiddle);
    tft.print(strPrint);
  }
  // Speed Fan2
  int speed2 = kwlControl.getFanControl().getFan2().getSpeed();
  if (abs(LastDisplaySpeedTachoFan2 - speed2) > 10 || updateDisplayNow) {
    LastDisplaySpeedTachoFan2 = speed2;
    sprintf(strPrint, "%5i", speed2);
    // Debug einkommentieren
    // tft.fillRect(280, 218, 60, numberfieldheight, colWindowTitle);
    tft.fillRect(280, 218, 80, numberfieldheight, colBackColor);
    tft.getTextBounds(strPrint, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(340 - w, 218 + baselineMiddle);
    tft.print(strPrint);
  }
  // T1
  if (abs(LastDisplayT1 - kwlControl.getTempSensors().get_t1_outside()) > 0.5 || updateDisplayNow) {
    LastDisplayT1 = kwlControl.getTempSensors().get_t1_outside();
    tft.fillRect(160, 166, 80, numberfieldheight, colBackColor);
    dtostrf(kwlControl.getTempSensors().get_t1_outside(), 5, 1, strPrint);
    tft.getTextBounds(strPrint, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(240 - w, 166 + baselineMiddle);
    tft.print(strPrint);
  }
  // T2
  if (abs(LastDisplayT2 - kwlControl.getTempSensors().get_t2_inlet()) > 0.5 || updateDisplayNow) {
    LastDisplayT2 = kwlControl.getTempSensors().get_t2_inlet();
    tft.fillRect(160, 192, 80, numberfieldheight, colBackColor);
    dtostrf(kwlControl.getTempSensors().get_t2_inlet(), 5, 1, strPrint);
    tft.getTextBounds(strPrint, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(240 - w, 192 + baselineMiddle);
    tft.print(strPrint);
  }
  // T3
  if (abs(LastDisplayT3 - kwlControl.getTempSensors().get_t3_outlet()) > 0.5 || updateDisplayNow) {
    LastDisplayT3 = kwlControl.getTempSensors().get_t3_outlet();
    tft.fillRect(160, 218, 80, numberfieldheight, colBackColor);
    dtostrf(kwlControl.getTempSensors().get_t3_outlet(), 5, 1, strPrint);
    tft.getTextBounds(strPrint, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(240 - w, 218 + baselineMiddle);
    tft.print(strPrint);
  }
  // T4
  if (abs(LastDisplayT4 - kwlControl.getTempSensors().get_t4_exhaust()) > 0.5 || updateDisplayNow) {
    LastDisplayT4 = kwlControl.getTempSensors().get_t4_exhaust();
    tft.fillRect(160, 244, 80, numberfieldheight, colBackColor);
    dtostrf(kwlControl.getTempSensors().get_t4_exhaust(), 5, 1, strPrint);
    tft.getTextBounds(strPrint, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(240 - w, 244 + baselineMiddle);
    tft.print(strPrint);
  }
  // Etha Wirkungsgrad
  if (abs(LastEffiencyKwl - kwlControl.getTempSensors().getEfficiency()) > 1 || updateDisplayNow) {
    LastEffiencyKwl = kwlControl.getTempSensors().getEfficiency();
    tft.fillRect(160, 270, 80, numberfieldheight, colBackColor);
    sprintf(strPrint, "%5d %%", (int)kwlControl.getTempSensors().getEfficiency());
    tft.getTextBounds(strPrint, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(240 - w, 270 + baselineMiddle);
    tft.print(strPrint);
  }
}

void NewMenuScreen0() {
  NewMenuEntry (1, F("->"));  // Führt zur Screen 3
  NewMenuEntry (2, F("<-"));  // Führt zur Seite 2
  NewMenuEntry (3, F("+"));
  NewMenuEntry (4, F("-"));
}

void DoMenuActionScreen0() {
  // Standardseite
  switch (menuBtnPressed) {
    case 1:
      gotoScreen (2);
      break;
    case 2:
      gotoScreen (1);
      break;
    case 3:
      previousMillisDisplayUpdate = 0;
      if (kwlControl.getFanControl().getVentilationMode() < KWLConfig::StandardModeCnt - 1)  kwlControl.getFanControl().setVentilationMode(kwlControl.getFanControl().getVentilationMode() + 1);
      break;
    case 4:
      previousMillisDisplayUpdate = 0;
      if (kwlControl.getFanControl().getVentilationMode() > 0)  kwlControl.getFanControl().setVentilationMode(kwlControl.getFanControl().getVentilationMode() - 1);
      break;
    default:
      previousMillisDisplayUpdate = 0;
      break;
  }
}
// ************************************ ENDE: Screen 0  *****************************************************

// ****************************************** Screen 1: WEITERE SENSORWERTE *********************************
void SetupBackgroundScreen1() {
  // Menu Screen 1, Sensorwerte

  char title[] = "Weitere Sensorwerte";
  PrintScreenTitle(title);

  if (kwlControl.getAdditionalSensors().hasDHT1()) {
    tft.setCursor(18, 166 + baselineMiddle);
    tft.print(F("DHT1"));
  }

  if (kwlControl.getAdditionalSensors().hasDHT2()) {
    tft.setCursor(18, 192 + baselineMiddle);
    tft.print(F("DHT2"));
  }

  if (kwlControl.getAdditionalSensors().hasCO2()) {
    tft.setCursor(18, 218 + baselineMiddle);
    tft.print(F("CO2"));
  }
  if (kwlControl.getAdditionalSensors().hasVOC()) {
    tft.setCursor(18, 244 + baselineMiddle);
    tft.print(F("VOC"));
  }
}

void loopDisplayUpdateScreen1() {

  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(colFontColor, colBackColor);

  // DHT 1
  if (kwlControl.getAdditionalSensors().hasDHT1()) {
    if (abs(LastDisplayDHT1Temp - kwlControl.getAdditionalSensors().getDHT1Temp()) > 0.5 || updateDisplayNow) {
      LastDisplayDHT1Temp = kwlControl.getAdditionalSensors().getDHT1Temp();
      tft.fillRect(160, 166, 80, numberfieldheight, colBackColor);
      dtostrf(LastDisplayDHT1Temp, 5, 1, strPrint);
      tft.getTextBounds(strPrint, 0, 0, &x1, &y1, &w, &h);
      tft.setCursor(240 - w, 166 + baselineMiddle);
      tft.print(strPrint);
    }
  }
  // DHT 2
  if (kwlControl.getAdditionalSensors().hasDHT2()) {
    if (abs(LastDisplayDHT2Temp - kwlControl.getAdditionalSensors().getDHT2Temp()) > 0.5 || updateDisplayNow) {
      LastDisplayDHT2Temp = kwlControl.getAdditionalSensors().getDHT2Temp();
      tft.fillRect(160, 192, 80, numberfieldheight, colBackColor);
      dtostrf(LastDisplayDHT2Temp, 5, 1, strPrint);
      tft.getTextBounds(strPrint, 0, 0, &x1, &y1, &w, &h);
      tft.setCursor(240 - w, 192 + baselineMiddle);
      tft.print(strPrint);
    }
  }

  // CO2
  if (kwlControl.getAdditionalSensors().hasCO2()) {
    if (abs(LastDisplayMHZ14_CO2_ppm - kwlControl.getAdditionalSensors().getCO2()) > 10 || updateDisplayNow) {
      LastDisplayMHZ14_CO2_ppm = kwlControl.getAdditionalSensors().getCO2();
      tft.fillRect(160, 218, 80, numberfieldheight, colBackColor);
      sprintf(strPrint, "%5d", LastDisplayMHZ14_CO2_ppm);
      tft.getTextBounds(strPrint, 0, 0, &x1, &y1, &w, &h);
      tft.setCursor(240 - w, 218 + baselineMiddle);
      tft.print(strPrint);
    }
  }

  // VOC
  if (kwlControl.getAdditionalSensors().hasVOC()) {
    if (abs(LastDisplayTGS2600 - kwlControl.getAdditionalSensors().getVOC()) > 10 || updateDisplayNow) {
      LastDisplayTGS2600 = kwlControl.getAdditionalSensors().getVOC();
      tft.fillRect(160, 244, 80, numberfieldheight, colBackColor);
      sprintf(strPrint, "%5d", LastDisplayTGS2600);
      tft.getTextBounds(strPrint, 0, 0, &x1, &y1, &w, &h);
      tft.setCursor(240 - w, 244 + baselineMiddle);
      tft.print(strPrint);
    }
  }
}

void NewMenuScreen1() {
  //DHT, CO2, VOC Messwerte
  NewMenuEntry (1, F("->"));  // Führt zur Seite 0
}

void DoMenuActionScreen1() {
  // Weitere Sonsorwerte
  switch (menuBtnPressed) {
    case 1:
      gotoScreen (0);
      break;
    default:
      previousMillisDisplayUpdate = 0;
      break;
  }
}

// ************************************ ENDE: Screen 1  *****************************************************

// ****************************************** Screen 2: EINSTELLUNGEN ÜBERSICHT******************************
void SetupBackgroundScreen2() {
  // Übersicht Einstellungen
  char title[] = "Einstellungen";
  PrintScreenTitle(title);

  tft.setTextColor(colFontColor, colBackColor );

  tft.setCursor(18, 121 + baselineMiddle);
  tft.print (F("L1:  Normdrehzahl Zuluftventilator einstellen"));
  tft.setCursor(18, 166 + baselineMiddle);
  tft.print (F("L2:  Normdrehzahl Abluftventilator einstellen"));
  tft.setCursor(18, 211 + baselineMiddle);
  tft.print(F("KAL: Kalibrierung der Ventilatoransteuerung"));
  tft.setCursor(18, 256 + baselineMiddle);
  tft.print(F("RGL: Regelung der Ventilatoren"));
}

void loopDisplayUpdateScreen2() {

}

void NewMenuScreen2() {
  //Einstellungen Übersicht
  NewMenuEntry (1, F("->"));  // Führt zur Seite 6
  NewMenuEntry (2, F("<-"));  // Führt zur Seite 0
  NewMenuEntry (3, F("L1"));  // Führt zur Seite 3
  NewMenuEntry (4, F("L2"));  // Führt zur Seite 4
  NewMenuEntry (5, F("KAL")); // Führt zur Seite 5
  NewMenuEntry (6, F("RGL")); // Fährt zur Seite 6
}

void DoMenuActionScreen2() {
  // Einstellungen Übersicht
  switch (menuBtnPressed) {
    case 1:
      gotoScreen (6);
      break;
    case 2:
      gotoScreen (0);
      break;
    case 3:
      gotoScreen (3);
      break;
    case 4:
      gotoScreen (4);
      break;
    case 5:
      gotoScreen (5);
      break;
    case 6:
      gotoScreen (7);
      break;
    default:
      previousMillisDisplayUpdate = 0;
      break;
  }
}
// ************************************ ENDE: Screen 2  *****************************************************

// ****************************************** Screen 3: EINSTELLUNG NORMDREHZAHL L1 *************************
void SetupScreen3() {
  inputStandardSpeedSetpointFan1 = kwlControl.getFanControl().getFan1().getStandardSpeed();
}

void SetupBackgroundScreen3() {
  char title[] = "Normdrehzahl Abluftventilator";
  PrintScreenTitle(title);

  tft.setCursor(18, 75 + baselineMiddle);
  tft.print (F("Mit dem Button 'OK' wird der Wert gespeichert."));
  tft.setCursor(18, 100 + baselineMiddle);
  tft.print (F("Der aktueller Wert betraegt: "));
  tft.print (int(kwlControl.getFanControl().getFan1().getStandardSpeed()));
  tft.println(F(" U / min"));
  tft.setCursor(18, 150 + baselineMiddle);
  tft.print (F("Neuer Wert: "));

}

void loopDisplayUpdateScreen3() {
  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(colFontColor, colBackColor);
  tft.fillRect(18, 175, 80, numberfieldheight, colBackColor);
  tft.setCursor(18, 175 + baselineMiddle);
  tft.print (inputStandardSpeedSetpointFan1);
}

void NewMenuScreen3() {
  //Einstellungen L1
  NewMenuEntry (1, F("<-"));
  NewMenuEntry (3, F("+ 10"));
  NewMenuEntry (4, F("- 10"));
  NewMenuEntry (6, F("OK"));
}

void DoMenuActionScreen3() {
  // L1
  switch (menuBtnPressed) {
    case 1:
      gotoScreen (2);
      break;
    case 3:
      previousMillisDisplayUpdate = 0;
      inputStandardSpeedSetpointFan1 = inputStandardSpeedSetpointFan1 + 10;
      if (inputStandardSpeedSetpointFan1 > 10000) inputStandardSpeedSetpointFan1 = 10000;
      break;
    case 4:
      previousMillisDisplayUpdate = 0;
      if (inputStandardSpeedSetpointFan1 > 10)
        inputStandardSpeedSetpointFan1 = inputStandardSpeedSetpointFan1 - 10;
      else
        inputStandardSpeedSetpointFan1 = 0;
      break;
    case 6:
      previousMillisDisplayUpdate = 0;
      // Drehzahl Lüfter 1
      kwlControl.getFanControl().getFan1().setStandardSpeed(inputStandardSpeedSetpointFan1);
      kwlControl.getPersistentConfig().setSpeedSetpointFan1(inputStandardSpeedSetpointFan1);
      tft.setFont(&FreeSans9pt7b);
      tft.setTextColor(colFontColor, colBackColor);
      tft.setCursor(18, 225 + baselineMiddle);
      tft.print (F("Wert im EEPROM gespeichert"));
      tft.setCursor(18, 250 + baselineMiddle);
      tft.print (F("Bitte Kalibrierung starten!"));
      break;
    default:
      previousMillisDisplayUpdate = 0;
      break;
  }
}
// ************************************ ENDE: Screen 3  *****************************************************

// ****************************************** Screen 4: EINSTELLUNG NORMDREHZAHL L2 *************************
void SetupScreen4() {
  inputStandardSpeedSetpointFan2 = kwlControl.getFanControl().getFan2().getStandardSpeed();
}

void SetupBackgroundScreen4() {
  char title[] = "Normdrehzahl Abluftventilator";
  PrintScreenTitle(title);

  tft.setCursor(18, 75 + baselineMiddle);
  tft.print (F("Mit dem Button 'OK' wird der Wert gespeichert."));
  tft.setCursor(18, 100 + baselineMiddle);
  tft.print (F("Der aktueller Wert betraegt: "));
  tft.print (int(kwlControl.getFanControl().getFan2().getStandardSpeed()));
  tft.println(F(" U / min"));
  tft.setCursor(18, 150 + baselineMiddle);
  tft.print (F("Neuer Wert: "));

}

void loopDisplayUpdateScreen4() {
  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(colFontColor, colBackColor);
  tft.fillRect(18, 175, 80, numberfieldheight, colBackColor);
  tft.setCursor(18, 175 + baselineMiddle);
  tft.print (inputStandardSpeedSetpointFan2);
}

void NewMenuScreen4() {
  //Einstellungen L2
  NewMenuEntry (1, F("<-"));
  NewMenuEntry (3, F("+ 10"));
  NewMenuEntry (4, F("- 10"));
  NewMenuEntry (6, F("OK"));
}

void DoMenuActionScreen4() {
  // L2
  switch (menuBtnPressed) {
    case 1:
      gotoScreen (2);
      break;
    case 3:
      previousMillisDisplayUpdate = 0;
      inputStandardSpeedSetpointFan2 = inputStandardSpeedSetpointFan2 + 10;
      if (inputStandardSpeedSetpointFan2 > 10000) inputStandardSpeedSetpointFan2 = 10000;
      break;
    case 4:
      previousMillisDisplayUpdate = 0;
      if (inputStandardSpeedSetpointFan2 > 10)
        inputStandardSpeedSetpointFan2 = inputStandardSpeedSetpointFan2 - 10;
      else
        inputStandardSpeedSetpointFan2 = 0;
      break;
    case 6:
      previousMillisDisplayUpdate = 0;
      // Drehzahl Lüfter 1
      kwlControl.getFanControl().getFan2().setStandardSpeed(inputStandardSpeedSetpointFan2);
      kwlControl.getPersistentConfig().setSpeedSetpointFan2(inputStandardSpeedSetpointFan2);
      tft.setFont(&FreeSans9pt7b);
      tft.setTextColor(colFontColor, colBackColor);
      tft.setCursor(18, 225 + baselineMiddle);
      tft.print (F("Wert im EEPROM gespeichert"));
      tft.setCursor(18, 250 + baselineMiddle);
      tft.print (F("Bitte Kalibrierung starten!"));
      break;
    default:
      previousMillisDisplayUpdate = 0;
      break;
  }
}
// ************************************ ENDE: Screen 4  *****************************************************

// ****************************************** Screen 5: KALIBRIERUNG LÜFTER *********************************
void SetupScreen5() {
}

void SetupBackgroundScreen5() {
  char title[] = "Kalibrierung Ventilatoren";
  PrintScreenTitle(title);

  tft.setCursor(18, 75 + baselineMiddle);
  tft.print (F("Bei der Kalibrierung werden die Drehzahlen"));
  tft.setCursor(18, 100 + baselineMiddle);
  tft.print (F("der Luefter eingestellt und die notwendigen"));
  tft.setCursor(18, 125 + baselineMiddle);
  tft.print (F("PWM-Werte für jede Stufe gespeichert."));
}

void NewMenuScreen5() {
  //Einstellungen CAL
  NewMenuEntry (1, F("<-"));
  NewMenuEntry (6, F("OK"));
}

void loopDisplayUpdateScreen5() {

}

void DoMenuActionScreen5() {
  // CAL
  switch (menuBtnPressed) {
    case 1:
      gotoScreen (2);
      break;
    case 6:
      previousMillisDisplayUpdate = 0;
      kwlControl.getFanControl().speedCalibrationStart();
      tft.setFont(&FreeSans9pt7b);
      tft.setTextColor(colFontColor, colBackColor);
      tft.setCursor(18, 211 + baselineMiddle);
      tft.print (F("Kalibrierung Luefter ist gestartet"));
      break;
    default:
      previousMillisDisplayUpdate = 0;
      break;
  }
}
// ************************************ ENDE: Screen 5  *****************************************************

// ****************************************** Screen 6: WERKSEINSTELLUNGEN **********************************
void SetupScreen6() {
}

void SetupBackgroundScreen6() {
  char title[] = "Ruecksetzen auf Werkseinstellungen";
  PrintScreenTitle(title);

  tft.setTextColor(colFontColor, colBackColor );
  tft.setFont(&FreeSans9pt7b);

  tft.setCursor(18, 125 + baselineMiddle);
  tft.print (F("Es werden alle Werte der Steuerung auf die"));
  tft.setCursor(18, 150 + baselineMiddle);
  tft.print (F("Standardwerte zurueckgesetzt."));
  tft.setCursor(18, 175 + baselineMiddle);
  tft.print(F("Die Steuerung wird anschliessend neu gestartet."));
}

void NewMenuScreen6() {
  //Factory Reset
  NewMenuEntry (1, F("<-"));
  NewMenuEntry (6, F("OK"));
}

void loopDisplayUpdateScreen6() {
}

void DoMenuActionScreen6() {
  // RESET
  switch (menuBtnPressed) {
    case 1:
      gotoScreen (2);
      break;
    case 6:
      previousMillisDisplayUpdate = 0;
      Serial.println(F("Speicherbereich wird geloescht..."));
      tft.setFont(&FreeSans9pt7b);
      tft.setTextColor(colFontColor, colBackColor);
      tft.setCursor(18, 220 + baselineMiddle);
      tft.println(F("Speicherbereich wird geloescht..."));

      kwlControl.getPersistentConfig().factoryReset();

      tft.setCursor(18, 250 + baselineMiddle);
      tft.println(F("Loeschung erfolgreich, jetzt Reboot..."));

      // Reboot
      Serial.println(F("Reboot"));
      delay (5000);
      asm volatile ("jmp 0");
      break;
    default:
      previousMillisDisplayUpdate = 0;
      break;
  }
}
// ************************************ ENDE: Screen 6  *****************************************************

// ****************************************** Screen 7: REGELUNG VENTILATOREN **********************************
FanCalculateSpeedMode tmpInput = FanCalculateSpeedMode::UNSET;

void SetupScreen7() {
  tmpInput = FanCalculateSpeedMode::UNSET;
}

void SetupBackgroundScreen7() {
  // Übersicht Einstellungen
  char title[] = "Einstellungen Regelung Ventilatoren";
  PrintScreenTitle(title);

  tft.setTextColor(colFontColor, colBackColor );

  tft.setCursor(18, 75 + baselineMiddle);
  tft.print (F("Die Luefter koennen mit festen PWM Werten"));
  tft.setCursor(18, 100 + baselineMiddle);
  tft.print (F("oder dynamisch per PID-Regler geregelt"));
  tft.setCursor(18, 125 + baselineMiddle);
  tft.print(F("werden. Aktuelle Einstellung: "));
  if (kwlControl.getFanControl().getCalculateSpeedMode() == FanCalculateSpeedMode::PROP) {
    tft.print(F("PWM Wert"));
  } else {
    tft.print(F("PID-Regler"));
  }
  tft.setCursor(18, 175 + baselineMiddle);
  tft.print (F("Neuer Wert: "));
}

void NewMenuScreen7() {
  NewMenuEntry (1, F(" < -"));
  NewMenuEntry (3, F("PWM"));
  NewMenuEntry (4, F("PID"));
  NewMenuEntry (6, F("OK"));
}

void loopDisplayUpdateScreen7() {
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(colFontColor, colBackColor);
  tft.fillRect(18, 200, 130, numberfieldheight, colBackColor);
  tft.setCursor(18, 200 + baselineMiddle);
  if (tmpInput == FanCalculateSpeedMode::PROP) tft.print (F("PWM Wert"));
  if (tmpInput == FanCalculateSpeedMode::PID) tft.print (F("PID-Regler"));
}

void DoMenuActionScreen7() {
  // RESET
  switch (menuBtnPressed) {
    case 1:
      gotoScreen (2);
      break;
    case 3:
      previousMillisDisplayUpdate = 0;
      tmpInput = FanCalculateSpeedMode::PROP;
      break;
    case 4:
      previousMillisDisplayUpdate = 0;
      tmpInput = FanCalculateSpeedMode::PID;
      break;
    case 6:
      previousMillisDisplayUpdate = 0;
      if ((tmpInput == FanCalculateSpeedMode::PROP || tmpInput == FanCalculateSpeedMode::PID) && tmpInput != kwlControl.getFanControl().getCalculateSpeedMode()) {
        kwlControl.getFanControl().setCalculateSpeedMode(tmpInput);
        tft.setFont(&FreeSans9pt7b);
        tft.setTextColor(colFontColor, colBackColor);
        tft.fillRect(18, 250, 200, numberfieldheight, colBackColor);
        tft.setCursor(18, 250 + baselineMiddle);
        tft.print (F("Wert gespeichert."));
      } else {
        tft.setFont(&FreeSans9pt7b);
        tft.setTextColor(colFontColor, colBackColor);
        tft.fillRect(18, 250, 200, numberfieldheight, colBackColor);
        tft.setCursor(18, 250 + baselineMiddle);
        tft.print (F("Wert nicht geaendert"));
      }
    default:
      previousMillisDisplayUpdate = 0;
      break;
  }
}
// ************************************ ENDE: Screen 7  *****************************************************

void loopDisplayUpdate() {
  // Das Update wird alle 1000mS durchlaufen
  // Bevor Werte ausgegeben werden, wird auf Änderungen der Werte überprüft, nur geänderte Werte werden auf das Display geschrieben

  auto currentMillis = millis();

  if ((currentMillis - previousMillisDisplayUpdate >= intervalDisplayUpdate) || updateDisplayNow) {
    if (KWLConfig::serialDebugDisplay == 1) {
      Serial.println(F("loopDisplayUpdate"));
    }

    // Netzwerkverbindung anzeigen
    tft.setCursor(20, 0 + baselineMiddle);
    tft.setFont(&FreeSans9pt7b);  // Kleiner Font
    if (!kwlControl.getNetworkClient().isLANOk()) {
      if (LastLanOk != kwlControl.getNetworkClient().isLANOk() || updateDisplayNow) {
        LastLanOk = kwlControl.getNetworkClient().isLANOk();
        tft.fillRect(10, 0, 120, 20, colErrorBackColor);
        tft.setTextColor(colErrorFontColor);
        tft.print(F("ERR LAN"));
      }
    } else if (!kwlControl.getNetworkClient().isMQTTOk()) {
      if (LastMqttOk != kwlControl.getNetworkClient().isMQTTOk() || updateDisplayNow) {
        LastMqttOk = kwlControl.getNetworkClient().isMQTTOk();
        tft.fillRect(10, 0, 120, 20, colErrorBackColor);
        tft.setTextColor(colErrorFontColor );
        tft.print(F("ERR MQTT"));
      }
    } else {
      LastMqttOk = true;
      LastLanOk = true;
      tft.fillRect(10, 0, 120, 20, colBackColor);
      if (kwlControl.getNTP().hasTime()) {
        auto time = kwlControl.getNTP().currentTimeHMS(
              kwlControl.getPersistentConfig().getTimezoneMin() * 60,
              kwlControl.getPersistentConfig().getDST());
        char buffer[6];
        time.writeHM(buffer);
        buffer[5] = 0;
        tft.setTextColor(colFontColor);
        tft.print(buffer);
      }
    }

    // Einzelseiten
    if (KWLConfig::serialDebugDisplay == 1) {
      Serial.print (F("loopDisplayUpdate: menuScreen: "));
      Serial.println(menuScreen);
    }
    switch (menuScreen) {
      case 0:
        // Standardseite
        loopDisplayUpdateScreen0();
        break;
      case 1:
        loopDisplayUpdateScreen1();
        break;
      case 2:
        loopDisplayUpdateScreen2();
        break;
      case 3:
        loopDisplayUpdateScreen3();
        break;
      case 4:
        loopDisplayUpdateScreen4();
        break;
      case 5:
        loopDisplayUpdateScreen5();
        break;
      case 6:
        loopDisplayUpdateScreen6();
        break;
      case 7:
        loopDisplayUpdateScreen7();
        break;
    }

    unsigned errors = kwlControl.getErrors();
    unsigned infos = kwlControl.getInfos();
    if (errors != 0 ) {
      if (errors != LastErrorBits || updateDisplayNow) {
        // Neuer Fehler
        tft.fillRect(0, 300, 480, 21, colErrorBackColor );
        tft.setTextColor(colErrorFontColor );
        tft.setFont(&FreeSans9pt7b);  // Kleiner Font
        tft.setCursor(18, 301 + baselineSmall);
        char buffer[80];
        kwlControl.errorsToString(buffer, sizeof(buffer));
        tft.print(buffer);
        LastErrorBits = errors;
        tft.setFont(&FreeSans12pt7b);  // Mittlerer Font
      }
    } else if (infos != 0) {
      if (infos != LastInfoBits || updateDisplayNow) {
        // Neuer Fehler
        tft.fillRect(0, 300, 480, 21, colInfoBackColor );
        tft.setTextColor(colInfoFontColor );
        tft.setFont(&FreeSans9pt7b);  // Kleiner Font
        tft.setCursor(18, 301 + baselineSmall);
        char buffer[80];
        kwlControl.infosToString(buffer, sizeof(buffer));
        tft.print(buffer);
        LastInfoBits = infos;
        tft.setFont(&FreeSans12pt7b);  // Mittlerer Font
      }
    }
    else if (errors == 0 && infos == 0) {
      tft.fillRect(0, 300, 480, 20, colBackColor );
      LastErrorBits = 0;
      LastInfoBits = 0;
    }
    updateDisplayNow = false;
    previousMillisDisplayUpdate = currentMillis;
  }
}


void SetupBackgroundScreen() {

  if (KWLConfig::serialDebugDisplay == 1) {
    Serial.println(F("SetupBackgroundScreen"));
  }
  tft.fillRect(0, 30, 480 - touchBtnWidth, 290, colBackColor);

  switch (menuScreen) {
    case 0:
      // Standardseite
      SetupBackgroundScreen0();
      break;
    case 1:
      // Weitere Sensorwerte
      SetupBackgroundScreen1();
      break;
    case 2:
      // Übersicht Einstellungen
      SetupBackgroundScreen2();
      break;
    case 3:
      // L1
      SetupScreen3();
      SetupBackgroundScreen3();
      break;
    case 4:
      // L2
      SetupScreen4();
      SetupBackgroundScreen4();
      break;
    case 5:
      SetupScreen5();
      SetupBackgroundScreen5();
      break;
    case 6:
      SetupScreen6();
      SetupBackgroundScreen6();
      break;
    case 7:
      SetupScreen7();
      SetupBackgroundScreen7();
      break;
  }
  ShowMenu();
  updateDisplayNow = true;
}

void SetCursor(int x, int y) {
  tft.setCursor(x, y + baselineSmall);
}


// ********************************** Menüfunktionen ******************************************
// menuScreen = Angezeigte Bildschirmseite
// menuBtnActive[0..7] = true, wenn Menüeintrag sichtbar und aktiviert ist, ansonsten false

void gotoScreen(byte ScreenNo) {
  previousMillisDisplayUpdate = 0;
  menuScreen = ScreenNo;
  SetupBackgroundScreen();
}

void ShowMenu() {

  // Menu Hintergrund
  tft.fillRect(480 - touchBtnWidth , touchBtnYOffset, touchBtnWidth , 320 - touchBtnYOffset - 20, colMenuBackColor );

  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(colMenuFontColor, colMenuBackColor);

  for (int i = 0; i <= 6; i++) menuBtnActive[i] = false;

  if (menuScreen == 0) {
    //Standardseite
    NewMenuScreen0();
  } else if (menuScreen == 1) {
    //DHT, CO2, VOC Messwerte
    NewMenuScreen1();
  } else if (menuScreen == 2) {
    //Einstellungen Übersicht
    NewMenuScreen2();
  } else if (menuScreen == 3) {
    //Einstellungen L1
    NewMenuScreen3();
  } else if (menuScreen == 4) {
    //Einstellungen L2
    NewMenuScreen4();
  } else if (menuScreen == 5) {
    //Einstellungen CAL
    NewMenuScreen5();
  } else if (menuScreen == 6) {
    //Factory Reset
    NewMenuScreen6();
  } else if (menuScreen == 7) {
    //Einstellung Regelung
    NewMenuScreen7();
  }
}


void DoMenuAction() {
  if (KWLConfig::serialDebugDisplay) {
    Serial.print (F("DoMenuAction "));
    Serial.println(menuScreen);
  }

  if (menuScreen == 0) {
    DoMenuActionScreen0();
    menuBtnPressed = -1;
  }
  if (menuScreen == 1) {
    DoMenuActionScreen1();
    menuBtnPressed = -1;
  }
  if (menuScreen == 2) {
    DoMenuActionScreen2();
    menuBtnPressed = -1;
  }
  if (menuScreen == 3) {
    DoMenuActionScreen3();
    menuBtnPressed = -1;
  }
  if (menuScreen == 4) {
    DoMenuActionScreen4();
    menuBtnPressed = -1;
  }
  if (menuScreen == 5) {
    DoMenuActionScreen5();
    menuBtnPressed = -1;
  }
  if (menuScreen == 6) {
    DoMenuActionScreen6();
    menuBtnPressed = -1;
  }
  if (menuScreen == 7) {
    DoMenuActionScreen7();
    menuBtnPressed = -1;
  }
}

void NewMenuEntry(byte mnuBtn, String mnuTxt) {
  menuBtnActive[mnuBtn] = true;
  PrintMenuBtn (mnuBtn, mnuTxt, colMenuBtnFrame);
}

void PrintMenuBtn (byte mnuBtn, String mnuTxt, long colFrame) {
  // ohne mnuText wird vom Button nur der Rand gezeichnet

  if (mnuBtn > 0 && mnuBtn <= 6 && menuBtnActive[mnuBtn]) {
    int x, y;
    x = 480 - touchBtnWidth + 1;
    y = touchBtnYOffset + 1 + touchBtnHeight * (mnuBtn - 1);

    if (colFrame == colMenuBtnFrameHL) {
      // Highlight Frame
      tft.drawRoundRect(x, y, touchBtnWidth - 2, touchBtnHeight - 2, 5, colFrame);
      tft.drawRoundRect(x + 1, y + 1, touchBtnWidth - 4, touchBtnHeight - 4, 5, colFrame);
      tft.drawRoundRect(x + 2, y + 2, touchBtnWidth - 6, touchBtnHeight - 6, 5, colFrame);
    } else {
      tft.drawRoundRect(x, y, touchBtnWidth - 2, touchBtnHeight - 2, 5, colFrame);
      tft.drawRoundRect(x + 1, y + 1, touchBtnWidth - 4, touchBtnHeight - 4, 5, colMenuBackColor);
      tft.drawRoundRect(x + 2, y + 2, touchBtnWidth - 6, touchBtnHeight - 6, 5, colMenuBackColor);
    }

    mnuTxt.toCharArray(strPrint, 10) ;
    tft.getTextBounds(strPrint, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(480 - touchBtnWidth / 2 - w / 2 , y + 15 + baselineSmall);
    tft.print(strPrint);
  }
}

void SetMenuBorder(byte menuBtn) {
  if (millis() - millisLastMenuBtnPress > intervalMenuBtn - 100 || menuBtn > 0) {
    if (menuBtn > 0) {
      PrintMenuBtn (menuBtn, "", colMenuBtnFrameHL);
      LastMenuBtn = menuBtn;
    } else {
      PrintMenuBtn (LastMenuBtn, "", colMenuBtnFrame);
    }
  }
}

void PrintScreenTitle(char* title) {
  //title.toCharArray(strPrint, title.length()) ;
  tft.setFont(&FreeSans9pt7b);  // Kleiner Font
  tft.setTextSize(fontFactorSmall);
  tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((480 - touchBtnWidth) / 2 - w / 2, 30 + baselineSmall);
  tft.setTextColor(colWindowTitleFontColor, colWindowTitleBackColor );
  tft.fillRect(0, 30, 480 - touchBtnWidth, 20, colWindowTitleBackColor );
  tft.print(title);
  tft.setTextColor(colFontColor, colBackColor );
}

void loopIsDisplayTouched() {
  if (millis() - LastMillisTouch > intervalLastMillisTouch) {
    LastMillisTouch = millis();
    if (menuScreen != 0) gotoScreen (0);
  }
}

void loopTouch()
{
  loopIsDisplayTouched();

  SetMenuBorder(0);

  if (millis() - millisLastMenuBtnPress > intervalMenuBtn) {

    int16_t xpos, ypos;  //screen coordinates
    tp = ts.getPoint();   //tp.x, tp.y are ADC values

    // if sharing pins, you'll need to fix the directions of the touchscreen pins
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    pinMode(XP, OUTPUT);
    pinMode(YM, OUTPUT);
    //    digitalWrite(XM, HIGH);
    //    digitalWrite(YP, HIGH);
    // we have some minimum pressure we consider 'valid'
    // pressure of 0 means no pressing!

    if (tp.z > MINPRESSURE && tp.z < MAXPRESSURE) {
      // is controller wired for Landscape ? or are we oriented in Landscape?
      if (KWLConfig::TouchSwapXY != (Orientation & 1)) SWAP(tp.x, tp.y);
      // scale from 0->1023 to tft.width  i.e. left = 0, rt = width
      // most mcufriend have touch (with icons) that extends below the TFT
      // screens without icons need to reserve a space for "erase"
      // scale the ADC values from ts.getPoint() to screen values e.g. 0-239
      xpos = map(tp.x, TS_LEFT, TS_RT, 0, tft.width());
      ypos = map(tp.y, TS_TOP, TS_BOT, 0, tft.height());

      LastMillisTouch = millis();

      if (KWLConfig::serialDebugDisplay) {
        Serial.print(F("Touch (xpos/ypos, tp.x/tp.y): "));
        Serial.print(xpos);
        Serial.print('/');
        Serial.print(ypos);
        Serial.print(',');
        Serial.print(tp.x);
        Serial.print('/');
        Serial.println(tp.y);
      }

      // are we in top color box area ?
      if (xpos > 480 - touchBtnWidth) {               // Touch im Menübereich, rechte Seite

        if         (ypos < touchBtnYOffset + touchBtnHeight * 0) {
          // Headline, Nothing to do
        } else if (ypos <  touchBtnYOffset + touchBtnHeight * 1) {
          if (menuBtnActive[1]) {
            menuBtnPressed = 1;
            SetMenuBorder(menuBtnPressed);
            millisLastMenuBtnPress = millis();
          }
        } else if (ypos <  touchBtnYOffset + touchBtnHeight * 2) {
          if (menuBtnActive[2]) {
            menuBtnPressed = 2;
            SetMenuBorder(menuBtnPressed);
            millisLastMenuBtnPress = millis();
          }
        } else if (ypos <  touchBtnYOffset + touchBtnHeight * 3) {
          if (menuBtnActive[3]) {
            menuBtnPressed = 3;
            SetMenuBorder(menuBtnPressed);
            millisLastMenuBtnPress = millis();
          }
        } else if (ypos <  touchBtnYOffset + touchBtnHeight * 4) {
          if (menuBtnActive[4]) {
            menuBtnPressed = 4;
            SetMenuBorder(menuBtnPressed);
            millisLastMenuBtnPress = millis();
          }
        } else if (ypos <  touchBtnYOffset + touchBtnHeight * 5) {
          if (menuBtnActive[5]) {
            menuBtnPressed = 5;
            SetMenuBorder(menuBtnPressed);
            millisLastMenuBtnPress = millis();
          }
        } else if (ypos <  touchBtnYOffset + touchBtnHeight * 6) {
          if (menuBtnActive[6]) {
            menuBtnPressed = 6;
            SetMenuBorder(menuBtnPressed);
            millisLastMenuBtnPress = millis();
          }
        }
        DoMenuAction();
      }
    }
  }
}

// **************************** ENDE  Menüfunktionen ******************************************


// *** oberer Rand
void print_header() {
  tft.fillRect(0, 0, 480, 20, colBackColor);
  tft.setCursor(140, 0 + baselineSmall);
  tft.setTextColor(colFontColor);
  tft.setTextSize(fontFactorSmall);
  tft.print(F(" * Pluggit AP 300 * "));
  tft.setCursor(420, 0 + baselineSmall);
  tft.print(KWLConfig::VersionString);
}

// *** TFT starten
void SetupTftScreen() {
  if (KWLConfig::serialDebugDisplay == 1) {
    Serial.println(F("start_tft"));
  }
  ID = tft.readID();  // you must detect the correct controller
  tft.begin(ID);      // everything will start working

  int16_t  x1, y1;
  uint16_t w, h;
  tft.setFont(&FreeSansBold24pt7b);  // Großer Font
  // Baseline bestimmen für kleinen Font
  char CharM[] = "M";
  tft.getTextBounds(CharM, 0, 0, &x1, &y1, &w, &h);
  baselineBigNumber = h;
  Serial.print (F("Font baseline (big / middle / small): "));
  Serial.print (h);
  Serial.print (F(" / "));
  tft.setFont(&FreeSans12pt7b);  // Mittlerer Font
  // Baseline bestimmen für kleinen Font
  char Char9[] = "9";
  tft.getTextBounds(Char9, 0, 0, &x1, &y1, &w, &h);
  baselineMiddle = h;
  numberfieldheight = h + 1 ;
  Serial.print (numberfieldheight);
  Serial.print (F(" / "));
  tft.setFont(&FreeSans9pt7b);  // Kleiner Font
  // Baseline bestimmen für kleinen Font
  tft.getTextBounds(CharM, 0, 0, &x1, &y1, &w, &h);
  baselineSmall = h;
  Serial.print (h);
  Serial.println ();

  Serial.print(F("TFT controller: "));
  Serial.println(ID);
  tft.setRotation(1);
  tft.fillScreen(colBackColor);

  print_header();
  SetCursor(0, 30);
}

void SetupTouch(void)
{
  uint16_t tmp;
  identifier = tft.readID();
  switch (Orientation) {      // adjust for different aspects
    case 0:   break;        //no change,  calibrated for PORTRAIT
    case 1:   tmp = TS_LEFT, TS_LEFT = TS_BOT, TS_BOT = TS_RT, TS_RT = TS_TOP, TS_TOP = tmp;  break;
    case 2:   SWAP(TS_LEFT, TS_RT);  SWAP(TS_TOP, TS_BOT); break;
    case 3:   tmp = TS_LEFT, TS_LEFT = TS_TOP, TS_TOP = TS_RT, TS_RT = TS_BOT, TS_BOT = tmp;  break;
  }
  ts = TouchScreen(XP, YP, XM, YM, 300);     //call the constructor AGAIN with new values.
  show_Touch_Serial();
}

void show_Touch_Serial(void)
{
  Serial.print(F("Found "));
  Serial.print(name);
  Serial.println(F(" LCD driver"));
  Serial.print(F("ID = 0x"));
  Serial.println(identifier, HEX);
  Serial.print(F("Screen is "));
  Serial.print(tft.width());
  Serial.print('x');
  Serial.println(tft.height());
  Serial.print(F("Calibration is: LEFT = "));
  Serial.print(TS_LEFT);
  Serial.print(F(" RT = "));
  Serial.print(TS_RT);
  Serial.print(F(" TOP = "));
  Serial.print(TS_TOP);
  Serial.print(F(" BOT = "));
  Serial.println(TS_BOT);
  Serial.print(F("Wiring is: "));
  Serial.println(KWLConfig::TouchSwapXY ? F("SwapXY") : F("PORTRAIT"));
  Serial.print(F("YP = "));
  Serial.print(YP);
  Serial.print(F(" XM = "));
  Serial.println(XM);
  Serial.print(F("YM = "));
  Serial.print(YM);
  Serial.print(F(" XP = "));
  Serial.println(XP);
}