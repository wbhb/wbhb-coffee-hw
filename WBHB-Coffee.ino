#include <Adafruit_RGBLCDShield.h>
#include <MAX6675.h>
#include <PID_v1.h>
#include <EEPROM.h>

#define OFF 0x0
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7 



const uint8_t PARAMS_VERSION = 1;
#define PARAMS_VER_ADD 0x00
#define PARAMS_BASE_ADD 0x04

#define KP 1
#define KI 2
#define KD 3
#define SET_BREW 4
#define SET_STEAM 5
#define OFFSET 6
#define BANG_LOW 7
#define BANG_HIGH 8
#define PID_LOW_ADJUST 9

// Params

double params[10] = {
  0, // NOT USED
  15, // params[KP] = 15;
  0.5, // params[KI] = 0.5;
  10, // params[KD] = 10;
  95, // params[SET_BREW] = 95;
  120, // steamSetPoint = 120;
  6, // offset = 6;

  // Advanced Params
  75, // params[BANG_LOW] = 75;
  105, // params[BANG_HIGH] = 105;
  -50 // params[PID_LOW_ADJUST] = -50;
};

enum STATE {
  INIT,   // 
  SETUP,  // change parameters
  HEAT,   // run boiler at brew temperature
  STEAM   // run boiler at steam temperature
};

// Vars
STATE state = INIT;

double boilerTemp = 0;
double groupTemp = boilerTemp - params[OFFSET];
double boilerSetPoint = params[SET_BREW] + params[OFFSET];

double pidOut = 0;
int pidWindowSize = 400;
unsigned long pidWindowStartTime;

unsigned long serialTime = 100;
unsigned long lastSerialUpdate;

unsigned long frameTime = 500;
unsigned long lastScreenUpdate;

unsigned long stableTarget = 20000;
unsigned long stableTime = 0;

unsigned short editSetting = SET_BREW;

bool buttonLatch = false;

// hardware interfaces

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

#define CS_PIN 6
MAX6675 tcouple(CS_PIN);

float tempSmooth[4] = {0, 0, 0, 0};
unsigned char tempSmoothInd = 0;

#define SSR_PIN 4
PID tempControl(&boilerTemp, &pidOut, &boilerSetPoint, params[KP], params[KI], params[KD], DIRECT);

// Convenience functions

void updateSetPoint(double atGroup) {
  boilerSetPoint = atGroup + params[OFFSET];
}

void updateTemp() {
  tempSmooth[tempSmoothInd] = tcouple.readTempC();
  tempSmoothInd++;
  if (tempSmoothInd > 3) {
    tempSmoothInd = 0;
  }

  boilerTemp = (tempSmooth[0] + tempSmooth[1] + tempSmooth[2] + tempSmooth[3]) / 4;
  groupTemp = boilerTemp - params[OFFSET];

  if (tempControl.GetMode() == AUTOMATIC) {
    if (groupTemp > params[BANG_HIGH]) {
      tempControl.SetMode(MANUAL);
      pidOut = params[PID_LOW_ADJUST];
    }
    if (groupTemp < params[BANG_LOW]) {
      tempControl.SetMode(MANUAL);
      pidOut = pidWindowSize;
    }
  }

  if (tempControl.GetMode() == MANUAL) {
    if (groupTemp < params[BANG_HIGH] && groupTemp > params[BANG_LOW]) {
      pidOut = 0;
      tempControl.SetMode(AUTOMATIC);
    }
  }
}

bool pidWindow() {
  unsigned long now = millis();

  if (now - pidWindowStartTime > pidWindowSize) {
    pidWindowStartTime += pidWindowSize;
  }

  if (pidOut > now - pidWindowStartTime) {
    return true;
  } else {
    return false;
  }
}

void setup() {
    state = INIT;

    pinMode(SSR_PIN, OUTPUT);

    Serial.begin(9600, SERIAL_8N1);

    lcd.begin(16, 2);
    lcd.print("WBHB Coffee");
    lcd.setCursor(0, 1);
    lcd.print("Init v");

    uint8_t version = 0;

    EEPROM.get(PARAMS_VER_ADD, version);

    if (version == PARAMS_VERSION) {
      lcd.print(PARAMS_VERSION);
      // load stored params from EEPROM
      EEPROM.get(PARAMS_BASE_ADD, params);
    } else {
      // print update
      lcd.print(version);
      lcd.print("->");
      lcd.print(PARAMS_VERSION);
      // store version to EEPROM
      EEPROM.put(PARAMS_VER_ADD, PARAMS_VERSION);
      // store default params to EEPROM
      EEPROM.put(PARAMS_BASE_ADD, params);
    }

    delay(500);

    state = HEAT;

    updateSetPoint(params[SET_BREW]);

    pidWindowStartTime = millis();
    tempControl.SetOutputLimits(params[PID_LOW_ADJUST], pidWindowSize);
    tempControl.SetMode(AUTOMATIC);

    lastSerialUpdate = millis();
    lastScreenUpdate = millis();

}

void loop() {

  unsigned long now = millis();

  updateTemp();

  tempControl.Compute();

  if (pidWindow()) {
    digitalWrite(SSR_PIN, HIGH);
  } else {
    digitalWrite(SSR_PIN, LOW);
  }

  uint8_t buttons = lcd.readButtons();

  if (buttons) {
    if (!buttonLatch) {
      buttonLatch = true;
      switch (state) {
        case HEAT:
          if (buttons & BUTTON_SELECT) {
            // state = BREW;
          }
          if (buttons & BUTTON_UP) {
            params[SET_BREW]++;
            updateSetPoint(params[SET_BREW]);
            EEPROM.put(PARAMS_BASE_ADD + SET_BREW * sizeof(double), params[SET_BREW]);
            break;
          }
          if (buttons & BUTTON_DOWN) {
            params[SET_BREW]--;
            updateSetPoint(params[SET_BREW]);
            EEPROM.put(PARAMS_BASE_ADD + SET_BREW * sizeof(double), params[SET_BREW]);
            break;
          }
          if (buttons & BUTTON_LEFT) {
            state = SETUP;
          }
          if (buttons & BUTTON_RIGHT) {
            state = STEAM;
            updateSetPoint(params[SET_STEAM]);
          }
          break;
        case STEAM:
          if (buttons & BUTTON_UP) {
            params[SET_STEAM]++;
            updateSetPoint(params[SET_STEAM]);
            EEPROM.put(PARAMS_BASE_ADD + SET_STEAM * sizeof(double), params[SET_STEAM]);
            break;
          }
          if (buttons & BUTTON_DOWN) {
            params[SET_STEAM]--;
            updateSetPoint(params[SET_STEAM]);
            EEPROM.put(PARAMS_BASE_ADD + SET_STEAM * sizeof(double), params[SET_STEAM]);
            break;
          }
          if (buttons & BUTTON_LEFT) {
            state = HEAT;
            updateSetPoint(params[SET_BREW]);
          }
          break;
        case SETUP:
          if (buttons & BUTTON_SELECT) {
            switch (editSetting) {
              case KP:
                editSetting = KI;
                break;
              case KI:
                editSetting = KD;
                break;
              case KD:
                editSetting = KP;
                break;
            }
          }
          if (buttons & BUTTON_UP) {
            switch (editSetting) {
              case KP:
                params[KP] += 1;
                tempControl.SetTunings(params[KP], params[KI], params[KD]);
                EEPROM.put(PARAMS_BASE_ADD + KP * sizeof(double), params[KP]);
                break;
              case KI:
                params[KI] += 0.1;
                tempControl.SetTunings(params[KP], params[KI], params[KD]);
                EEPROM.put(PARAMS_BASE_ADD + KI * sizeof(double), params[KI]);
                break;
              case KD:
                params[KD] += 0.1;
                tempControl.SetTunings(params[KP], params[KI], params[KD]);
                EEPROM.put(PARAMS_BASE_ADD + KD * sizeof(double), params[KD]);
                break;
            }
          }
          if (buttons & BUTTON_DOWN) {
            switch (editSetting) {
              case KP:
                params[KP] -= 1;
                tempControl.SetTunings(params[KP], params[KI], params[KD]);
                EEPROM.put(PARAMS_BASE_ADD + KP * sizeof(double), params[KP]);
                break;
              case KI:
                params[KI] -= 0.1;
                tempControl.SetTunings(params[KP], params[KI], params[KD]);
                EEPROM.put(PARAMS_BASE_ADD + KI * sizeof(double), params[KI]);
                break;
              case KD:
                params[KD] -= 0.1;
                tempControl.SetTunings(params[KP], params[KI], params[KD]);
                EEPROM.put(PARAMS_BASE_ADD + KD * sizeof(double), params[KD]);
                break;
            }
          }
          break;
          if (buttons & BUTTON_RIGHT) {
            state = HEAT;
          }
      }
    }
  } else {
    if (buttonLatch) {
      buttonLatch = false;
    }
  }

  if (now - lastSerialUpdate > serialTime) {
    if (Serial) {
      lastSerialUpdate = now;

      Serial.print("temp:");
      Serial.print(groupTemp);
      Serial.print(";pid:");
      Serial.print(pidOut);
      Serial.print("\n");
    }
  }

  if (now - lastScreenUpdate > frameTime) {
    lastScreenUpdate = now;
    lcd.clear();

    lcd.setCursor(0, 0);

    lcd.print("Set: ");
    if (state == HEAT) {
      lcd.print(params[SET_BREW]);
      lcd.setCursor(10, 0);
      lcd.print("BREW");
    }
    if (state == STEAM) {
      lcd.print(params[SET_STEAM]);
      lcd.setCursor(10, 0);
      lcd.print("STEAM");
    }
    // lcd.print(pidOut);
    
    lcd.setCursor(0, 1);
    
    lcd.print("Act: ");
    lcd.print(groupTemp);

    if (state == SETUP) {
    lcd.setCursor(12, 0);
      switch (editSetting) {
        case SET_BREW:
          lcd.print(" *S");
          break;
        case KP:
          lcd.print(" *P");
          break;
        case KI:
          lcd.print(" *I");
          break;
        case KD:
          lcd.print(" *D");
          break;
      }

      
      lcd.setCursor(12, 1);
      switch (editSetting) {
        case SET_BREW:
          lcd.print(params[SET_BREW], 0);
          break;
        case KP:
          lcd.print(params[KP], 1);
          break;
        case KI:
          lcd.print(params[KI], 1);
          break;
        case KD:
          lcd.print(params[KD], 1);
          break;
      }
    }

    if (groupTemp <= params[SET_BREW] + 1
     && groupTemp >= params[SET_BREW] - 1) {
      if (stableTime == 0) {
        stableTime = now;
      }
      if (now - stableTime > stableTarget) {
        lcd.setBacklight(GREEN);
      } else {
        lcd.setBacklight(VIOLET);
      }
      
    } else if (groupTemp < params[SET_BREW]) {
      stableTime = 0;
      lcd.setBacklight(BLUE);
    } else {
      stableTime = 0;
      lcd.setBacklight(RED);
    }
  }
}
