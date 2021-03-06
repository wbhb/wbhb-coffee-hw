#define PRINTF_SUPPORT_EXPONENTIAL_SPECIFIERS 0
#define PRINTF_SUPPORT_LONG_LONG 0

#include <Adafruit_RGBLCDShield.h>
#include <MAX6675.h>
#include <PID_v1.h>
#include <EEPROM.h>
#include "lib/printf/printf.cpp"

// Constants

#define OFF 0x0
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7 

enum STATE {
  INIT,   // 
  SETUP,  // change parameters
  HEAT,   // run boiler at brew temperature
  BREW,   // pull a shot
  STEAM   // run boiler at steam temperature
};

enum SET {
  AT_BOILER,
  AT_GROUP
};

enum TEMP_STATE {
  TEMP_LOW,
  TEMP_HIGH,
  UNSTABLE,
  STABLE
};

/*const PROGMEM */byte DEGREES_C[8] = {
  B01000,
  B10100,
  B01000,
  B00011,
  B00100,
  B00100,
  B00011,
  B00000
};

// Params

const PROGMEM uint8_t PARAMS_VERSION = 3;
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
#define MAX_TEMP 9
#define PID_LOW_ADJUST 10

double params[11] = {
  0, // NOT USED (version)
  15, // params[KP] = 15;
  0.5, // params[KI] = 0.5;
  10, // params[KD] = 10;
  95, // params[SET_BREW] = 95;
  130, // steamSetPoint = 120;
  6, // offset = 6;

  // Advanced Params
  20, // params[BANG_LOW] force output when more than this far below setpoint;
  10, // params[BANG_HIGH] force output off when more that this far above setpoint;
  140, // params[MAX_TEMP] maximum boiler temperature
  -50 // params[PID_LOW_ADJUST] = -50;
};

// Vars
STATE state = STATE::INIT;

double boilerTemp = 0;
double groupTemp = boilerTemp - params[OFFSET];
double boilerSetPoint = params[SET_BREW] + params[OFFSET];

TEMP_STATE tempState = TEMP_STATE::TEMP_LOW;

double pidOut = 0;
int pidWindowSize = 400;
unsigned long pidWindowStartTime;

unsigned long serialTime = 100;
unsigned long lastSerialUpdate;

unsigned long frameTime = 500;
unsigned long lastScreenUpdate;

unsigned long stableTarget = 20000;
unsigned long stableTime = 0;

unsigned short editSetting = KP;

bool buttonLatch = false;

// hardware interfaces

Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

char lcdBuffer_top[]    = "                ";
char lcdBuffer_bottom[] = "                ";

#define CS_PIN 6
MAX6675 tcouple(CS_PIN);

float tempSmooth[4] = {0, 0, 0, 0};
unsigned char tempSmoothInd = 0;

#define SSR_PIN 4
PID tempControl(&boilerTemp, &pidOut, &boilerSetPoint, params[KP], params[KI], params[KD], DIRECT);

// Convenience functions

double updateParam(uint8_t address, double delta) {
  params[address] += delta;
  EEPROM.put(PARAMS_BASE_ADD + address * sizeof(double), params[address]);
}

void updateSetPoint(double setPoint, SET setAt) {
  if (setAt == SET::AT_GROUP) {
    setPoint += params[OFFSET];
  }
  boilerSetPoint = setPoint;
}

void updateTemp() {
  tempSmooth[tempSmoothInd] = tcouple.readTempC();
  tempSmoothInd++;
  if (tempSmoothInd > 3) {
    tempSmoothInd = 0;
  }

  boilerTemp = (tempSmooth[0] + tempSmooth[1] + tempSmooth[2] + tempSmooth[3]) / 4;
  groupTemp = boilerTemp - params[OFFSET];

  // force boiler off if too hot
  if (boilerTemp > params[MAX_TEMP]) {
    tempControl.SetMode(MANUAL);
    pidOut = params[PID_LOW_ADJUST];
  } else {

    // handle switching from pid to bang
    if (tempControl.GetMode() == AUTOMATIC) {
      if (boilerTemp > boilerSetPoint + params[BANG_HIGH]) {
        tempControl.SetMode(MANUAL);
        pidOut = params[PID_LOW_ADJUST];
      }
      if (boilerTemp < boilerSetPoint - params[BANG_LOW]) {
        tempControl.SetMode(MANUAL);
        pidOut = pidWindowSize;
      }
    }

    // hadle switched from bang to pid
    if (tempControl.GetMode() == MANUAL) {
      if (boilerTemp < boilerSetPoint + params[BANG_HIGH]
       && boilerTemp > boilerSetPoint - params[BANG_LOW]) {
        pidOut = 0;
        tempControl.SetMode(AUTOMATIC);
      }
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
  state = STATE::INIT;

  pinMode(SSR_PIN, OUTPUT);

  Serial.begin(9600, SERIAL_8N1);

  lcd.begin(16, 2);

  lcd.createChar(7, DEGREES_C);

  sprintf_(lcdBuffer_bottom, "   WBHB Coffee  ");

  uint8_t version = 0;

  EEPROM.get(PARAMS_VER_ADD, version);

  if (version == PARAMS_VERSION) {
    sprintf_(lcdBuffer_top, "Init: v%2d       ", PARAMS_VERSION);
    // load stored params from EEPROM
    EEPROM.get(PARAMS_BASE_ADD, params);
  } else {
    // print update
    sprintf_(lcdBuffer_top, "Init: v%2d->%2d   ", version, PARAMS_VERSION);
    // store version to EEPROM
    EEPROM.put(PARAMS_VER_ADD, PARAMS_VERSION);
    // store default params to EEPROM
    EEPROM.put(PARAMS_BASE_ADD, params);
  }

  lcd.setCursor(0, 0);
  lcd.print(lcdBuffer_top);
  lcd.setCursor(0, 1);
  lcd.print(lcdBuffer_bottom);

  delay(500);
  
  state = STATE::HEAT;

  updateSetPoint(params[SET_BREW], SET::AT_GROUP);

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
        case STATE::HEAT:
          if (buttons & BUTTON_SELECT) {
            // state = STATE::BREW;
          }
          if (buttons & BUTTON_UP) {
            updateParam(SET_BREW, 1);
            updateSetPoint(params[SET_BREW], SET::AT_GROUP);
            break;
          }
          if (buttons & BUTTON_DOWN) {
            updateParam(SET_BREW, -1);
            updateSetPoint(params[SET_BREW], SET::AT_GROUP);
            break;
          }
          if (buttons & BUTTON_LEFT) {
            state = STATE::SETUP;
          }
          if (buttons & BUTTON_RIGHT) {
            state = STATE::STEAM;
            updateSetPoint(params[SET_STEAM], SET::AT_BOILER);
          }
          break;
        case STATE::STEAM:
          if (buttons & BUTTON_UP) {
            updateParam(SET_STEAM, 1);
            updateSetPoint(params[SET_STEAM], SET::AT_BOILER);
            break;
          }
          if (buttons & BUTTON_DOWN) {
            updateParam(SET_STEAM, -1);
            updateSetPoint(params[SET_STEAM], SET::AT_BOILER);
            break;
          }
          if (buttons & BUTTON_LEFT) {
            state = STATE::HEAT;
            updateSetPoint(params[SET_BREW], SET::AT_GROUP);
          }
          break;
        case STATE::SETUP:
          if (buttons & BUTTON_SELECT) {
            switch (editSetting) {
              case KP:
                editSetting = KI;
                break;
              case KI:
                editSetting = KD;
                break;
              case KD:
                editSetting = OFFSET;
                break;
              case OFFSET:
                editSetting = BANG_HIGH;
                break;
              case BANG_HIGH:
                editSetting = BANG_LOW;
                break;
              case BANG_LOW:
                editSetting = MAX_TEMP;
                break;
              case MAX_TEMP:
                editSetting = KP;
                break;
            }
          }
          if (buttons & BUTTON_UP) {
            switch (editSetting) {
              case KP:
                updateParam(KP, 1);
                tempControl.SetTunings(params[KP], params[KI], params[KD]);
                break;
              case KI:
                updateParam(KI, 0.1);
                tempControl.SetTunings(params[KP], params[KI], params[KD]);
                break;
              case KD:
                updateParam(KD, 0.1);
                tempControl.SetTunings(params[KP], params[KI], params[KD]);
                break;
              case OFFSET:
                updateParam(OFFSET, 0.1);
                break;
              case BANG_HIGH:
                updateParam(BANG_HIGH, 1);
                break;
              case BANG_LOW:
                updateParam(BANG_LOW, -1);
                break;
              case MAX_TEMP:
                updateParam(MAX_TEMP, 1);
                break;
            }
          }
          if (buttons & BUTTON_DOWN) {
            switch (editSetting) {
              case KP:
                updateParam(KP, -1);
                tempControl.SetTunings(params[KP], params[KI], params[KD]);
                break;
              case KI:
                updateParam(KI, -0.1);
                tempControl.SetTunings(params[KP], params[KI], params[KD]);
                break;
              case KD:
                updateParam(KD, -0.1);
                tempControl.SetTunings(params[KP], params[KI], params[KD]);
                break;
              case OFFSET:
                updateParam(OFFSET, -0.1);
                break;
              case BANG_HIGH:
                updateParam(BANG_HIGH, -1);
                break;
              case BANG_LOW:
                updateParam(BANG_LOW, 1);
                break;
              case MAX_TEMP:
                updateParam(MAX_TEMP, -1);
                break;
            }
          }
          if (buttons & BUTTON_RIGHT) {
            state = STATE::HEAT;
          }
      }
    }
  } else {
    if (buttonLatch) {
      buttonLatch = false;
    }
  }

  if (boilerTemp <= boilerSetPoint + 1
     && boilerTemp >= boilerSetPoint - 1) {
    if (stableTime == 0) {
      stableTime = now;
    }
    if (now - stableTime > stableTarget) {
      tempState = TEMP_STATE::STABLE;
    } else {
      tempState = TEMP_STATE::UNSTABLE;
    }
  } else if (boilerTemp < boilerSetPoint) {
    stableTime = 0;
    tempState = TEMP_STATE::TEMP_LOW;
  } else {
    stableTime = 0;
    tempState = TEMP_STATE::TEMP_HIGH;
  }

  if (now - lastSerialUpdate > serialTime) {
    if (Serial) {
      lastSerialUpdate = now;

      Serial.print("bTemp:");
      Serial.print(boilerTemp);
      Serial.print(";gTemp:");
      Serial.print(groupTemp);
      Serial.print(";setP:");
      Serial.print(boilerSetPoint);
      Serial.print(";pid:");
      Serial.print(pidOut);
      Serial.print("\n");
    }
  }

  if (now - lastScreenUpdate > frameTime) {
    lastScreenUpdate = now;
    
    if (state == STATE::HEAT) {
      sprintf_(lcdBuffer_top, "Heat:       %3.0f%c", params[SET_BREW], 7);

      if (tempState == TEMP_STATE::STABLE) {
        sprintf_(lcdBuffer_bottom, "     <Brew>     ");
      } else {
        sprintf_(lcdBuffer_bottom, "         %6.2f%c", groupTemp, 7);
      }
    }

    if (state == STATE::STEAM) {
      sprintf_(lcdBuffer_top, "Steam:      %3.0f%c", params[SET_STEAM], 7);
      sprintf_(lcdBuffer_bottom, "         %6.2f%c", boilerTemp, 7);
    }

    if (state == STATE::SETUP) {
      switch (editSetting) {
        case KP:
          sprintf_(lcdBuffer_top,    "Setup:        kP");
          sprintf_(lcdBuffer_bottom, "             %3.0f", params[KP]);
          break;
        case KI:
          sprintf_(lcdBuffer_top,    "Setup:        kI");
          sprintf_(lcdBuffer_bottom, "           %5.1f", params[KI]);
          break;
        case KD:
          sprintf_(lcdBuffer_top,    "Setup:        kD");
          sprintf_(lcdBuffer_bottom, "           %5.1f", params[KD]);
          break;
        case OFFSET:
          sprintf_(lcdBuffer_top,    "Setup: GrpOffset");
          sprintf_(lcdBuffer_bottom, "            %4.1f", params[OFFSET]);
          break;
        case BANG_HIGH:
          sprintf_(lcdBuffer_top,    "Setup: Bang High");
          sprintf_(lcdBuffer_bottom, "              %2.0f", params[BANG_HIGH]);
          break;
        case BANG_LOW:
          sprintf_(lcdBuffer_top,    "Setup:  Bang Low");
          sprintf_(lcdBuffer_bottom, "             -%2.0f", params[BANG_LOW]);
          break;
        case MAX_TEMP:
          sprintf_(lcdBuffer_top,    "Setup:  Max Temp");
          sprintf_(lcdBuffer_bottom, "             %3.0f", params[MAX_TEMP]);
          break;
      }
    }

    lcd.setCursor(0, 0);
    lcd.print(lcdBuffer_top);
    lcd.setCursor(0, 1);
    lcd.print(lcdBuffer_bottom);

    switch (tempState) {
      case TEMP_STATE::TEMP_LOW:
        lcd.setBacklight(BLUE);
        break;
      case TEMP_STATE::TEMP_HIGH:
        lcd.setBacklight(RED);
        break;
      case TEMP_STATE::UNSTABLE:
        lcd.setBacklight(VIOLET);
        break;
      case TEMP_STATE::STABLE:
        lcd.setBacklight(GREEN);
        break;
    }
  }
}
