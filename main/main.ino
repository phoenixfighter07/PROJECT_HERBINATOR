#include "Arduino_LED_Matrix.h"
#include <Servo.h>
#include "dht_nonblocking.h"
#include <EEPROM.h>
#define DHT_SENSOR_TYPE DHT_TYPE_11

// Enum declarations must come before any function definitions so that the
// Arduino IDE's auto-generated forward declarations can reference these types.
enum class State {
  MainLoop,
  RedFlash,
  WaterLoop,
  PumpOn,
  PumpOff,
  PumpCD,
  NoWater,
  PromptDry,
  PromptWet,
  CalibrationDone,
  ButtonPressed,
  CalibrationButtonYield,
  MoistureSensorFail,
  TempSensorFail
};

const char* stateNames[] = { 
  "MainLoop", 
  "RedFlash", 
  "WaterLoop", 
  "PumpOn", 
  "PumpOff", 
  "PumpCD", 
  "NoWater", 
  "PromptDry",
  "PromptWet",
  "CalibrationDone", 
  "ButtonPressed",
  "CalibrationButtonYield",
  "MoistureSensorFail",
  "TempSensorFail"
};

enum class LEDColor {
  Green,
  Blue,
  Red
};

// Temporary motor pins

// -------------------- Pins --------------------
const int PUMP_MOTOR = 10; //D10
const int WATER_HIGH = 0; //D0
const int WATER_READ = 5; //A5
const int MOISTURE_READ = 0; //A0
const int TEMP_READ = 1; //A1
const int DHT_SENSOR_PIN = 8; //D8
const int GREEN_LED = 5; //D5
const int BLUE_LED = 6; //D6
const int RED_LED = 7; //D7
const int BUTTON = 13; //D13

Servo pump; // Pump uses PPM signal, so it needs to use the servo library

// How frequently we want to sample the sensors
// Essentially our clock cycle time, which recall is the inverse of frequency.
const unsigned long SENSOR_SAMPLE_MS = 1000UL; // read sensors every 1s

// -------------------- Moisture Calibration --------------------
// You MUST calibrate these for your sensor + soil:
// - moistureDry: reading when probe is in dry soil (or in air)
// - moistureWet: reading when probe is in fully wet soil (water-saturated)
// Many capacitive sensors read HIGH when dry and LOWER when wet, but it varies.
const int IDEAL_MOISTURE_WET = 475; // Read from A0 with moisture sensor in the air.
const int IDEAL_MOISTURE_DRY = 210; // Read from A0 with moisture sensor in damp soil/a cup of water

// -------------------- Watering Policy --------------------
// Base thresholds (in % moisture)
const float ON_THRESHOLD_BASE  = 0.25;  // water when moisture drops below this
const float OFF_THRESHOLD_BASE = 0.75;  // stop when moisture rises above this, too wet!
const int WATER_THRESHOLD = 500; // Threshold for when we detect current going thru the water to verify that the water is in the container.
const float MOIST_FAIL_GRACE = 0.20f; // The percent +/- added to the threshold clamp to ensure proper operation of the moisture sensor
const int MIN_RAW_MOIST = 110; // Minimum moisture necessary to ensure proper sensor operation

// -------------------- Vapor Pressure Deficit (VPD) Adjustment --------------------
// VPD (kPa) measures evaporative demand. Higher VPD = plant transpires faster.
// VPD = SVP(T) - (RH/100)*SVP(T), where SVP uses the Magnus formula.
// Low VPD  (< VPD_LOW_KPA)  -> low demand -> delay watering (lower ON threshold)
// High VPD (> VPD_HIGH_KPA) -> high demand -> water sooner  (raise ON threshold)
const float VPD_LOW_KPA    = 0.4f;  // kPa — minimal evaporative stress
const float VPD_HIGH_KPA   = 1.2f;  // kPa — high evaporative stress
const float VPD_ADJUST_MAX = 0.1f; // max ± adjustment applied to ON_THRESHOLD_BASE (fraction)

// Pump timing safety
const char PUMP_ON_DEGREE = 0; // Value from 0 to 80 describing the speed of the pump. 0 fastest 80 slowest.
const unsigned long MAX_PUMP_ON_MS = 45UL * 1000UL; // 45 seconds max per cycle, let users set this!

// -------------------- State --------------------
bool pumpOn = false;
unsigned long pumpStartMs = 0;
unsigned long lastWaterMs = 0;
unsigned long lastSampleMs = 0;

// Last valid DHT11 readings — seeded to neutral indoor defaults
float lastTempC = 22.0f;
float lastHumidity = 50.0f;

int moistureDry = 0;
int moistureWet = 0;

const unsigned int SENSOR_WET_ADDRESS = 0; //Addresses 0 through 3
const unsigned int SENSOR_DRY_ADDRESS = 4; //Addresses 4 through 7
const unsigned int PUMP_CD_MINS = 5; // 5 minute failsafe!
const unsigned long MIN_PUMP_CD = 1000UL; //1 Second 
const unsigned long PUMP_CD_MILLIS = (unsigned long)PUMP_CD_MINS * 60UL * 1000UL;
const unsigned int TEMP_READ_FAIL_THRESHOLD = 10; // 10 Times allowed before failure is flagged!

const int calibrationHoldMs = 2000;
int btnDown = 0;

ArduinoLEDMatrix matrix;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

// -------------------- VPD Helpers --------------------

// VPD is tells us how quickly the plant is losing water through transpiration (Evaporation through leaves).
// Higher VPD means the air dries the plant quickly (hot/dry air), so the watering threshold needs to be lowered.
// Lower VPD means the air dries the plant slower (cool/humid air), so the watering threshold needs to be increased.

// Magnus formula: saturation vapor pressure in kPa
float saturationVaporPressure(float tempC) {
  return 0.6108f * exp(17.27f * tempC / (tempC + 237.3f));
}

// VPD = SVP - actual vapor pressure (kPa)
float computeVPD(float tempC, float humidity) {
  float svp = saturationVaporPressure(tempC);
  return svp * (1.0f - humidity / 100.0f);
}

// Returns ON_THRESHOLD_BASE shifted by VPD-derived demand.
float getAdjustedOnThreshold(float tempC, float humidity) {
  float vpd = computeVPD(tempC, humidity);
  // Normalise VPD into [0,1] over the expected operating range
  float t = (constrain(vpd, VPD_LOW_KPA, VPD_HIGH_KPA) - VPD_LOW_KPA)
            / (VPD_HIGH_KPA - VPD_LOW_KPA);
  // Map to [-VPD_ADJUST_MAX, +VPD_ADJUST_MAX]
  float adjust = (t - 0.5f) * 2.0f * VPD_ADJUST_MAX;
  return constrain(ON_THRESHOLD_BASE + adjust, 0.05f, 0.95f);
}

State currentState;

int getLEDPin(LEDColor LED) {
  switch (LED) {
    case LEDColor::Green: {
      return GREEN_LED;
    }
    case LEDColor::Blue: {
      return BLUE_LED;
    }
    case LEDColor::Red: {
      return RED_LED;
    }
  }
}

void setLEDEnabled(LEDColor LED, bool enabled) {
  digitalWrite(getLEDPin(LED), enabled ? HIGH : LOW);
}

void disableLEDS() {
  setLEDEnabled(LEDColor::Green, false);
  setLEDEnabled(LEDColor::Blue, false);
  setLEDEnabled(LEDColor::Red, false);
}

void enableLEDS() {
  setLEDEnabled(LEDColor::Green, true);
  setLEDEnabled(LEDColor::Blue, true);
  setLEDEnabled(LEDColor::Red, true);
}

void setup() {
  pump.attach(PUMP_MOTOR);

  // put your setup code here, to run once:
  //pinMode(PUMP_MOTOR, OUTPUT);
  pinMode(WATER_HIGH, OUTPUT);
  digitalWrite(WATER_HIGH, HIGH);
  
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the digital pin as an output.

  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  pinMode(BUTTON, INPUT_PULLUP);

  setLEDEnabled(LEDColor::Green, true);
  setLEDEnabled(LEDColor::Blue, true);
  setLEDEnabled(LEDColor::Red, true);

  Serial.begin(9600);

  currentState = State::MainLoop;

  delay(1000);

  // CSV header — matches printCSVRow() column order
  Serial.println("timestamp_ms,state,rawMoist,moist_pct,on_thresh,temp_c,humidity_pct,vpd_kpa,dht_ok,pump_on,water_raw,water_present");

  // Load initial values

  EEPROM.get(SENSOR_WET_ADDRESS, moistureWet);
  EEPROM.get(SENSOR_DRY_ADDRESS, moistureDry);
  disableLEDS();
}

int lastLEDFlash = 0;

void processFlash(LEDColor LED, int flashTimeMs) {
  if (millis() - lastLEDFlash < flashTimeMs) return;

  lastLEDFlash = millis();
  setLEDEnabled(LED, digitalRead(getLEDPin(LED)) == LOW);
}

void pumpSet(bool on) {
  pumpOn = on;

  //analogWrite(PUMP_MOTOR, pumpOn ? PUMP_ON_DEGREE : LOW); // enable on
  pump.write(pumpOn ? PUMP_ON_DEGREE : 90); // 90 deg is off

  if (on) {
    pumpStartMs = millis();
  } else {
    lastWaterMs = millis();
  }
}
int flashBegin = 0;
bool lastBtnDown = false;
unsigned long pumpOffCDTime = 0;    // millis() timestamp when pump cooldown ends
unsigned long now = 0;
State preButtonState = State::PromptDry; // where ButtonPressed should return on early release

bool isButtonDown() {
  return digitalRead(BUTTON) == LOW;
}

void beginRedFlash() {
  disableLEDS();
  flashBegin = millis();
  currentState = State::RedFlash;
}

bool canReadSensors() {
  bool canRead = (now - lastSampleMs >= SENSOR_SAMPLE_MS);

  if (canRead) {
    lastSampleMs = now;
  }

  return canRead;
}

int readFailCount = 0;
bool tempSensorFail = false;

bool readDHT() {
    float tempC = 0;
    float humidity = 0;
    bool readSuccess = dht_sensor.measure(&tempC, &humidity);

    //tempC -= 4;
    
    if (readSuccess) {
      tempSensorFail = false;
      readFailCount = 0; // Reset
      lastTempC = tempC;
      lastHumidity = humidity;
    } else {
      readFailCount++;

      if (readFailCount >= TEMP_READ_FAIL_THRESHOLD) {
        tempSensorFail = true;
      }
    }

    return readSuccess;
}

bool moistureSensorFail = false;
bool validCalibration = false;

float readMoistureSensor(int *rawMoist) {
  *rawMoist = analogRead(MOISTURE_READ);

  float pct = (float)(*rawMoist - moistureDry) / (moistureWet - moistureDry); // Derived from lerp equation to get moisture pct
  moistureSensorFail = validCalibration && (pct < 0 - MOIST_FAIL_GRACE || pct > 1 + MOIST_FAIL_GRACE) || *rawMoist < MIN_RAW_MOIST;

  return pct;
}

void printCSVRow(const char* state, int rawMoist, float moistPct, float onThresh,
                 bool dhtOK, int rawWater, bool waterPresent) {
  Serial.print(millis());        Serial.print(',');
  Serial.print(state);           Serial.print(',');
  Serial.print(rawMoist);        Serial.print(',');
  Serial.print(moistPct, 2);     Serial.print(',');
  Serial.print(onThresh, 2);     Serial.print(',');
  Serial.print(lastTempC, 1);    Serial.print(',');
  Serial.print(lastHumidity, 1); Serial.print(',');
  Serial.print(computeVPD(lastTempC, lastHumidity), 3); Serial.print(',');
  Serial.print(dhtOK ? 1 : 0);  Serial.print(',');
  Serial.print(pumpOn ? 1 : 0);  Serial.print(',');
  Serial.print(rawWater);        Serial.print(',');
  Serial.println(waterPresent ? 1 : 0);
}

void loop() {
  now = millis();

  bool btnDownNow = isButtonDown();
  if (btnDownNow && btnDownNow != lastBtnDown) {
    btnDown = now;
  }
  lastBtnDown = btnDownNow;

  //Serial.println(stateNames[(int) currentState]);

  // Failure states
  if (tempSensorFail) {
    currentState = State::TempSensorFail;
  } else if (moistureSensorFail) {
    currentState = State::MoistureSensorFail;
  }

  switch (currentState) {


    // ── MainLoop ─────────────────────────────────────────────────────
    // Hub state: validate calibration, check for button-initiated
    // calibration, detect missing water, then enter WaterLoop.
    case State::MainLoop: {
      setLEDEnabled(LEDColor::Green, true);
      setLEDEnabled(LEDColor::Blue, false);
      setLEDEnabled(LEDColor::Red, false);

      // Button held → enter calibration
      if (isButtonDown()) {
        if (now - btnDown > calibrationHoldMs) {
          Serial.println("User initiated calibration!");
          beginRedFlash();
        }
        break;  // stay in MainLoop while button is held but not long enough
      }

      // Bad calibration values → force recalibration
      validCalibration = moistureWet >= 0 && moistureDry <= 1024 && moistureWet < moistureDry;

      if (!validCalibration) {
        Serial.println("Bad calibration values!");
        beginRedFlash();
        break;
      }

      // Everything OK → enter watering mode
      currentState = State::WaterLoop;
      break;
    }

    // ── RedFlash ─────────────────────────────────────────────────────
    // Flash red for 3 seconds, then start dry calibration prompt.
    case State::RedFlash: {
      processFlash(LEDColor::Red, 500);

      if (now - flashBegin > 3000) {
        disableLEDS();
        currentState = State::PromptDry;
      }
      break;
    }

    // ── PromptDry ────────────────────────────────────────────────────
    // Flash green LED — waiting for user to press & hold button with
    // sensor in dry soil / air.
    case State::PromptDry: {
      setLEDEnabled(LEDColor::Blue, false);
      setLEDEnabled(LEDColor::Red, false);
      processFlash(LEDColor::Green, 1000);

      if (isButtonDown()) {
        preButtonState = State::PromptDry;
        currentState = State::ButtonPressed;
      }
      break;
    }

    // ── PromptWet ────────────────────────────────────────────────────
    // Flash blue LED — waiting for user to press & hold button with
    // sensor in wet soil / water.
    case State::PromptWet: {
      setLEDEnabled(LEDColor::Green, false);
      setLEDEnabled(LEDColor::Red, false);
      processFlash(LEDColor::Blue, 1000);

      if (isButtonDown()) {
        preButtonState = State::PromptWet;
        currentState = State::ButtonPressed;
      }
      break;
    }

    // ── ButtonPressed ────────────────────────────────────────────────
    // Shared hold-detection state for calibration.  While held, stays
    // here.  On release before the hold threshold, returns to the
    // previous prompt.  On successful hold, records the calibration
    // value and advances.
    case State::ButtonPressed: {
      if (!isButtonDown()) {
        // Released too early — go back to the prompt
        currentState = preButtonState;
        break;
      }

      if (now - btnDown > calibrationHoldMs) {
        int wrote;
        readMoistureSensor(&wrote);

        if (preButtonState == State::PromptDry) {
          EEPROM.put(SENSOR_DRY_ADDRESS, wrote);
          moistureDry = wrote;
          Serial.print("Dry=");
          Serial.println(wrote);
          currentState = State::CalibrationButtonYield;
        } else {
          // preButtonState == State::PromptWet
          EEPROM.put(SENSOR_WET_ADDRESS, wrote);
          moistureWet = wrote;
          Serial.print("Wet=");
          Serial.println(wrote);
          currentState = State::CalibrationDone;
        }
      }
      // else: still holding, stay in ButtonPressed
      break;
    }

    // ── CalibrationButtonRelease ─────────────────────────────────
    // Waits for the button to be released.
    // Then, goes to prompt wet state.
    case State::CalibrationButtonYield: {
      enableLEDS();
      if (isButtonDown()) break;

      currentState = State::PromptWet;
      break;
    }

    // ── CalibrationDone ──────────────────────────────────────────────
    // Quick LED celebration, then back to MainLoop.
    case State::CalibrationDone: {
      disableLEDS();  delay(500);
      enableLEDS();   delay(500);
      disableLEDS();  delay(500);
      enableLEDS();   delay(500);
      disableLEDS();

      currentState = State::MainLoop;
      break;
    }

    // ── WaterLoop ────────────────────────────────────────────────────
    // Core monitoring state.  Reads sensors at SENSOR_SAMPLE_MS rate,
    // decides whether to start pumping, flags errors.
    case State::WaterLoop: {
      setLEDEnabled(LEDColor::Green, true);
      setLEDEnabled(LEDColor::Blue, false);
      setLEDEnabled(LEDColor::Red, false);

      if (!canReadSensors()) break;

      // ── Sensor reads ──
      int rawMoist;
      float moistPct = readMoistureSensor(&rawMoist);

      bool readSuccess = readDHT();

      float onThreshold = getAdjustedOnThreshold(lastTempC, lastHumidity);

      int rawWaterPresent = analogRead(WATER_READ);
      bool waterPresent = rawWaterPresent >= WATER_THRESHOLD;

      printCSVRow("WaterLoop", rawMoist, moistPct, onThreshold,
                  readSuccess, rawWaterPresent, waterPresent);

      // ── No water → Error ──
      if (!waterPresent) {
        currentState = State::NoWater;
        break;
      }

      // ── Moisture low → start pumping ──
      if (moistPct <= onThreshold) {
        currentState = State::PumpOn;
        break;
      }

      if (now - lastWaterMs < MIN_PUMP_CD) break;

      currentState = State::MainLoop;
      break;
    }

    // ── PumpOn ───────────────────────────────────────────────────
    // Pump is running.  Re-reads sensors each cycle; stays here while
    // moisture is still low and pump hasn't run too long.
    case State::PumpOn: {
      setLEDEnabled(LEDColor::Blue, true);
      setLEDEnabled(LEDColor::Green, false);
      setLEDEnabled(LEDColor::Red, false);

      if (!pumpOn) pumpSet(true);  // turn on once on first entry

      if (isButtonDown()) {
        currentState = State::PumpOff;
        break;
      }

      if (!canReadSensors()) break;

      int rawMoist;
      float moistPct = readMoistureSensor(&rawMoist);

      int rawWaterPresent = analogRead(WATER_READ);
      bool waterPresent = rawWaterPresent >= WATER_THRESHOLD;

      bool dhtOK = readDHT();
      printCSVRow("PumpOn", rawMoist, moistPct,
                  getAdjustedOnThreshold(lastTempC, lastHumidity),
                  dhtOK, rawWaterPresent, waterPresent);

      bool pumpTooLong = (now - pumpStartMs) > MAX_PUMP_ON_MS;

      // Exit conditions: moisture satisfied OR pump safety timeout
      if (moistPct >= OFF_THRESHOLD_BASE || pumpTooLong || !waterPresent) {
        currentState = State::PumpOff;
        break;
      }

      // Otherwise keep pumping (self-loop)
      break;
    }

    // ── PumpOff ──────────────────────────────────────────────────
    // Turns pump off.  Routes to PumpCD if pump ran too long (safety),
    // otherwise straight back to MainLoop.
    case State::PumpOff: {
      bool wasTooLong = (now - pumpStartMs) > MAX_PUMP_ON_MS;
      pumpSet(false);
      disableLEDS();

      if (isButtonDown()) {
        currentState = State::MainLoop;
        break;
      }

      if (wasTooLong) {
        pumpOffCDTime = now + PUMP_CD_MILLIS;
        currentState = State::PumpCD;
      } else {
        currentState = State::MainLoop;
      }
      break;
    }

    // ── PumpCD (Pump Cooldown) ───────────────────────────────────────
    // Enforces a cooldown period after the pump ran for too long.
    // Flashes red while cooling down, then returns to MainLoop.
    case State::PumpCD: {
      processFlash(LEDColor::Red, 1000);
      setLEDEnabled(LEDColor::Green, false);
      setLEDEnabled(LEDColor::Blue, false);

      if (now >= pumpOffCDTime) {
        pumpOffCDTime = 0;
        disableLEDS();
        currentState = State::MainLoop;
      }
      // else: still cooling down (self-loop)
      break;
    }

    // ── Error (No Water) ─────────────────────────────────────────────
    // Water reservoir empty.  Self-loops until water is detected,
    // then returns to WaterLoop.
    case State::NoWater: {
      setLEDEnabled(LEDColor::Red, true);
      setLEDEnabled(LEDColor::Green, false);
      setLEDEnabled(LEDColor::Blue, true);

      pumpSet(false);

      int rawWater = analogRead(WATER_READ);
      if (rawWater >= WATER_THRESHOLD) {
        disableLEDS();
        currentState = State::WaterLoop;
      }
      break;
    }

    case State::MoistureSensorFail: {
      pumpSet(false); // Just in case

      processFlash(LEDColor::Red, 200);
      setLEDEnabled(LEDColor::Green, false);
      setLEDEnabled(LEDColor::Blue, true);

      if (!canReadSensors()) break;

      int rawMoist; 
      float moistPct = readMoistureSensor(&rawMoist);

      printCSVRow("MoistureFail", rawMoist, moistPct,
                  getAdjustedOnThreshold(lastTempC, lastHumidity),
                  false, -1, false);

      if (moistureSensorFail) break; // Still not reading properly

      currentState = State::MainLoop; // Reset to main loop
      break;
    }

    case State::TempSensorFail: {
      pumpSet(false); // Just in case

      processFlash(LEDColor::Red, 200);
      setLEDEnabled(LEDColor::Green, true);
      setLEDEnabled(LEDColor::Blue, false);

      if (!canReadSensors()) break;

      readDHT(); // Attempt to read

      if (tempSensorFail) break; // Still not reading properly

      currentState = State::MainLoop; // Reset to main loop
      break;
    }
  }
}
