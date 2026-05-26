/**
 * @file ESPMain.ino
 * Brain for ESP Herbinator units. Uses a FSM to read sensor data
 * and decide when to water the plant.
 *
 * @author Landon Wardle
 * @date 5/8/2026
 * @version 1.0
 */

 /* Standard Library Imports */
#include <DHT11.h>
#include <ESP32Servo.h>
#include <ESP32PWM.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>

/**
 * Store last value for:
 * Temperature
 * Moisture
 * Humidity
 * State 
 * Time (Full timestamp)
 * Unit name
*/

/* Exposed variables for JSON */

/* Last valid DHT11 readings. */
float lastTempC = 22.0f;
float lastHumidity = 50.0f;

/* Last valid moisture */
float lastMoisturePct = 0.0f;

/* State */
const char* currentStateName = "";

/* State Enums */
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

/* State Names in an Array for simplicity */
const char* const stateNames[] = { 
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

/* LED Colors */
enum class LEDColor {
  Green,
  Blue,
  Red
};

/* ESP32 Settings -----------------------------------------------------------------*/


/* Pins */
const int PUMP_MOTOR = 7;
const int WATER_HIGH = 0;
const int WATER_READ = 10;
const int MOISTURE_READ = 4;
const int DHT_SENSOR_PIN = 5;
const int GREEN_LED = 13;
const int BLUE_LED = 12;
const int RED_LED = 11;
const int BUTTON = 6;
const int SD_CS = 18;
const int SD_MOSI = 8;
const int SD_SCK = 3; // Clk
const int SD_MISO = 46;

/* The baud rate to the serial monitor. */
const unsigned int BAUD_RATE = 9600;

/**
 * The max value the Microcontroller can read of an analog signal
 * Used for verifying if the recorded moisture calibration is legit
*/
const unsigned int ANALOG_MAX = 4096;

/**
 * How frequently the sensors will sample.
 * Essentially our clock cycle time, which recall is the inverse of frequency.
*/
const unsigned long SENSOR_SAMPLE_MS = 1000UL;

/* Constant for when water reading fails. */
const int NO_WATER_READING = -1;

/* How long Herbinator waits until starting main FSM after initialization. */
const unsigned int INIT_DELAY_MS = 1000;

/* How long to hold the calibration button to switch to calibration mode. */
const unsigned int CALIBRATION_HOLD_MS = 2000;

/* SD Settings ---------------------------------------------------------------------- */



/* The reference to the filename to write logs to. */
const char* LOG_FILENAME = "/herbinator_logs.txt";

/* The header of the .csv file the logs write to. */
const char* CSV_HEADER = "timestamp_ms,state,rawMoist,moist_pct,on_thresh,temp_c,humidity_pct,vpd_kpa,dht_ok,pump_on,water_raw,water_present";

/* How long between each attempt at flushing the sensor data to the log file. */
const unsigned long FLUSH_CYCLE_TIME = 30UL * 1000UL;

/* The frequency the SD class operates at */
const unsigned long SD_FREQUENCY = 4000000UL;

/* Watering Settings -----------------------------------------------------------------*/



/**
 * How frequently the sensors will sample when in the
 * WaterLoop (Idle) state. This should be slower than
 * SENSOR_SAMPLE_MS because Herbinator is in idle the
 * majority of its operating time.
*/
const unsigned int WATER_LOOP_SAMPLE_MS = 5000UL;

/* Water threshold Herbinator must be below to trigger the pump. */
const float ON_THRESHOLD_BASE  = 0.25; 

/* Water threshold Herbinator cannot be above for the pump to be powered. */
const float OFF_THRESHOLD_BASE = 0.75;

/* Threshold for when we detect current going thru the water to verify that the water is in the container. */
const int WATER_THRESHOLD = 4000;

/* The percent +/- added to the threshold clamp to ensure proper operation of the moisture sensor. */
const float MOIST_FAIL_GRACE = 0.20f;

/* Minimum moisture expected of the moisture sensor to validate proper operation. */
const unsigned int MIN_RAW_MOIST = 110;

/* VPD Settings -----------------------------------------------------------------*/



/**
 *                 Vapor Pressure Deficit (VPD) Adjustment
 * 
 * VPD (kPa) measures evaporative demand. Higher VPD = plant transpires faster.
 * VPD = SVP(T) - (RH/100)*SVP(T), where SVP uses the Magnus formula.
 * Low VPD  (< VPD_LOW_KPA)  -> low demand -> delay watering (lower ON threshold)
 * High VPD (> VPD_HIGH_KPA) -> high demand -> water sooner  (raise ON threshold)
*/

/* Minimal evaporative stress in kPa. */
const float VPD_LOW_KPA = 0.4f;

/* High evaporative stress in kPa. */
const float VPD_HIGH_KPA = 1.2f;

/* The maximum VPD can influence ON_THRESHOLD_BASE. */
const float VPD_ADJUST_MAX = 0.1f;

/* Pump Settings -----------------------------------------------------------------*/



/* Value from 0 to 80 describing the speed of the pump. 0 fastest 80 slowest. */
const uint8_t PUMP_ON_DEGREE = 0;

/** 
 * The speed of the pump when turning it off. 
 * 90 stops operation completely.
*/
const uint8_t PUMP_OFF_DEGREE = 90;

/* The longest the pump can run for. */
const unsigned long MAX_PUMP_ON_MS = 45UL * 1000UL;

/* The default cooldown of the pump when turning the pump off. */
const unsigned long MIN_PUMP_CD = 1000UL;

/** 
 * The cooldown of the pump when it auto-
 * terminates after MAX_PUMP_ON_MS seconds.
*/
const unsigned long PUMP_CD_MILLIS = static_cast<unsigned long>(5 * 60UL * 1000UL);

/* Memory Settings ----------------------------------------------------------------- */



/** 
 * The address in EEPROM memory of where to store the 
 * wet value for the moisture sensor calibration. 
 * NOTE: The value being stored is an integer, so it 
 *       will take 3 addresses AFTER the address set
 *       here!
*/
const unsigned int SENSOR_WET_ADDRESS = 0;

/** 
 * The address in EEPROM memory of where to store the 
 * wet value for the moisture sensor calibration. 
 * NOTE: The value being stored is an integer, so it 
 *       will take 3 addresses AFTER the address set
 *       here!
*/
const unsigned int SENSOR_DRY_ADDRESS = 4;

/* The size of EEPROM memory to simulate. */
const unsigned int EEPROM_SIZE = 8;

/**
 * The number of consecutive moisture sensor fails 
 * that must occur before an error state is entered.
 */
const unsigned int TEMP_READ_FAIL_THRESHOLD = 10;

/* LED Settings -----------------------------------------------------------------*/



/* How long the RED LEDs flash for. */
const unsigned int RED_FLASH_DURATION_MS = 3000;

/* How long between flashes when using an LED to indicate a state. */
const unsigned int PROMPT_FLASH_INTERVAL_MS = 1000;
const unsigned int PROMPT_FLASH_INTERVAL_MS_FAST = 200;

/* How long between flashing the Red LED on the calibrate state. */
const unsigned int RED_FLASH_MS = 500;

/* How long to wait between each flash to show calibration is done. */
const unsigned int CALIBRATION_DONE_DELAY_MS = 500;

/* State  ----------------------------------------------------------------- */



/* General state variables. */
bool pumpOn = false;
bool lastBtnDown = false;
bool tempSensorFail = false;
bool moistureSensorFail = false;
bool validCalibration = false;

unsigned long lastLEDFlash = 0;
unsigned long pumpStartMs = 0;
unsigned long lastWaterMs = 0;
unsigned long lastSampleMs = 0;
unsigned long pumpOffCDTime = 0;
unsigned long btnDown = 0;
unsigned long now = 0;
unsigned long stateEnteredMs = 0;

unsigned int readFailCount = 0;
unsigned long flashBegin = 0;

/* Current calibration values for the moisture sensor. */
int moistureDry = 0;
int moistureWet = 0;

/* DHT Sensor object using DHT library */
DHT11 dhtSensor(DHT_SENSOR_PIN);

/* Pump object using servo library */
Servo pump;

/* Current state of Herbinator */
State currentState;

/* Where ButtonPressed should return on early release */
State preButtonState = State::PromptDry;

/**
 * SPI bus instance so we can pick the pins explicitly
 * On the ESP32-S3 this maps to the FSPI/HSPI peripheral.
 */
SPIClass sdSPI(FSPI);

/* Log File class. */
File logFile;

/* VPD Helpers ----------------------------------------------------------------- */



/* See documentation in VPD Settings for how VPD works. */

/**
 * Magnus formula: saturation vapor pressure in kPa
 * @param tempC the temperature in Celcius.
 * @return the pressure of the vapor with the given temperature tempC
 */
float saturationVaporPressure(const float tempC) {
  return 0.6108f * exp(17.27f * tempC / (tempC + 237.3f));
}

/**
 * Computes VPD.
 * @param tempC the temperature in Celcius.
 * @param humidity the humidity as a percent.
 * 
 * @note humidity should be given as a percent, not a value from 0 to 1!
 * 
 * @return VPD as a float value in kPa.
 */
float computeVPD(const float tempC, const float humidity) {
  float svp = saturationVaporPressure(tempC);
  return svp * (1.0f - humidity / 100.0f);
}

/**
 * Calculates the threshold for Herbinator to water the plant based off of VPD.
 * @param tempC the temperature in Celcius.
 * @param humidity the humidity as a percent.
 * 
 * @note humidity should be given as a percent, not a value from 0 to 1!
 * 
 * @return the adjusted threshold value for Herbinator to water.
 */
float getAdjustedOnThreshold(const float tempC, const float humidity) {
  float vpd = computeVPD(tempC, humidity);

  // Normalise VPD into [0,1] over the expected operating range
  float t = (constrain(vpd, VPD_LOW_KPA, VPD_HIGH_KPA) - VPD_LOW_KPA)
            / (VPD_HIGH_KPA - VPD_LOW_KPA);

  // Map to [-VPD_ADJUST_MAX, +VPD_ADJUST_MAX]
  float adjust = (t - 0.5f) * 2.0f * VPD_ADJUST_MAX;
  return constrain(ON_THRESHOLD_BASE + adjust, 0.05f, 0.95f);
}

/**
 * Calculates the threshold for Herbinator to water the plant based off of VPD.
 * @param LED the LED as an enum.
 * 
 * @return the pin on the microcontroller that leads to the pin.
 */
int getLEDPin(const LEDColor LED) {
  switch (LED) {
    case LEDColor::Green: return GREEN_LED;
    case LEDColor::Blue: return BLUE_LED;
    case LEDColor::Red: return RED_LED;
    default: return 0;
  }
}

/**
 * Sets an LED to be enabled.
 * @param LED the LED as an enum.
 * @param enabled if the LED is enabled.
 */
void setLEDEnabled(const LEDColor LED, const bool enabled) {
  digitalWrite(getLEDPin(LED), enabled ? HIGH : LOW);
}

/**
 * Disables all LEDs.
 */
void disableLEDS() {
  setLEDEnabled(LEDColor::Green, false);
  setLEDEnabled(LEDColor::Blue, false);
  setLEDEnabled(LEDColor::Red, false);
}

/**
 * Enables all LEDs.
 */
void enableLEDS() {
  setLEDEnabled(LEDColor::Green, true);
  setLEDEnabled(LEDColor::Blue, true);
  setLEDEnabled(LEDColor::Red, true);
}

/**
 * Transitions the FSM to a new state and records when it was entered.
 * Logs the transition to serial for debugging.
 *
 * @param next the state to transition to.
 */
void transitionTo(const State next) {
  currentState = next;
  stateEnteredMs = millis();
  currentStateName = stateNames[static_cast<int>(next)];

  Serial.print("Transition -> ");
  Serial.println(currentStateName);
}

/**
 * Called when the microcontroller initializes. Manages state initialization.
 */
void setup() {
  Serial.begin(BAUD_RATE);

  // CSV header — matches printCSVRow() column order
  dhtSensor.setDelay(SENSOR_SAMPLE_MS);

  pump.attach(PUMP_MOTOR);
  EEPROM.begin(EEPROM_SIZE);

  pinMode(WATER_HIGH, OUTPUT);
  digitalWrite(WATER_HIGH, HIGH);
  
  // Initialize the digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  pinMode(BUTTON, INPUT_PULLUP);

  setLEDEnabled(LEDColor::Green, true);
  setLEDEnabled(LEDColor::Blue, true);
  setLEDEnabled(LEDColor::Red, true);

  currentState = State::MainLoop;
  stateEnteredMs = millis();

  // SD setup
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  bool sdSuccess = SD.begin(SD_CS, sdSPI, SD_FREQUENCY);

  while (!sdSuccess) {
    setLEDEnabled(LEDColor::Green, false);
    setLEDEnabled(LEDColor::Blue, false);
    setLEDEnabled(LEDColor::Red, true);

    Serial.println("WARNING: SD Card failed to initialize! Please check wiring and if the SD is inserted!");
    delay(INIT_DELAY_MS);
  }

  Serial.println("SD Card initialized!");

  bool isNew = !SD.exists(LOG_FILENAME);
  logFile = SD.open(LOG_FILENAME, FILE_APPEND);
  if (isNew) logFile.println(CSV_HEADER);

  if (!logFile) {
    Serial.println("WARNING: Failure to open log file, continuing without logging!");
  }

  delay(INIT_DELAY_MS);

  // Load initial values

  EEPROM.get(SENSOR_WET_ADDRESS, moistureWet);
  EEPROM.get(SENSOR_DRY_ADDRESS, moistureDry);
  disableLEDS();
}

/**
 * manages LED flashing in error states.
 * 
 * @param LED the LED to manage the flashing of.
 * @param flashTimeMs how long between the LED switching states.
 */
void processFlash(const LEDColor LED, const int flashTimeMs) {
  if (millis() - lastLEDFlash < flashTimeMs) return;

  lastLEDFlash = millis();
  setLEDEnabled(LED, digitalRead(getLEDPin(LED)) == LOW);
}

/**
 * sets the pump on or off.
 * 
 * @param on the state of the pump.
 */
void pumpSet(const bool on) {
  pumpOn = on;

  pump.write(pumpOn ? PUMP_ON_DEGREE : PUMP_OFF_DEGREE);

  if (on) {
    pumpStartMs = millis();
  } else {
    lastWaterMs = millis();
  }
}

/**
 * checks if the calibration button is pressed.
 * 
 * @return a bool describing if the calibration button is pressed.
 */
bool isButtonDown() {
  return digitalRead(BUTTON) == LOW;
}

/**
 * helper method for beginning the red flash sequence.
 */
void beginRedFlash() {
  disableLEDS();
  flashBegin = millis();
  transitionTo(State::RedFlash);
}

/**
 * helper function for determining if sensors can be read.
 * 
 * @return returns a boolean describing if the sensors can be sampled from.
 */
bool canReadSensors() {
  return canReadSensors(SENSOR_SAMPLE_MS);
}

/**
 * helper function for determining if sensors can be read.
 * @param intervalMs how long to wait between sensor reads.
 * 
 * @return returns a boolean describing if the sensors can be sampled from.
 */
bool canReadSensors(const unsigned long intervalMs) {
  bool canRead = (now - lastSampleMs >= intervalMs);

  if (canRead) {
    lastSampleMs = now;
  }

  return canRead;
}

/**
 * helper function for reading the temperature sensor.
 * 
 * @return returns an int describing if the reading is valid. 0 for valid, 1 for invalid.
 */
int readDHT() {
    int tempC = 0;
    int humidity = 0;
    
    int readSuccess = dhtSensor.readTemperatureHumidity(tempC, humidity);
    
    if (readSuccess == 0) {
      tempSensorFail = false;
      readFailCount = 0; // Reset
      lastTempC = static_cast<float>(tempC);
      lastHumidity = static_cast<float>(humidity);
    } else {
      readFailCount++;

      if (readFailCount >= TEMP_READ_FAIL_THRESHOLD) {
        tempSensorFail = true;
      }
    }

    return readSuccess;
}

/**
 * helper function for reading the moisture sensor.
 * @param rawMosit a pointer to where to write the raw moisture value.
 * 
 * @return returns the percent moisture from 0-100% from the moisture sensor based on calibration.
 */
float readMoistureSensor(int *rawMoist) {
  *rawMoist = analogRead(MOISTURE_READ);

  float pct = (float)(*rawMoist - moistureDry) / (moistureWet - moistureDry); // Derived from lerp equation to get moisture pct
  moistureSensorFail = (validCalibration && (pct < -MOIST_FAIL_GRACE || pct > 1 + MOIST_FAIL_GRACE)) || *rawMoist < MIN_RAW_MOIST;

  lastMoisturePct = pct;

  return pct;
}

/**
 * helper function to print a CSV row.
 * @param state a pointer to the first character of an input string.
 * @param rawMoist the raw moisture value read from the moisture sensor
 * @param moistPct the percent moisture calculated from the moisture sensor
 * @param onThresh the current threshold of moisture (0-100%) to start the pump
 * @param dhtOK if the last DHT11 reading was valid
 * @param rawWater the raw voltage reading into an analog pin on the microcontroller for the float switch circuit
 * @param waterPresent a boolean describing if water is present or not
 */
void printCSVRow(const char* state, const int rawMoist, const float moistPct, const float onThresh,
                 const bool dhtOK, const int rawWater, const bool waterPresent) {
  if (!logFile) {
    Serial.println("WARNING: Log file not open, cannot write CSV row!");
    return;
  }

  logFile.print(millis());        logFile.print(',');
  logFile.print(state);           logFile.print(',');
  logFile.print(rawMoist);        logFile.print(',');
  logFile.print(moistPct, 2);     logFile.print(',');
  logFile.print(onThresh, 2);     logFile.print(',');
  logFile.print(lastTempC, 1);    logFile.print(',');
  logFile.print(lastHumidity, 1); logFile.print(',');
  logFile.print(computeVPD(lastTempC, lastHumidity), 3); logFile.print(',');
  logFile.print(dhtOK ? 1 : 0);  logFile.print(',');
  logFile.print(pumpOn ? 1 : 0);  logFile.print(',');
  logFile.print(rawWater);        logFile.print(',');
  logFile.println(waterPresent ? 1 : 0);
}

static unsigned long last_flush = 0;

/**
 * loop function that drives the FSM.
 */
void loop() {
  now = millis();

  if (logFile && millis() - last_flush > FLUSH_CYCLE_TIME) {
    logFile.flush();
    last_flush = millis();
    Serial.println("Flushing buffer to SD card!");
  }

  bool btnDownNow = isButtonDown();
  if (btnDownNow && btnDownNow != lastBtnDown) {
    btnDown = now;
  }
  lastBtnDown = btnDownNow;

  // Debug
  //Serial.printf("Last moisture percent is %f.\n", lastMoisturePct);
  //Serial.printf("Current state is %s.\n", currentStateName);

  // Failure states — preempt whatever state we were in
  if (tempSensorFail && currentState != State::TempSensorFail) {
    transitionTo(State::TempSensorFail);
  } else if (moistureSensorFail && currentState != State::MoistureSensorFail) {
    transitionTo(State::MoistureSensorFail);
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
        if (now - btnDown > CALIBRATION_HOLD_MS) {
          Serial.println("User initiated calibration!");
          beginRedFlash();
        }
        break;  // stay in MainLoop while button is held but not long enough
      }

      // Bad calibration values → force recalibration
      validCalibration = moistureWet >= 0 && moistureDry <= ANALOG_MAX && moistureWet < moistureDry;

      if (!validCalibration) {
        Serial.println("Bad calibration values!");
        beginRedFlash();
        break;
      }

      // Everything OK → enter watering mode
      transitionTo(State::WaterLoop);
      break;
    }

    // ── RedFlash ─────────────────────────────────────────────────────
    // Flash red for 3 seconds, then start dry calibration prompt.
    case State::RedFlash: {
      processFlash(LEDColor::Red, RED_FLASH_MS);

      if (now - flashBegin > RED_FLASH_DURATION_MS) {
        disableLEDS();
        transitionTo(State::PromptDry);
      }
      break;
    }

    // ── PromptDry ────────────────────────────────────────────────────
    // Flash green LED — waiting for user to press & hold button with
    // sensor in dry soil / air.
    case State::PromptDry: {
      setLEDEnabled(LEDColor::Blue, false);
      setLEDEnabled(LEDColor::Red, false);
      processFlash(LEDColor::Green, PROMPT_FLASH_INTERVAL_MS);

      if (isButtonDown()) {
        preButtonState = State::PromptDry;
        transitionTo(State::ButtonPressed);
      }
      break;
    }

    // ── PromptWet ────────────────────────────────────────────────────
    // Flash blue LED — waiting for user to press & hold button with
    // sensor in wet soil / water.
    case State::PromptWet: {
      setLEDEnabled(LEDColor::Green, false);
      setLEDEnabled(LEDColor::Red, false);
      processFlash(LEDColor::Blue, PROMPT_FLASH_INTERVAL_MS);

      if (isButtonDown()) {
        preButtonState = State::PromptWet;
        transitionTo(State::ButtonPressed);
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
        transitionTo(preButtonState);
        break;
      }

      if (now - btnDown > CALIBRATION_HOLD_MS) {
        int wrote;
        readMoistureSensor(&wrote);

        if (preButtonState == State::PromptDry) {
          EEPROM.put(SENSOR_DRY_ADDRESS, wrote);
          moistureDry = wrote;
          Serial.print("Dry=");
          Serial.println(wrote);
          transitionTo(State::CalibrationButtonYield);
        } else {
          EEPROM.put(SENSOR_WET_ADDRESS, wrote);
          moistureWet = wrote;
          Serial.print("Wet=");
          Serial.println(wrote);
          transitionTo(State::CalibrationDone);
        }
        // Save changes
        EEPROM.commit();
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

      transitionTo(State::PromptWet);
      break;
    }

    // ── CalibrationDone ──────────────────────────────────────────────
    // Quick LED celebration, then back to MainLoop.
    case State::CalibrationDone: {
      disableLEDS();  delay(CALIBRATION_DONE_DELAY_MS);
      enableLEDS();   delay(CALIBRATION_DONE_DELAY_MS);
      disableLEDS();  delay(CALIBRATION_DONE_DELAY_MS);
      enableLEDS();   delay(CALIBRATION_DONE_DELAY_MS);
      disableLEDS();

      transitionTo(State::MainLoop);
      break;
    }

    // ── WaterLoop ────────────────────────────────────────────────────
    // Core monitoring state.  Reads sensors at SENSOR_SAMPLE_MS rate,
    // decides whether to start pumping, flags errors.
    case State::WaterLoop: {
      setLEDEnabled(LEDColor::Green, true);
      setLEDEnabled(LEDColor::Blue, false);
      setLEDEnabled(LEDColor::Red, false);

      int rawWaterPresent = analogRead(WATER_READ);
      bool waterPresent = rawWaterPresent >= WATER_THRESHOLD;

      // ── No water → Error ──
      if (!waterPresent) {
        transitionTo(State::NoWater);
        break;
      }

      if (!canReadSensors(WATER_LOOP_SAMPLE_MS)) break;

      // ── Sensor reads ──
      int rawMoist;
      float moistPct = readMoistureSensor(&rawMoist);

      bool readSuccess = readDHT();

      float onThreshold = getAdjustedOnThreshold(lastTempC, lastHumidity);

      printCSVRow("WaterLoop", rawMoist, moistPct, onThreshold,
                  readSuccess, rawWaterPresent, waterPresent);

      // ── Moisture low → start pumping ──
      if (moistPct <= onThreshold) {
        transitionTo(State::PumpOn);
        break;
      }

      if (now - lastWaterMs < MIN_PUMP_CD) break;

      transitionTo(State::MainLoop);
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
        transitionTo(State::PumpOff);
        break;
      }

      if (!canReadSensors()) break;

      int rawMoist;
      float moistPct = readMoistureSensor(&rawMoist);

      int rawWaterPresent = analogRead(WATER_READ);
      bool waterPresent = rawWaterPresent >= WATER_THRESHOLD;

      printCSVRow("PumpOn", rawMoist, moistPct,
                  getAdjustedOnThreshold(lastTempC, lastHumidity),
                  false, rawWaterPresent, waterPresent);

      bool pumpTooLong = (now - pumpStartMs) > MAX_PUMP_ON_MS;

      // Exit conditions: moisture satisfied OR pump safety timeout
      if (moistPct >= OFF_THRESHOLD_BASE || pumpTooLong || !waterPresent) {
        transitionTo(State::PumpOff);
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
        transitionTo(State::MainLoop);
        break;
      }

      if (wasTooLong) {
        pumpOffCDTime = now + PUMP_CD_MILLIS;
        transitionTo(State::PumpCD);
      } else {
        transitionTo(State::MainLoop);
      }
      break;
    }

    // ── PumpCD (Pump Cooldown) ───────────────────────────────────────
    // Enforces a cooldown period after the pump ran for too long.
    // Flashes red while cooling down, then returns to MainLoop.
    case State::PumpCD: {
      processFlash(LEDColor::Red, PROMPT_FLASH_INTERVAL_MS);
      setLEDEnabled(LEDColor::Green, false);
      setLEDEnabled(LEDColor::Blue, false);

      if (now >= pumpOffCDTime) {
        pumpOffCDTime = 0;
        disableLEDS();
        transitionTo(State::MainLoop);
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
        transitionTo(State::WaterLoop);
      }
      break;
    }

    case State::MoistureSensorFail: {
      pumpSet(false); // Just in case

      processFlash(LEDColor::Red, PROMPT_FLASH_INTERVAL_MS_FAST);
      setLEDEnabled(LEDColor::Green, false);
      setLEDEnabled(LEDColor::Blue, true);

      if (!canReadSensors()) break;

      int rawMoist; 
      float moistPct = readMoistureSensor(&rawMoist);

      printCSVRow("MoistureFail", rawMoist, moistPct,
                  getAdjustedOnThreshold(lastTempC, lastHumidity),
                  false, NO_WATER_READING, false);

      if (moistureSensorFail) break; // Still not reading properly

      transitionTo(State::MainLoop); // Reset to main loop
      break;
    }

    case State::TempSensorFail: {
      pumpSet(false); // Just in case

      processFlash(LEDColor::Red, PROMPT_FLASH_INTERVAL_MS_FAST);
      setLEDEnabled(LEDColor::Green, true);
      setLEDEnabled(LEDColor::Blue, false);

      if (!canReadSensors()) break;

      readDHT(); // Attempt to read

      if (tempSensorFail) break; // Still not reading properly

      transitionTo(State::MainLoop); // Reset to main loop
      break;
    }

    // Fail case
    default: {
      Serial.print("ERROR: Unhandled state: ");
      Serial.println(stateNames[static_cast<int>(currentState)]);
      transitionTo(State::MainLoop);
      break;
    }
  }
}
