#include "Arduino_LED_Matrix.h"
#include <Servo.h>
#include "dht_nonblocking.h"
#define DHT_SENSOR_TYPE DHT_TYPE_11

// Temporary motor pins

// -------------------- Pins --------------------
const int PUMP_MOTOR = 10; //D10
const int WATER_HIGH = 0; //D0
const int WATER_READ = 5; //A5
const int MOISTURE_READ = 0; //A0
const int TEMP_READ = 1; //A1
const int DHT_SENSOR_PIN = 8; //D8

Servo pump; // Pump uses PPM signal, so it needs to use the servo library

// How frequently we want to sample the sensors
// Essentially our clock cycle time, which recall is the inverse of frequency.
const unsigned long SENSOR_SAMPLE_MS = 100UL; // read sensors every 1s

// -------------------- Moisture Calibration --------------------
// You MUST calibrate these for your sensor + soil:
// - MOISTURE_DRY: reading when probe is in dry soil (or in air)
// - MOISTURE_WET: reading when probe is in fully wet soil (water-saturated)
// Many capacitive sensors read HIGH when dry and LOWER when wet, but it varies.
const int MOISTURE_DRY = 475; // Read from A0 with moisture sensor in the air.
const int MOISTURE_WET = 210; // Read from A0 with moisture sensor in damp soil/a cup of water

// -------------------- Watering Policy --------------------
// Base thresholds (in % moisture)
const float ON_THRESHOLD_BASE  = 0.25;  // water when moisture drops below this
const float OFF_THRESHOLD_BASE = 0.75;  // stop when moisture rises above this, too wet!
const int WATER_THRESHOLD = 100; // Threshold for when we detect current going thru the water to verify that the water is in the container.

// Temperature adjustment (simple, practical)
// Hotter -> water sooner (raise thresholds slightly) and/or water a bit longer
// Colder -> water later (lower thresholds)
const float HOT_TEMP_C  = 30.0;
const float COLD_TEMP_C = 12.0;
const float TEMP_ADJUST_MAX = 7.0; // max +/- percent points added/subtracted

// Pump timing safety
const char PUMP_ON_DEGREE = 0; // Value from 0 to 80 describing the speed of the pump. 0 fastest 80 slowest.
const unsigned long MAX_PUMP_ON_MS = 10UL * 1000UL; // 10 seconds max per cycle

// -------------------- State --------------------
bool pumpOn = false;
unsigned long pumpStartMs = 0;
unsigned long lastWaterMs = 0;
unsigned long lastSampleMs = 0;

ArduinoLEDMatrix matrix;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

enum class State {
  Loop,
  Read,
  EnablePump,
  DisablePump,
  Error,
};

State currentState;

void setup() {
  pump.attach(PUMP_MOTOR);

  // put your setup code here, to run once:
  //pinMode(PUMP_MOTOR, OUTPUT);
  pinMode(WATER_HIGH, OUTPUT);
  digitalWrite(WATER_HIGH, HIGH);
  
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the digital pin as an output.
  currentState = State::Loop;
  Serial.begin(9600);
}

void pumpSet(bool on) {
  pumpOn = on;

  //analogWrite(PUMP_MOTOR, pumpOn ? PUMP_ON_DEGREE : LOW); // enable on
  pump.write(pumpOn ? PUMP_ON_DEGREE : 90); // 90 deg is off

  if (on) pumpStartMs = millis();
}

float getMoistPct(int raw) {
  return (float)(raw - MOISTURE_DRY) / (MOISTURE_WET - MOISTURE_DRY); // Derived from lerp equation to get moisture pct
}

void loop() {
  unsigned long now = millis();

  switch (currentState) {
    case State::Loop: {
        if (now - lastSampleMs <= SENSOR_SAMPLE_MS) return;

        currentState = State::Read;
        break;
    }
    case State::Read: {
      lastSampleMs = now;

      // Read moisture
      int rawMoist = analogRead(MOISTURE_READ);
      float moistPct = getMoistPct(rawMoist);

      // Need to store temperature reading across state
      // Only sample when reading successful!

      // Read temperature
      float tempC = 0; 
      float humidity = 0;

      bool readSuccess = dht_sensor.measure(&tempC, &humidity);

      int rawWaterPresent = analogRead(WATER_READ);
      bool waterPresent = true; //rawWaterPresent >= WATER_THRESHOLD;

      bool pumpCdExceeded = now - pumpStartMs > MAX_PUMP_ON_MS;

      if (!pumpOn && waterPresent && moistPct <= ON_THRESHOLD_BASE) {
        currentState = State::EnablePump;
        break;
      } else if (pumpOn && (!waterPresent || moistPct >= OFF_THRESHOLD_BASE || pumpCdExceeded)) {
        currentState = State::DisablePump;
        break;
      } else if (!waterPresent) {
        currentState = State::Error;
        break;
      }

      currentState = State::Loop;

      // Debug output
      Serial.print("rawMoist=");
      Serial.print(rawMoist);
      Serial.print(" rawWater=");
      Serial.print(rawWaterPresent);
      Serial.print(" moist%=");
      Serial.print(moistPct, 1);

      Serial.print(" tempC=");
      Serial.print(tempC, 1);
      // if (tempValid) Serial.print(tempC, 1);
      // else Serial.print("NA");

      Serial.print(" pumpOn=");
      Serial.println(pumpOn ? "YES" : "NO");
      Serial.print(" tempSuccess=");
      Serial.println(readSuccess ? "YES" : "NO");

      break;
    }
    case State::EnablePump: {
      pumpSet(true);
      currentState = State::Loop;
      break;
    }
    case State::DisablePump: {
      pumpSet(false);
      currentState = State::Loop;
      break;
    }
    case State::Error: {
        Serial.println("ERROR! NO WATER PRESENT!!");
        currentState = State::Loop;
        break;
    }
  }
}
