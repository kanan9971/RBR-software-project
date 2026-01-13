#include <Arduino.h>

// Pin definitions (using direct PIN_PCx as suggested)
#define APPS_5V_PIN   PIN_PC0  // Analog input for 5V APPS
#define APPS_3V3_PIN  PIN_PC1  // Analog input for 3.3V APPS
#define BRAKE_PIN     PIN_PC3  // Analog input for brake
#define START_BUTTON_PIN PIN_PC4  // Digital input (active LOW)
#define BRAKE_LIGHT_PIN  PIN_PD2  // Digital output
#define DRIVE_LED_PIN    PIN_PD3  // Digital output
#define BUZZER_PIN       PIN_PD4  // Digital output (corrected from PD5 in your code, per project PDF)

// Thresholds and constants (using unsigned types)
const uint16_t BRAKE_THRESHOLD = 200;  // 0-1023 scale, brake depressed if > this
const uint16_t APPS_FAULT_THRESHOLD = 102;  // Absolute diff for 10% fault (≈10% of 1023)
const uint32_t STARTIN_HOLD_TIME = 2000UL;  // 2 seconds
const uint32_t BUZZIN_TIME = 2000UL;        // 2 seconds
const uint32_t APPS_FAULT_TIME = 100UL;     // 100ms for persistent fault
const bool MOTOR_REVERSE = false;           // True to flip motor direction (negative torque)

// States enum
enum CarState {
  INIT,
  STARTIN,
  BUZZIN,
  DRIVE
};

// Global variables
CarState currentState = INIT;
uint32_t stateStartTime = 0;  // For state transition timing
uint32_t faultStartTime = 0;  // For APPS fault timing
bool isFaulty = false;
int16_t motorTorque = 0;      // Calculated torque (-32768 to 32767)

// Function to check for pedal fault (returns true if persistent fault detected)
bool checkPedalFault(uint16_t apps5v, uint16_t apps3v3) {
  // Scale APPS_3V3 to 0-1023 range using integer math (50/33 ≈ 5/3.3)
  uint16_t scaled_apps3v3 = (static_cast<uint32_t>(apps3v3) * 50UL) / 33UL;

  // Calculate absolute difference
  uint16_t diff = (apps5v > scaled_apps3v3) ? (apps5v - scaled_apps3v3) : (scaled_apps3v3 - apps5v);

  if (diff > APPS_FAULT_THRESHOLD) {
    if (!isFaulty) {
      faultStartTime = millis();
      isFaulty = true;
    }
    // Check if fault persists > 100ms
    if (millis() - faultStartTime >= APPS_FAULT_TIME) {
      return true;  // Persistent fault
    }
  } else {
    isFaulty = false;
  }
  return false;  // No persistent fault
}

// Function to calculate torque from pedal readings (assumes no fault)
int16_t calculateTorque(uint16_t apps5v, uint16_t apps3v3) {
  // Scale APPS_3V3 to 0-1023 range
  uint16_t scaled_apps3v3 = (static_cast<uint32_t>(apps3v3) * 50UL) / 33UL;

  // Average the two APPS readings
  uint32_t avg_apps = (static_cast<uint32_t>(apps5v) + static_cast<uint32_t>(scaled_apps3v3)) / 2UL;

  // Map to torque (0 to 32767) using integer math
  int16_t torque = static_cast<int16_t>((avg_apps * 32767LL) / 1023LL);

  // Reverse if needed
  if (MOTOR_REVERSE) {
    torque = -torque;
  }
  return torque;
}

void setup() {
  // Set pin modes (explicit for analogs, as per feedback)
  pinMode(APPS_5V_PIN, INPUT);
  pinMode(APPS_3V3_PIN, INPUT);
  pinMode(BRAKE_PIN, INPUT);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);  // Active LOW; could use INPUT if external pullup
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  pinMode(DRIVE_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Initial off states
  digitalWrite(BRAKE_LIGHT_PIN, LOW);
  digitalWrite(DRIVE_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  stateStartTime = millis();  // Start timer
}

void loop() {
  // Read inputs every loop
  uint16_t apps5v = analogRead(APPS_5V_PIN);    // 0-1023
  uint16_t apps3v3 = analogRead(APPS_3V3_PIN);  // 0-~675 (3.3V max)
  uint16_t brake = analogRead(BRAKE_PIN);       // 0-1023
  bool startButtonPressed = (digitalRead(START_BUTTON_PIN) == LOW);  // Active LOW

  // Always handle brake light (in all states)
  digitalWrite(BRAKE_LIGHT_PIN, (brake > BRAKE_THRESHOLD) ? HIGH : LOW);

  // State machine
  switch (currentState) {
    case INIT:
      motorTorque = 0;
      digitalWrite(DRIVE_LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);

      // Transition to STARTIN if button pressed AND brakes depressed
      if (startButtonPressed && (brake > BRAKE_THRESHOLD)) {
        currentState = STARTIN;
        stateStartTime = millis();
      }
      break;

    case STARTIN:
      motorTorque = 0;
      digitalWrite(DRIVE_LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);

      // Back to INIT if brakes released OR button not held
      if (!startButtonPressed || (brake <= BRAKE_THRESHOLD)) {
        currentState = INIT;
        stateStartTime = millis();
        break;
      }

      // To BUZZIN if held >= 2s
      if (millis() - stateStartTime >= STARTIN_HOLD_TIME) {
        currentState = BUZZIN;
        stateStartTime = millis();
      }
      break;

    case BUZZIN:
      motorTorque = 0;
      digitalWrite(DRIVE_LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, HIGH);  // Buzzer on

      // To DRIVE after 2s (can release inputs)
      if (millis() - stateStartTime >= BUZZIN_TIME) {
        currentState = DRIVE;
        stateStartTime = millis();
        digitalWrite(BUZZER_PIN, LOW);  // Turn off buzzer
      }
      break;

    case DRIVE:
      digitalWrite(DRIVE_LED_PIN, HIGH);  // LED on
      digitalWrite(BUZZER_PIN, LOW);

      // Check for pedal fault and update torque
      if (checkPedalFault(apps5v, apps3v3)) {
        motorTorque = 0;
        currentState = INIT;
        stateStartTime = millis();
        isFaulty = false;
      } else {
        motorTorque = calculateTorque(apps5v, apps3v3);
      }
      break;
  }
}