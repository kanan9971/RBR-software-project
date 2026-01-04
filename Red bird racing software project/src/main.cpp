#include <Arduino.h>


// Pin definitions (based on project drawing: start button on PC4 = A4, buzzer on PD5 = 5)
#define APPS_5V_PIN A0    // PC0 (analog)
#define APPS_3V3_PIN A1   // PC1 (analog)
#define BRAKE_PIN A3      // PC3 (analog)
#define START_BUTTON_PIN A4  // PC4 (digital, use A4 for analog pin as digital)
#define BRAKE_LIGHT_PIN 2    // PD2 (digital)
#define DRIVE_LED_PIN 3      // PD3 (digital)
#define BUZZER_PIN 5         // PD5 (digital)

// Thresholds (adjust based on hardware/testing; brake > this = depressed)
const int BRAKE_THRESHOLD = 200;  // 0-1023 scale
const int APPS_FAULT_THRESHOLD_PCT = 10;  // 10% difference for fault
const unsigned long STARTIN_HOLD_TIME = 2000;  // 2 seconds
const unsigned long BUZZIN_TIME = 2000;        // 2 seconds
const unsigned long APPS_FAULT_TIME = 100;     // 100ms for persistent fault

const bool MOTOR_REVERSE = false;  // True to flip motor direction (negative torque)

// States enum
enum CarState {
  INIT,
  STARTIN,
  BUZZIN,
  DRIVE
};

// Global variables
CarState currentState = INIT;
unsigned long stateStartTime = 0;  // For state transition timing
unsigned long faultStartTime = 0;  // For APPS fault timing
bool isFaulty = false;
int16_t motorTorque = 0;  // Calculated torque (-32768 to 32767)

void setup() {
  // Set pin modes
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);  // Pullup for active LOW button
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
  int apps5v = analogRead(APPS_5V_PIN);     // 0-1023
  int apps3v3 = analogRead(APPS_3V3_PIN);   // 0-~675 (3.3V max)
  int brake = analogRead(BRAKE_PIN);        // 0-1023
  bool startButtonPressed = (digitalRead(START_BUTTON_PIN) == LOW);  // Assume LOW = pressed

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

      // Scale APPS_3V3 to match 5V range (0-1023 equivalent)
      float scaledApps3v3 = apps3v3 * (5.0 / 3.3);

      // Check fault: difference >10% of full scale (1023)
      float diff = abs(apps5v - scaledApps3v3);
      float faultThreshold = (APPS_FAULT_THRESHOLD_PCT / 100.0) * 1023.0;

      if (diff > faultThreshold) {
        if (!isFaulty) {
          faultStartTime = millis();
          isFaulty = true;
        }
        // If persistent >100ms, fault out to INIT
        if (millis() - faultStartTime >= APPS_FAULT_TIME) {
          motorTorque = 0;
          currentState = INIT;
          stateStartTime = millis();
          isFaulty = false;
          break;
        }
      } else {
        isFaulty = false;  // Reset fault
      }

      // Calculate torque (only if no fault)
      if (!isFaulty) {
        // Average scaled APPS and normalize to 0-1
        float avgApps = (apps5v + scaledApps3v3) / 2.0;
        float normalized = avgApps / 1023.0;

        // Scale to torque range (0 to 32767 forward)
        motorTorque = (int16_t)(normalized * 32767.0);

        // Flip to negative if MOTOR_REVERSE is true
        if (MOTOR_REVERSE) {
          motorTorque = -motorTorque;  // -32768 to 0
        }
      } else {
        motorTorque = 0;
      }
      break;
  }
}

  