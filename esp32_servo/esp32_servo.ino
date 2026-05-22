/*
 * ESP32 + PCA9685 — 3-DOF Robotic Arm Serial Receiver
 * ----------------------------------------------------
 * Receives joint angles from pygame IK simulation over USB Serial.
 * 
 * Expected serial format: "shoulder,elbow,wrist\n"
 *   e.g. "90.0,120.5,45.3\n"
 *
 * Pin mapping:
 *   Pin 0 & 1 (coupled, opposite): Shoulder (bicep = 11 cm)
 *   Pin 2: Elbow (forearm = 8.5 cm)
 *   Pin 3: Wrist (end effector = 7.5 cm)
 *
 * Smooth interpolation prevents jerky motion.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#define LED_PIN 2
#define SERVO_MIN_TICKS 102
#define SERVO_MAX_TICKS 512
#define PCA9685_FREQ 50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Smooth servo: track current vs target ticks
float cur[6], tgt[6];
const float MAX_STEP = 3.0f; // ticks per loop iteration

// Serial receive buffer
String inputBuffer = "";

float degToTicks(float d) {
    d = fmax(0.0f, fmin(180.0f, d));
    return SERVO_MIN_TICKS + (d / 180.0f) * (SERVO_MAX_TICKS - SERVO_MIN_TICKS);
}

void updateServos() {
    for (uint8_t i = 0; i < 6; i++) {
        float diff = tgt[i] - cur[i];
        if (fabs(diff) <= MAX_STEP)
            cur[i] = tgt[i];
        else
            cur[i] += (diff > 0 ? MAX_STEP : -MAX_STEP);
        pwm.setPWM(i, 0, (uint16_t)cur[i]);
    }
}

void parseAndApply(String line) {
    // Parse "shoulder,elbow,wrist,wrist_roll,gripper"
    int c1 = line.indexOf(',');
    if (c1 < 0) return;
    int c2 = line.indexOf(',', c1 + 1);
    if (c2 < 0) return;
    int c3 = line.indexOf(',', c2 + 1);
    if (c3 < 0) return;
    int c4 = line.indexOf(',', c3 + 1);
    if (c4 < 0) return;

    float sh = line.substring(0, c1).toFloat();
    float el = line.substring(c1 + 1, c2).toFloat();
    float wr = line.substring(c2 + 1, c3).toFloat();
    float w_roll = line.substring(c3 + 1, c4).toFloat();
    float grip = line.substring(c4 + 1).toFloat();

    // Validate ranges
    if (sh < 0 || sh > 180 || el < 0 || el > 180 || wr < 0 || wr > 180 || w_roll < 0 || w_roll > 180 || grip < 0 || grip > 180)
        return;

    // Set targets
    tgt[0] = degToTicks(sh);              // Shoulder
    tgt[1] = degToTicks(180.0f - sh);     // Coupled shoulder (mirrored)
    tgt[2] = degToTicks(el);              // Elbow
    tgt[3] = degToTicks(wr);              // Wrist
    tgt[4] = degToTicks(w_roll);          // Wrist Roll (P04)
    tgt[5] = degToTicks(grip);            // Gripper (P05)

    // Blink LED on valid command
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Wire.begin(21, 22);
    pwm.begin();
    pwm.setPWMFreq(PCA9685_FREQ);
    delay(10);

    // Start all servos at 90° (default vertical pose)
    float center = degToTicks(90.0f);
    for (uint8_t i = 0; i < 4; i++) {
        cur[i] = center;
        tgt[i] = center;
        pwm.setPWM(i, 0, (uint16_t)center);
    }
    delay(500);

    Serial.println("READY");
}

// --- Sweep logic for Pin 15 ---
unsigned long lastSweepTime = 0;
int sweepState = 0; 
float cur_pin15 = 0;
float tgt_pin15 = 90.0f;
const unsigned long HOLD_TIME = 2000;

void updatePin15Sweep() {
    float tgt_ticks = degToTicks(tgt_pin15);
    float diff = tgt_ticks - cur_pin15;
    
    // Smooth step
    if (fabs(diff) <= MAX_STEP) {
        cur_pin15 = tgt_ticks;
    } else {
        cur_pin15 += (diff > 0 ? MAX_STEP : -MAX_STEP);
    }
    
    pwm.setPWM(15, 0, (uint16_t)cur_pin15);

    // State machine for holding and next target
    bool reached = (fabs(tgt_ticks - cur_pin15) <= 0.1f);
    unsigned long now = millis();
    
    if (reached) {
        if (now - lastSweepTime >= HOLD_TIME) {
            sweepState = (sweepState + 1) % 4;
            if (sweepState == 0) tgt_pin15 = 90.0f;       // go to 90
            else if (sweepState == 1) tgt_pin15 = 180.0f; // from 90 to 180
            else if (sweepState == 2) tgt_pin15 = 90.0f;  // from 180 to 90
            else if (sweepState == 3) tgt_pin15 = 0.0f;   // from 90 to 0
            lastSweepTime = now;
        }
    } else {
        // Reset hold timer while moving so we hold for exactly HOLD_TIME after reaching
        lastSweepTime = now;
    }
}

void loop() {
    // Read serial data
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            inputBuffer.trim();
            if (inputBuffer.length() > 0) {
                parseAndApply(inputBuffer);
            }
            inputBuffer = "";
        } else {
            inputBuffer += c;
        }
    }

    // Smoothly interpolate servos toward targets
    updateServos();
    
    // Update Pin 15 continuous sweep
    updatePin15Sweep();

    delay(10);
}
