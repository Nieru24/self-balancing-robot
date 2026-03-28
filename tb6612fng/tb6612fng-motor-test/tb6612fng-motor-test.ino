/**
 * @file    MotorTest.ino
 * @brief   Hardware checkout sketch for TB6612FNG_ESP32 library.
 *
 * Tests every motor-control primitive on both channels in sequence:
 *
 *   Step  Duration  What happens
 *   ────  ────────  ──────────────────────────────────────────────────────
 *    1     2 s      Motor A forward,  full speed (255)
 *    2     2 s      Motor A forward,  half speed (128)
 *    3     2 s      Motor A backward, full speed (255)
 *    4     2 s      Motor A backward, half speed (128)
 *    5     1 s      Motor A BRAKE  (active short-brake)
 *    6     1 s      Motor A COAST  (free-wheel)
 *    7     2 s      Motor B forward,  full speed (255)
 *    8     2 s      Motor B forward,  half speed (128)
 *    9     2 s      Motor B backward, full speed (255)
 *   10     2 s      Motor B backward, half speed (128)
 *   11     1 s      Motor B BRAKE
 *   12     1 s      Motor B COAST
 *   13     2 s      Both motors forward simultaneously (setSpeed signed API)
 *   14     2 s      Both motors backward simultaneously
 *   15     1 s      brakeAll()
 *   16     1 s      coastAll()
 *   17     —        STBY disable / re-enable demonstration
 *   18     —        Sequence repeats from Step 1
 *
 * After every state transition a Serial status line is printed so you can
 * follow along without an oscilloscope:
 *
 *   [MotorTest] Motor A → FORWARD  speed=255 | Motor B → COAST  speed=0
 *
 * ─────────────────────────────────────────────────────────────────────────
 * PIN ASSIGNMENT  (edit the #defines below to match your wiring)
 * ─────────────────────────────────────────────────────────────────────────
 *
 *  TB6612FNG pin  │ ESP32 GPIO  │ Notes
 *  ───────────────┼─────────────┼───────────────────────────────────────
 *  PWMA           │    18       │ Motor A PWM  — must be PWM-capable
 *  AIN1           │    16       │ Motor A direction 1
 *  AIN2           │    17       │ Motor A direction 2
 *  PWMB           │    19       │ Motor B PWM  — must be PWM-capable
 *  BIN1           │    21       │ Motor B direction 1
 *  BIN2           │    22       │ Motor B direction 2
 *  STBY           │     4       │ Standby control (active HIGH)
 *  VM             │    12 V     │ Motor supply (external)
 *  VCC            │   3.3 V     │ Logic supply (ESP32 3V3 pin)
 *  GND            │   GND       │ Common ground with ESP32
 *
 *  Encoder pins are declared here but the encoders are NOT actively read in
 *  this sketch.  begin() still initialises the ISRs so the counters run
 *  silently in the background — see examples/EncoderRead for encoder output.
 *
 *  Encoder A  │  A: GPIO 34  │  B: GPIO 35  │ Yellow / Green wires
 *  Encoder B  │  A: GPIO 32  │  B: GPIO 33  │ Yellow / Green wires
 *
 * ─────────────────────────────────────────────────────────────────────────
 * LEDC CHANNEL ASSIGNMENT
 * ─────────────────────────────────────────────────────────────────────────
 *  Motor A PWM → LEDC channel 0
 *  Motor B PWM → LEDC channel 1
 *  Channels 2–15 are free for other peripherals (e.g. servo, tone).
 *
 * ─────────────────────────────────────────────────────────────────────────
 * EXPECTED SERIAL OUTPUT  (115200 baud)
 * ─────────────────────────────────────────────────────────────────────────
 *  ╔══════════════════════════════════════════════════════╗
 *  ║  TB6612FNG_ESP32 — Motor Test                       ║
 *  ║  Library v1.0.0  |  ESP32 Arduino                   ║
 *  ╚══════════════════════════════════════════════════════╝
 *  STBY enabled.
 *  ── Motor A individual tests ──────────────────────────
 *  [A] FORWARD  speed=255 (full)
 *  [A] FORWARD  speed=128 (half)
 *  [A] BACKWARD speed=255 (full)
 *  [A] BACKWARD speed=128 (half)
 *  [A] BRAKE
 *  [A] COAST
 *  ── Motor B individual tests ──────────────────────────
 *  [B] FORWARD  speed=255 (full)
 *  ...
 *
 * @hardware  ESP32 DevKit (WROOM), TB6612FNG, 2× JGB37-520 178 RPM
 * @version   1.0.0
 * @date      2025
 */

#include <Arduino.h>
#include "TB6612FNG_ESP32.h"

// ============================================================
//  ── PIN DEFINITIONS ─────────────────────────────────────────
//  Edit these to match your physical wiring.
// ============================================================

// Motor A (TB6612FNG channel A)
#define PIN_PWMA      18   ///< PWMA  — PWM output for Motor A
#define PIN_AIN1      16   ///< AIN1  — direction control 1
#define PIN_AIN2      17   ///< AIN2  — direction control 2
#define LEDC_CH_A      0   ///< LEDC hardware channel for Motor A (unique)

// Motor B (TB6612FNG channel B)
#define PIN_PWMB      19   ///< PWMB  — PWM output for Motor B
#define PIN_BIN1      21   ///< BIN1  — direction control 1
#define PIN_BIN2      22   ///< BIN2  — direction control 2
#define LEDC_CH_B      1   ///< LEDC hardware channel for Motor B (unique)

// Shared driver pins
#define PIN_STBY       4   ///< STBY  — standby control (HIGH = active)

// Encoder A  (Yellow = C1/A, Green = C2/B)
#define PIN_ENC_A_A   34   ///< Encoder A, channel A (Yellow)
#define PIN_ENC_A_B   35   ///< Encoder A, channel B (Green)

// Encoder B  (Yellow = C1/A, Green = C2/B)
#define PIN_ENC_B_A   32   ///< Encoder B, channel A (Yellow)
#define PIN_ENC_B_B   33   ///< Encoder B, channel B (Green)

// ============================================================
//  ── TEST TIMING ─────────────────────────────────────────────
// ============================================================

/// Duration of each run/reverse phase (milliseconds)
static constexpr uint32_t RUN_MS   = 2000;

/// Duration of each brake/coast phase (milliseconds)
static constexpr uint32_t STOP_MS  = 1000;

/// Pause between major test sections (milliseconds)
static constexpr uint32_t GAP_MS   = 500;

/// Full-speed PWM duty cycle
static constexpr uint8_t SPEED_FULL = 255;

/// Half-speed PWM duty cycle
static constexpr uint8_t SPEED_HALF = 128;

/// Low-speed PWM duty cycle (demonstrates minimum usable torque)
static constexpr uint8_t SPEED_LOW  = 80;

// ============================================================
//  ── DRIVER INSTANCE ─────────────────────────────────────────
// ============================================================

TB6612FNG_ESP32 driver;

// ============================================================
//  ── HELPER UTILITIES ────────────────────────────────────────
// ============================================================

/**
 * @brief Print a section banner to Serial.
 * @param title  Text to display inside the banner.
 */
static void printBanner(const char* title)
{
    Serial.println();
    Serial.print(F("── "));
    Serial.print(title);
    Serial.println(F(" ──────────────────────────────────────"));
}

/**
 * @brief Print a single-line status for Motor A, then Motor B.
 *
 * Format:
 *   [A] FORWARD  speed=255  [B] COAST  speed=0
 */
static void printStatus()
{
    // Direction label lookup
    const char* const dirLabel[] = { "FORWARD ", "BACKWARD", "BRAKE   ", "COAST   " };

    Serial.print(F("  Status → [A] "));
    Serial.print(dirLabel[static_cast<uint8_t>(driver.motorA.getDirection())]);
    Serial.print(F(" spd="));
    Serial.print(driver.motorA.getSpeed());

    Serial.print(F("  |  [B] "));
    Serial.print(dirLabel[static_cast<uint8_t>(driver.motorB.getDirection())]);
    Serial.print(F(" spd="));
    Serial.println(driver.motorB.getSpeed());
}

/**
 * @brief Drive, print status, wait, then stop and wait.
 *
 * Abstracts the repetitive "run for RUN_MS → brake for STOP_MS" pattern
 * used throughout the test sequence.
 *
 * @param label    Short description printed to Serial before the run.
 * @param runMs    How long to run the motor(s) in milliseconds.
 * @param stopMs   How long to hold the brake after the run.
 */
static void runAndStop(const char* label, uint32_t runMs, uint32_t stopMs)
{
    Serial.print(F("  "));
    Serial.println(label);
    printStatus();
    delay(runMs);

    driver.brakeAll();
    Serial.println(F("  → brakeAll()"));
    printStatus();
    delay(stopMs);
}

// ============================================================
//  ── INDIVIDUAL MOTOR TESTS ──────────────────────────────────
// ============================================================

/**
 * @brief Run the full individual test sequence for one motor.
 *
 * Tests: forward (full), forward (half), forward (low),
 *        backward (full), backward (half), backward (low),
 *        brake, coast.
 *
 * The other motor is held in brake during this test so the robot
 * (if wheels are mounted) pivots in place rather than drifting.
 *
 * @param label  "A" or "B", used in Serial output only.
 * @param isA    true → test motorA; false → test motorB.
 */
static void testSingleMotor(const char* label, bool isA)
{
    Motor& mot = isA ? driver.motorA : driver.motorB;
    Motor& other = isA ? driver.motorB : driver.motorA;

    // Hold the untested motor in brake so the platform stays put.
    other.brake();

    Serial.print(F("── Motor "));
    Serial.print(label);
    Serial.println(F(" individual tests ─────────────────────"));

    // --- Forward: full speed ---
    Serial.print(F("  ["));  Serial.print(label);  Serial.println(F("] FORWARD  speed=255 (full)"));
    mot.forward(SPEED_FULL);
    printStatus();
    delay(RUN_MS);

    // --- Forward: half speed ---
    Serial.print(F("  ["));  Serial.print(label);  Serial.println(F("] FORWARD  speed=128 (half)"));
    mot.forward(SPEED_HALF);
    printStatus();
    delay(RUN_MS);

    // --- Forward: low speed — tests dead-band visibility ---
    Serial.print(F("  ["));  Serial.print(label);  Serial.println(F("] FORWARD  speed=80  (low)"));
    mot.forward(SPEED_LOW);
    printStatus();
    delay(RUN_MS);

    // --- Brake between forward and backward ---
    Serial.print(F("  ["));  Serial.print(label);  Serial.println(F("] BRAKE (direction change buffer)"));
    mot.brake();
    printStatus();
    delay(STOP_MS);

    // --- Backward: full speed ---
    Serial.print(F("  ["));  Serial.print(label);  Serial.println(F("] BACKWARD speed=255 (full)"));
    mot.backward(SPEED_FULL);
    printStatus();
    delay(RUN_MS);

    // --- Backward: half speed ---
    Serial.print(F("  ["));  Serial.print(label);  Serial.println(F("] BACKWARD speed=128 (half)"));
    mot.backward(SPEED_HALF);
    printStatus();
    delay(RUN_MS);

    // --- Backward: low speed ---
    Serial.print(F("  ["));  Serial.print(label);  Serial.println(F("] BACKWARD speed=80  (low)"));
    mot.backward(SPEED_LOW);
    printStatus();
    delay(RUN_MS);

    // --- Active brake ---
    Serial.print(F("  ["));  Serial.print(label);  Serial.println(F("] BRAKE (active short-brake)"));
    mot.brake();
    printStatus();
    delay(STOP_MS);

    // --- Coast ---
    Serial.print(F("  ["));  Serial.print(label);  Serial.println(F("] COAST (free-wheel)"));
    mot.coast();
    printStatus();
    delay(STOP_MS);

    // Leave this motor braked before handing off to the next test.
    mot.brake();
    delay(GAP_MS);
}

// ============================================================
//  ── SIGNED setSpeed() TEST ──────────────────────────────────
// ============================================================

/**
 * @brief Exercise Motor::setSpeed(int16_t) on both channels simultaneously.
 *
 * Demonstrates the signed API:
 *   +speed → forward, −speed → backward, 0 → brake.
 */
static void testSetSpeed()
{
    printBanner("setSpeed() signed API — both motors");

    // Positive → forward
    Serial.println(F("  setSpeed(+200) on both"));
    driver.motorA.setSpeed(+200);
    driver.motorB.setSpeed(+200);
    printStatus();
    delay(RUN_MS);

    // Negative → backward
    Serial.println(F("  setSpeed(-200) on both"));
    driver.motorA.setSpeed(-200);
    driver.motorB.setSpeed(-200);
    printStatus();
    delay(RUN_MS);

    // Asymmetric speeds (useful for turning on a differential drive robot)
    Serial.println(F("  setSpeed(+255 / -255) — spin in place"));
    driver.motorA.setSpeed(+255);
    driver.motorB.setSpeed(-255);
    printStatus();
    delay(RUN_MS);

    // Reverse spin direction
    Serial.println(F("  setSpeed(-255 / +255) — spin opposite"));
    driver.motorA.setSpeed(-255);
    driver.motorB.setSpeed(+255);
    printStatus();
    delay(RUN_MS);

    // Zero → brake
    Serial.println(F("  setSpeed(0) → brake on both"));
    driver.motorA.setSpeed(0);
    driver.motorB.setSpeed(0);
    printStatus();
    delay(STOP_MS);
}

// ============================================================
//  ── SIMULTANEOUS BOTH-MOTOR TEST ────────────────────────────
// ============================================================

/**
 * @brief Run both motors together with forward/backward/brake/coast
 *        using the high-level driver convenience methods.
 */
static void testBothMotors()
{
    printBanner("Both motors — simultaneous (direct API)");

    // Forward together
    Serial.println(F("  Both FORWARD full speed"));
    driver.motorA.forward(SPEED_FULL);
    driver.motorB.forward(SPEED_FULL);
    printStatus();
    delay(RUN_MS);

    // Half speed
    Serial.println(F("  Both FORWARD half speed"));
    driver.motorA.forward(SPEED_HALF);
    driver.motorB.forward(SPEED_HALF);
    printStatus();
    delay(RUN_MS);

    // brakeAll convenience method
    Serial.println(F("  brakeAll()"));
    driver.brakeAll();
    printStatus();
    delay(STOP_MS);

    // Backward together
    Serial.println(F("  Both BACKWARD full speed"));
    driver.motorA.backward(SPEED_FULL);
    driver.motorB.backward(SPEED_FULL);
    printStatus();
    delay(RUN_MS);

    // brakeAll
    Serial.println(F("  brakeAll()"));
    driver.brakeAll();
    printStatus();
    delay(STOP_MS);

    // coastAll convenience method
    Serial.println(F("  coastAll() — free wheel"));
    driver.coastAll();
    printStatus();
    delay(STOP_MS);
}

// ============================================================
//  ── PWM RAMP TEST ───────────────────────────────────────────
// ============================================================

/**
 * @brief Sweep PWM from 0 → 255 → 0 on both motors simultaneously.
 *
 * This test reveals the motor's dead-band (the minimum duty cycle at which
 * the motor shaft actually starts to turn under its own inertia) and
 * confirms that LEDC PWM is working correctly across the full 8-bit range.
 *
 * Watch the Serial plotter or listen for the point where shaft rotation
 * begins and ends — that threshold is the practical PWM_MIN for your
 * specific motor + load combination.
 */
static void testPWMRamp()
{
    printBanner("PWM ramp sweep — both motors FORWARD");

    Serial.println(F("  Ramping UP  0 → 255  (5 ms / step)"));
    driver.motorA.forward(0);
    driver.motorB.forward(0);

    for (int16_t spd = 0; spd <= 255; spd++) {
        driver.motorA.forward(static_cast<uint8_t>(spd));
        driver.motorB.forward(static_cast<uint8_t>(spd));
        delay(5);   // 5 ms × 256 steps = ~1.3 s total sweep
    }

    Serial.println(F("  Ramping DOWN  255 → 0  (5 ms / step)"));
    for (int16_t spd = 255; spd >= 0; spd--) {
        driver.motorA.forward(static_cast<uint8_t>(spd));
        driver.motorB.forward(static_cast<uint8_t>(spd));
        delay(5);
    }

    driver.brakeAll();
    Serial.println(F("  Ramp complete → brakeAll()"));
    delay(STOP_MS);
}

// ============================================================
//  ── STBY DISABLE / RE-ENABLE TEST ───────────────────────────
// ============================================================

/**
 * @brief Demonstrate that STBY LOW overrides all motor commands.
 *
 * Even with IN1/IN2 set to forward and PWM = 255, the TB6612FNG outputs
 * are disabled while STBY is LOW.  This test confirms the hardware
 * standby interlock is functional.
 */
static void testStandby()
{
    printBanner("STBY standby interlock test");

    // Command motors forward at full speed…
    driver.motorA.forward(SPEED_FULL);
    driver.motorB.forward(SPEED_FULL);
    Serial.println(F("  Motors commanded FORWARD speed=255"));
    printStatus();
    delay(GAP_MS);

    // …then disable STBY.  Motors should stop immediately even though the
    // software state still reflects FORWARD/255.
    Serial.println(F("  driver.disable() — pulling STBY LOW"));
    Serial.println(F("  *** Motors should coast to stop NOW ***"));
    driver.disable();
    printStatus();   // Will still show COAST/0 because disable() calls coastAll()
    delay(RUN_MS);

    // Re-enable — motors should resume the commanded state.
    Serial.println(F("  driver.enable() — pulling STBY HIGH"));
    driver.enable();

    // Re-apply forward command (disable() coasted the motors).
    driver.motorA.forward(SPEED_FULL);
    driver.motorB.forward(SPEED_FULL);
    Serial.println(F("  Motors re-commanded FORWARD speed=255"));
    printStatus();
    delay(RUN_MS);

    driver.brakeAll();
    Serial.println(F("  brakeAll() — end of STBY test"));
    delay(STOP_MS);
}

// ============================================================
//  ── setup() ─────────────────────────────────────────────────
// ============================================================

void setup()
{
    Serial.begin(115200);
    // Brief pause so the terminal has time to connect before output begins.
    delay(500);

    // Header banner
    Serial.println();
    Serial.println(F("╔══════════════════════════════════════════════════════╗"));
    Serial.println(F("║  TB6612FNG_ESP32 — Motor Test                       ║"));
    Serial.println(F("║  Library v1.0.0  |  ESP32 Arduino                   ║"));
    Serial.println(F("║                                                      ║"));
    Serial.println(F("║  Motors will spin!  Keep fingers clear of wheels.   ║"));
    Serial.println(F("╚══════════════════════════════════════════════════════╝"));
    Serial.println();

    // ── Pin-config structs ──────────────────────────────────────
    MotorPinConfig motorCfgA = {
        .pwmPin   = PIN_PWMA,
        .in1Pin   = PIN_AIN1,
        .in2Pin   = PIN_AIN2,
        .ledcChan = LEDC_CH_A
    };

    MotorPinConfig motorCfgB = {
        .pwmPin   = PIN_PWMB,
        .in1Pin   = PIN_BIN1,
        .in2Pin   = PIN_BIN2,
        .ledcChan = LEDC_CH_B
    };

    EncoderPinConfig encoderCfgA = {
        .pinA = PIN_ENC_A_A,
        .pinB = PIN_ENC_A_B
    };

    EncoderPinConfig encoderCfgB = {
        .pinA = PIN_ENC_B_A,
        .pinB = PIN_ENC_B_B
    };

    // ── Initialise the driver ───────────────────────────────────
    driver.begin(motorCfgA, motorCfgB, encoderCfgA, encoderCfgB, PIN_STBY);

    Serial.print(F("LEDC PWM   → freq="));
    Serial.print(LEDC_FREQ_HZ / 1000);
    Serial.print(F(" kHz  resolution="));
    Serial.print(LEDC_RESOLUTION);
    Serial.println(F("-bit"));

    Serial.print(F("Motor A    → PWMA:GPIO"));  Serial.print(PIN_PWMA);
    Serial.print(F("  AIN1:GPIO"));             Serial.print(PIN_AIN1);
    Serial.print(F("  AIN2:GPIO"));             Serial.print(PIN_AIN2);
    Serial.print(F("  LEDC-ch:"));              Serial.println(LEDC_CH_A);

    Serial.print(F("Motor B    → PWMB:GPIO"));  Serial.print(PIN_PWMB);
    Serial.print(F("  BIN1:GPIO"));             Serial.print(PIN_BIN1);
    Serial.print(F("  BIN2:GPIO"));             Serial.print(PIN_BIN2);
    Serial.print(F("  LEDC-ch:"));              Serial.println(LEDC_CH_B);

    Serial.print(F("STBY       → GPIO"));       Serial.println(PIN_STBY);

    Serial.print(F("Encoder A  → A:GPIO"));     Serial.print(PIN_ENC_A_A);
    Serial.print(F("  B:GPIO"));                Serial.println(PIN_ENC_A_B);

    Serial.print(F("Encoder B  → A:GPIO"));     Serial.print(PIN_ENC_B_A);
    Serial.print(F("  B:GPIO"));                Serial.println(PIN_ENC_B_B);

    Serial.println();

    // ── Safety countdown ────────────────────────────────────────
    // Gives the operator time to secure the robot / clear the work area
    // before motors start spinning.
    for (int i = 3; i > 0; i--) {
        Serial.print(F("Starting in "));
        Serial.print(i);
        Serial.println(F(" ..."));
        delay(1000);
    }

    // ── Enable the driver (STBY HIGH) ───────────────────────────
    driver.enable();
    Serial.println(F("STBY → HIGH (driver enabled)"));
    Serial.println();
}

// ============================================================
//  ── loop() ──────────────────────────────────────────────────
// ============================================================

void loop()
{
    // ── Section 1: Motor A individual tests ─────────────────────
    testSingleMotor("A", true);
    delay(GAP_MS);

    // ── Section 2: Motor B individual tests ─────────────────────
    testSingleMotor("B", false);
    delay(GAP_MS);

    // ── Section 3: Both motors simultaneously ───────────────────
    testBothMotors();
    delay(GAP_MS);

    // ── Section 4: Signed setSpeed() API ────────────────────────
    testSetSpeed();
    delay(GAP_MS);

    // ── Section 5: PWM ramp sweep ───────────────────────────────
    testPWMRamp();
    delay(GAP_MS);

    // ── Section 6: STBY interlock ───────────────────────────────
    testStandby();
    delay(GAP_MS);

    // ── End of one full pass — announce repeat ───────────────────
    Serial.println();
    Serial.println(F("════════════════════════════════════════════════════════"));
    Serial.println(F("  Full test pass complete.  Repeating in 3 s …"));
    Serial.println(F("════════════════════════════════════════════════════════"));
    delay(3000);
}
