/**
 * @file main.ino
 * @brief Two-wheeled self-balancing robot with integrated calibration support.
 *
 * This sketch combines:
 *   - MPU6050 IMU calibration (from calibration sketch)
 *   - TB6612FNG encoder calibration (from calibration sketch)
 *   - PID balancing control loop
 *
 * USAGE:
 *   1. Run MPU6050 calibration sketch → copy offsets
 *   2. Run TB6612FNG encoder calibration sketch → verify CPR and direction
 *   3. Paste both calibration results in the sections below
 *   4. Upload this sketch and tune PID gains
 *
 * Hardware:
 *   - ESP32
 *   - MPU6050 (I2C: SDA=21, SCL=22)
 *   - TB6612FNG (Motor driver)
 *   - JGB37-520 motors with Hall encoders
 *     - Red:    Motor VCC (12V)
 *     - White:  Motor GND
 *     - Blue:   Encoder VCC (3.3V)
 *     - Black:  Encoder GND
 *     - Green:  Encoder Channel B (C2/B)
 *     - Yellow: Encoder Channel A (C1/A) - 11 PPR
 *
 * @author Embedded Systems Engineer
 * @version 3.0.0
 * @date 2026-03-29
 */

// ============================================================================
//  Library includes
// ============================================================================

#include "TB6612FNG_ESP32.h"
#include "MPU6050_ESP32.h"

// ============================================================================
//  CALIBRATION DATA
//  ===========================================================================
//  SECTION 1: MPU6050 IMU Calibration
//  Run the MPU6050 calibration sketch and paste results here
//  ============================================================================

// TODO: REPLACE WITH YOUR MPU6050 CALIBRATION RESULTS
const CalibrationOffsets MPU6050_CALIBRATION = {
    .accelX = 0,      // Replace with your accel X offset
    .accelY = 0,      // Replace with your accel Y offset
    .accelZ = 0,      // Replace with your accel Z offset (gravity already removed)
    .gyroX  = 0,      // Replace with your gyro X offset
    .gyroY  = 0,      // Replace with your gyro Y offset
    .gyroZ  = 0       // Replace with your gyro Z offset
};

// ============================================================================
//  SECTION 2: Motor Configuration (from TB6612FNG calibration)
//  ============================================================================
// These values should be verified by running the TB6612FNG calibration sketch
// The calibration sketch will confirm:
//   - Encoder CPR matches 2464 (±2%)
//   - Direction mapping is correct
//   - Motor performance meets expectations

// Motor direction correction (if calibration showed reversed direction)
// Set to true if motor runs backward when forward command is given
constexpr bool REVERSE_MOTOR_A = false;   // Set to true if Motor A direction is reversed
constexpr bool REVERSE_MOTOR_B = false;   // Set to true if Motor B direction is reversed

// Maximum achievable RPM at full speed (from calibration sweep)
// Used for scaling and diagnostics
constexpr float MOTOR_A_MAX_RPM = 174.0f;  // Replace with your measured max RPM
constexpr float MOTOR_B_MAX_RPM = 172.0f;  // Replace with your measured max RPM

// ============================================================================
//  Pin definitions
// ============================================================================

// ----- TB6612FNG Motor A (Right motor) -----
constexpr uint8_t PIN_PWM_A    = 18;   // PWMA (PWM output)
constexpr uint8_t PIN_IN1_A    = 16;   // AIN1 (Direction)
constexpr uint8_t PIN_IN2_A    = 17;   // AIN2 (Direction)

// ----- TB6612FNG Motor B (Left motor) -----
constexpr uint8_t PIN_PWM_B    = 19;   // PWMB (PWM output)
constexpr uint8_t PIN_IN1_B    = 21;   // BIN1 (Direction)
constexpr uint8_t PIN_IN2_B    = 22;   // BIN2 (Direction)

// ----- JGB37-520 Encoder connections -----
constexpr uint8_t PIN_ENC_A_A  = 34;   // Motor A Encoder Channel A (Yellow wire - 11 PPR)
constexpr uint8_t PIN_ENC_A_B  = 35;   // Motor A Encoder Channel B (Green wire)
constexpr uint8_t PIN_ENC_B_A  = 32;   // Motor B Encoder Channel A (Yellow wire - 11 PPR)
constexpr uint8_t PIN_ENC_B_B  = 33;   // Motor B Encoder Channel B (Green wire)

// ----- Shared control -----
constexpr uint8_t PIN_STBY     = 4;    // Standby pin (active HIGH)

// ----- MPU6050 I2C -----
constexpr uint8_t PIN_I2C_SDA  = 21;   // I2C SDA
constexpr uint8_t PIN_I2C_SCL  = 22;   // I2C SCL

// ----- Status LED -----
constexpr uint8_t PIN_LED_BUILTIN = 2;   // Built-in LED on ESP32 dev boards

// ============================================================================
//  Control parameters
// ============================================================================

// ----- PID tuning constants (adjust these for your robot) -----
// Start with P-only, then add D to dampen oscillations, finally I to eliminate drift
constexpr float PID_KP = 25.0f;   // Proportional gain (core balancing)
constexpr float PID_KI = 0.5f;    // Integral gain (eliminate steady-state error)
constexpr float PID_KD = 1.2f;    // Derivative gain (dampen oscillations)

// ----- Control loop timing -----
constexpr uint32_t CONTROL_LOOP_INTERVAL_US = 10000; // 10,000 µs = 100 Hz

// ----- Safety limits -----
constexpr float MAX_TILT_ANGLE_DEG = 45.0f;   // Disable motors if tilt exceeds ±45°
constexpr int16_t MAX_MOTOR_SPEED = 200;       // Limit max PWM (0-255)

// ----- Balancing setpoint -----
constexpr float BALANCE_SETPOINT_DEG = 0.0f;   // Target angle (0° = vertical)

// ----- Filter parameters -----
constexpr float COMP_FILTER_ALPHA = 0.98f;     // Complementary filter trust factor

// ----- Debug options -----
constexpr bool ENABLE_DEBUG_OUTPUT = true;      // Enable/disable serial debug output
constexpr uint32_t DEBUG_INTERVAL_MS = 100;     // 10 Hz debug output

// ============================================================================
//  Global objects
// ============================================================================

MPU6050_ESP32 imu;
TB6612FNG_ESP32 driver;

// PID controller state
struct PIDController {
    float kp, ki, kd;
    float integral;
    float lastError;
    uint32_t lastUpdateUs;
};

static PIDController balancer = {
    .kp = PID_KP,
    .ki = PID_KI,
    .kd = PID_KD,
    .integral = 0.0f,
    .lastError = 0.0f,
    .lastUpdateUs = 0
};

// ============================================================================
//  PID calculation function
// ============================================================================

float updatePID(PIDController& pid, float setpoint, float measurement, float dtSeconds) {
    float error = setpoint - measurement;
    
    // Integral with anti-windup
    pid.integral += error * dtSeconds;
    pid.integral = constrain(pid.integral, -50.0f, 50.0f);
    
    // Derivative
    float derivative = (error - pid.lastError) / dtSeconds;
    
    // Output
    float output = (pid.kp * error) + (pid.ki * pid.integral) + (pid.kd * derivative);
    
    pid.lastError = error;
    return output;
}

// ============================================================================
//  Motor control with calibration corrections
// ============================================================================

void setMotorSpeeds(int16_t speed) {
    speed = constrain(speed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    
    // Apply direction correction if needed
    int16_t speedA = REVERSE_MOTOR_A ? -speed : speed;
    int16_t speedB = REVERSE_MOTOR_B ? -speed : speed;
    
    driver.motorA.setSpeed(speedA);
    driver.motorB.setSpeed(speedB);
}

// ============================================================================
//  Calibration verification
// ============================================================================

bool verifyCalibration() {
    bool allGood = true;
    
    // Verify MPU6050 calibration
    CalibrationOffsets loadedOffsets;
    imu.getCalibrationOffsets(loadedOffsets);
    
    Serial.println("\n--- Calibration Verification ---");
    
    // Check if MPU6050 offsets were loaded
    if (loadedOffsets.accelX == 0 && loadedOffsets.accelY == 0 && 
        loadedOffsets.accelZ == 0 && loadedOffsets.gyroX == 0 && 
        loadedOffsets.gyroY == 0 && loadedOffsets.gyroZ == 0) {
        Serial.println("⚠ WARNING: MPU6050 calibration offsets are ZERO!");
        Serial.println("  Please run the MPU6050 calibration sketch first.");
        allGood = false;
    } else {
        Serial.println("✓ MPU6050 calibration loaded:");
        Serial.printf("  Accel: X=%d, Y=%d, Z=%d\n", 
                     loadedOffsets.accelX, loadedOffsets.accelY, loadedOffsets.accelZ);
        Serial.printf("  Gyro:  X=%d, Y=%d, Z=%d\n", 
                     loadedOffsets.gyroX, loadedOffsets.gyroY, loadedOffsets.gyroZ);
    }
    
    // Test encoder readings
    Serial.println("\nTesting encoder readings...");
    Serial.println("Manually rotate the wheels and watch for tick counts:");
    Serial.println("  Forward rotation should show POSITIVE ticks");
    Serial.println("  Backward rotation should show NEGATIVE ticks");
    Serial.println("Testing for 3 seconds...\n");
    
    driver.encoderA.resetTicks();
    driver.encoderB.resetTicks();
    
    uint32_t startTest = millis();
    while (millis() - startTest < 3000) {
        static uint32_t lastPrint = 0;
        if (millis() - lastPrint >= 500) {
            lastPrint = millis();
            int32_t ticksA = driver.encoderA.getTicks();
            int32_t ticksB = driver.encoderB.getTicks();
            Serial.printf("  Motor A: %+6d ticks | Motor B: %+6d ticks\n", ticksA, ticksB);
        }
        delay(10);
    }
    
    Serial.println("\nIf ticks don't change when rotating wheels, check encoder wiring.");
    Serial.println("If direction is reversed, update REVERSE_MOTOR_A/B constants.\n");
    
    return allGood;
}

// ============================================================================
//  Setup function
// ============================================================================

void setup() {
    // Initialize LED
    pinMode(PIN_LED_BUILTIN, OUTPUT);
    digitalWrite(PIN_LED_BUILTIN, LOW);
    
    // Initialize serial
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n╔══════════════════════════════════════════════════════════════════╗");
    Serial.println("║     Self-Balancing Robot with Integrated Calibration          ║");
    Serial.println("╚══════════════════════════════════════════════════════════════════╝");
    Serial.println("\nJGB37-520 Motor Configuration:");
    Serial.println("  - Encoder: 11 PPR × 56 gear × 4 decoding = 2464 counts/rev");
    Serial.printf("  - Motor A max RPM: %.1f (from calibration)\n", MOTOR_A_MAX_RPM);
    Serial.printf("  - Motor B max RPM: %.1f (from calibration)\n", MOTOR_B_MAX_RPM);
    Serial.println("========================================\n");
    
    // --- Initialize motor driver ---
    Serial.println("Initializing motor driver...");
    
    MotorPinConfig motorCfgA = {
        .pwmPin   = PIN_PWM_A,
        .in1Pin   = PIN_IN1_A,
        .in2Pin   = PIN_IN2_A,
        .ledcChan = 0
    };
    
    MotorPinConfig motorCfgB = {
        .pwmPin   = PIN_PWM_B,
        .in1Pin   = PIN_IN1_B,
        .in2Pin   = PIN_IN2_B,
        .ledcChan = 1
    };
    
    EncoderPinConfig encCfgA = { .pinA = PIN_ENC_A_A, .pinB = PIN_ENC_A_B };
    EncoderPinConfig encCfgB = { .pinA = PIN_ENC_B_A, .pinB = PIN_ENC_B_B };
    
    driver.begin(motorCfgA, motorCfgB, encCfgA, encCfgB, PIN_STBY);
    driver.enable();
    driver.coastAll();
    
    // --- Initialize MPU6050 ---
    Serial.println("Initializing MPU6050...");
    
    if (!imu.begin(PIN_I2C_SDA, PIN_I2C_SCL)) {
        Serial.println("ERROR: MPU6050 not found! Check wiring.");
        while (true) {
            digitalWrite(PIN_LED_BUILTIN, HIGH);
            delay(200);
            digitalWrite(PIN_LED_BUILTIN, LOW);
            delay(200);
        }
    }
    
    // Configure IMU (must match calibration settings)
    imu.setAccelFSR(AccelFSR::FSR_2G);
    imu.setGyroFSR(GyroFSR::FSR_250DPS);
    imu.setDLPF(DLPFMode::DLPF_3);
    imu.setSampleRateDivider(9);
    
    // Load calibration
    imu.setCalibrationOffsets(MPU6050_CALIBRATION);
    
    // Verify all calibrations
    if (!verifyCalibration()) {
        Serial.println("\n⚠ CALIBRATION WARNING: Some calibrations missing!");
        Serial.println("The robot may not balance correctly.");
        Serial.println("Press any key to continue, or reset to abort...");
        delay(5000);
    }
    
    // Reset filter
    imu.resetFilter();
    
    // Test initial pitch
    Angles testAngles;
    if (imu.getAngles(testAngles, COMP_FILTER_ALPHA)) {
        Serial.printf("\nInitial pitch angle: %.2f°\n", testAngles.pitch);
        if (fabs(testAngles.pitch) > 10.0f) {
            Serial.println("⚠ WARNING: Robot not level! Pitch angle > 10°.");
        } else if (fabs(testAngles.pitch) > 2.0f) {
            Serial.println("✓ Robot is nearly level.");
        } else {
            Serial.println("✓ Robot is perfectly level.");
        }
    }
    
    // Countdown
    Serial.println("\n========================================");
    Serial.println("READY! Robot will start balancing in 3 seconds...");
    Serial.println("========================================");
    
    for (int i = 3; i > 0; i--) {
        Serial.print(i);
        Serial.println("...");
        digitalWrite(PIN_LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(PIN_LED_BUILTIN, LOW);
        delay(500);
    }
    
    Serial.println("ACTIVE! Robot is now balancing.\n");
    digitalWrite(PIN_LED_BUILTIN, HIGH);
    
    // Initialize PID timing
    balancer.lastUpdateUs = micros();
}

// ============================================================================
//  Main loop
// ============================================================================

void loop() {
    static uint32_t lastControlUs = 0;
    static uint32_t lastDebugPrint = 0;
    uint32_t nowUs = micros();
    
    // Fixed-frequency control loop
    if ((nowUs - lastControlUs) >= CONTROL_LOOP_INTERVAL_US) {
        lastControlUs = nowUs;
        
        // Calculate dt
        static uint32_t lastLoopUs = 0;
        float dtSeconds = (nowUs - lastLoopUs) * 1e-6f;
        lastLoopUs = nowUs;
        
        if (dtSeconds > 0.05f) dtSeconds = 0.05f;
        if (dtSeconds < 0.005f) dtSeconds = 0.01f;
        
        // Get pitch angle
        Angles angles;
        if (!imu.getAngles(angles, COMP_FILTER_ALPHA)) {
            driver.disable();
            digitalWrite(PIN_LED_BUILTIN, LOW);
            Serial.println("ERROR: IMU read failed!");
            return;
        }
        
        float pitchAngle = angles.pitch;
        
        // Safety check
        if (fabsf(pitchAngle) > MAX_TILT_ANGLE_DEG) {
            driver.disable();
            digitalWrite(PIN_LED_BUILTIN, LOW);
            Serial.printf("\n⚠ SAFETY: Excessive tilt (%.1f°)! Motors disabled.\n", pitchAngle);
            while (true) {
                digitalWrite(PIN_LED_BUILTIN, HIGH);
                delay(100);
                digitalWrite(PIN_LED_BUILTIN, LOW);
                delay(100);
                Serial.print(".");
            }
        }
        
        // PID calculation
        float pidOutput = updatePID(balancer, BALANCE_SETPOINT_DEG, pitchAngle, dtSeconds);
        int16_t motorSpeed = constrain((int16_t)pidOutput, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
        
        // Apply motor command
        setMotorSpeeds(motorSpeed);
        
        // Debug output
        if (ENABLE_DEBUG_OUTPUT && (millis() - lastDebugPrint) >= DEBUG_INTERVAL_MS) {
            lastDebugPrint = millis();
            Serial.printf("Pitch: %+6.2f° | PID: %+7.1f | Speed: %+4d | Integral: %+6.1f\n",
                          pitchAngle, pidOutput, motorSpeed, balancer.integral);
        }
    }
    
    delay(0);
}