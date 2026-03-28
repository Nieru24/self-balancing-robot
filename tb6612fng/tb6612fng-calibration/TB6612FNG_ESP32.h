/**
 * @file TB6612FNG_ESP32.h
 * @brief ESP32 Arduino library for TB6612FNG dual motor driver with
 *        JGB37-520 Hall encoder motors and optional PID speed control.
 *
 * Hardware targets:
 *  - MCU:          ESP32 (WROOM / DevKit)
 *  - Motor driver: TB6612FNG dual H-bridge
 *  - Motors:       JGB37-520, 178 RPM, 56:1 gearbox, 12 V
 *  - Encoders:     AB quadrature Hall encoder, 11 PPR (motor shaft)
 *
 * Encoder math (constexpr, computed at compile time):
 *  - PULSES_PER_MOTOR_REV  = 11
 *  - GEAR_RATIO            = 56
 *  - PULSES_PER_OUTPUT_REV = 616   (11 × 56)
 *  - COUNTS_PER_OUTPUT_REV = 2464  (616 × 4, quadrature ×4 decoding)
 *
 * PWM:
 *  - Uses ESP32 LEDC peripheral (ledcSetup / ledcAttachPin / ledcWrite)
 *  - Frequency : 20 kHz (above audible range)
 *  - Resolution: 8-bit (0–255 duty cycle)
 *
 * @version  1.0.0
 * @date     2025
 * @license  MIT
 */

#pragma once
#ifndef TB6612FNG_ESP32_H
#define TB6612FNG_ESP32_H

#include <Arduino.h>
#include <stdint.h>

// ============================================================
//  Encoder math constants  (constexpr — evaluated at compile time)
// ============================================================

/** Number of encoder pulses per single motor-shaft revolution (before gearbox). */
constexpr uint8_t  PULSES_PER_MOTOR_REV  = 11;

/** Gearbox reduction ratio of the JGB37-520. */
constexpr uint8_t  GEAR_RATIO            = 56;

/**
 * Encoder pulses per OUTPUT shaft revolution (after gearbox).
 * = PULSES_PER_MOTOR_REV × GEAR_RATIO = 11 × 56 = 616
 */
constexpr uint16_t PULSES_PER_OUTPUT_REV = PULSES_PER_MOTOR_REV * GEAR_RATIO; // 616

/**
 * Encoder COUNTS per output-shaft revolution with quadrature ×4 decoding
 * (both edges of both channels).
 * = PULSES_PER_OUTPUT_REV × 4 = 2464
 */
constexpr uint16_t COUNTS_PER_OUTPUT_REV = PULSES_PER_OUTPUT_REV * 4;         // 2464

// ============================================================
//  LEDC (PWM) configuration
// ============================================================

/** PWM carrier frequency (Hz). 20 kHz keeps switching noise above audible range. */
constexpr uint32_t LEDC_FREQ_HZ    = 20000;

/** PWM resolution in bits. 8-bit gives 256 discrete duty-cycle steps (0–255). */
constexpr uint8_t  LEDC_RESOLUTION = 8;

/** Maximum PWM duty cycle value at 8-bit resolution. */
constexpr uint8_t  PWM_MAX         = 255;

/** Minimum meaningful PWM duty cycle (motor dead-band threshold). */
constexpr uint8_t  PWM_MIN         = 0;

// ============================================================
//  PID constants — sensible defaults, fully tuneable
// ============================================================

/** Default proportional gain for PID speed controller. */
constexpr float PID_KP_DEFAULT = 1.5f;

/** Default integral gain for PID speed controller. */
constexpr float PID_KI_DEFAULT = 0.8f;

/** Default derivative gain for PID speed controller. */
constexpr float PID_KD_DEFAULT = 0.05f;

/** Anti-windup clamp: maximum absolute value of the integral accumulator. */
constexpr float PID_INTEGRAL_LIMIT = 200.0f;

// ============================================================
//  Enumerations
// ============================================================

/**
 * @brief Motor direction / braking states.
 *
 * Matches TB6612FNG truth table:
 * | State    | IN1 | IN2 | PWM |
 * |----------|-----|-----|-----|
 * | FORWARD  |  H  |  L  | PWM |
 * | BACKWARD |  L  |  H  | PWM |
 * | BRAKE    |  H  |  H  | H   |
 * | COAST    |  L  |  L  | L   |
 */
enum class MotorDirection : uint8_t {
    FORWARD  = 0, ///< IN1=HIGH, IN2=LOW,  PWM active
    BACKWARD = 1, ///< IN1=LOW,  IN2=HIGH, PWM active
    BRAKE    = 2, ///< IN1=HIGH, IN2=HIGH, active short-brake
    COAST    = 3  ///< IN1=LOW,  IN2=LOW,  free-wheel
};

/**
 * @brief Logical motor identifier.
 *
 * Used when a single TB6612FNG_ESP32 object manages both channels.
 */
enum class MotorChannel : uint8_t {
    MOTOR_A = 0, ///< Channel A (PWMA, AIN1, AIN2)
    MOTOR_B = 1  ///< Channel B (PWMB, BIN1, BIN2)
};

// ============================================================
//  Pin-configuration structures
// ============================================================

/**
 * @brief All ESP32 GPIO and LEDC-channel assignments for one TB6612FNG output channel.
 *
 * Populate one struct per motor and pass them to the Motor constructor.
 *
 * @note  Each PWM pin must use a UNIQUE LEDC channel (0–15) across the
 *        entire application.  Motor A and Motor B must not share a channel.
 *
 * Example — Motor A:
 * @code
 *   MotorPinConfig cfgA = {
 *       .pwmPin   = 18,   // GPIO connected to PWMA
 *       .in1Pin   = 16,   // GPIO connected to AIN1
 *       .in2Pin   = 17,   // GPIO connected to AIN2
 *       .ledcChan = 0     // LEDC channel 0 (must be unique)
 *   };
 * @endcode
 */
struct MotorPinConfig {
    uint8_t pwmPin;    ///< GPIO → PWMx pin on TB6612FNG
    uint8_t in1Pin;    ///< GPIO → IN1 pin  (AIN1 or BIN1)
    uint8_t in2Pin;    ///< GPIO → IN2 pin  (AIN2 or BIN2)
    uint8_t ledcChan;  ///< LEDC hardware channel (0–15, unique per PWM pin)
};

/**
 * @brief GPIO assignments for one AB quadrature encoder.
 *
 * Both pins must support external interrupts on the ESP32 (all GPIOs except
 * input-only strapping pins do).
 *
 * Example — Motor A encoder:
 * @code
 *   EncoderPinConfig encA = {
 *       .pinA = 34,  // Yellow wire — Channel A (C1)
 *       .pinB = 35   // Green  wire — Channel B (C2)
 *   };
 * @endcode
 */
struct EncoderPinConfig {
    uint8_t pinA; ///< Encoder channel A (Yellow — C1)
    uint8_t pinB; ///< Encoder channel B (Green  — C2)
};

// ============================================================
//  Forward declarations (for ISR friend access)
// ============================================================
class Motor;
class MotorEncoder;

// ============================================================
//  MotorEncoder — quadrature encoder, interrupt-driven
// ============================================================

/**
 * @brief Quadrature (AB) encoder driver for one JGB37-520 motor channel.
 *
 * Attaches hardware interrupts to both encoder channels and performs ×4
 * quadrature decoding, maintaining a signed 32-bit tick counter.
 *
 * Interrupt service routines are defined as file-scope friend functions in
 * the .cpp so that each encoder instance has its own dedicated ISR without
 * relying on dynamic dispatch or lambda captures (which are ISR-unsafe).
 *
 * @note  Because ISRs must be static/global, this class supports at most
 *        two simultaneous instances (one per TB6612FNG channel).  The
 *        instances are stored in the static pointer array s_instances[].
 */
class MotorEncoder {
public:
    /**
     * @brief Construct an uninitialised encoder object.
     *
     * Call begin() before using any other method.
     */
    MotorEncoder();

    /**
     * @brief Initialise GPIO pins and attach interrupts.
     *
     * @param cfg     Encoder pin configuration (A and B channels).
     * @param index   Instance index (0 = Motor A, 1 = Motor B).
     *                Must match the index used internally by the ISR dispatch.
     */
    void begin(const EncoderPinConfig& cfg, uint8_t index);

    /**
     * @brief Return the current accumulated tick count.
     *
     * The value is read atomically (interrupts briefly disabled).
     *
     * @return Signed 32-bit tick count.  Increments in FORWARD direction,
     *         decrements in BACKWARD direction.
     */
    int32_t getTicks() const;

    /**
     * @brief Zero the tick counter.
     *
     * Thread-safe: disables interrupts for the duration of the reset.
     */
    void resetTicks();

    /**
     * @brief Compute the current output-shaft speed in RPM.
     *
     * Uses a delta-tick / delta-time method.  Must be called periodically
     * (recommended ≥ every 100 ms) for reasonable accuracy.
     *
     * Formula:
     * @code
     *   RPM = (deltaTicks / COUNTS_PER_OUTPUT_REV) / (deltaTimeMs / 60000.0f)
     * @endcode
     *
     * @return Output shaft speed in RPM (float, signed — negative = reverse).
     */
    float getRPM();

    /**
     * @brief Return the cumulative output-shaft angle in degrees.
     *
     * @return Angle in degrees (float, unbounded — can exceed ±360°).
     */
    float getAngleDeg() const;

    /**
     * @brief Compute linear distance travelled given a wheel diameter.
     *
     * Uses the cumulative tick count since the last resetTicks() call.
     *
     * @param wheelDiameterMM  Wheel outer diameter in millimetres.
     * @return Distance in millimetres (float, signed).
     */
    float getDistanceMM(float wheelDiameterMM) const;

    // --- ISR-accessible interface (do not call from user code) ---

    /**
     * @brief Called by the ISR when channel A changes state.
     * @note  IRAM_ATTR applied in the .cpp implementation.
     */
    void IRAM_ATTR handleChannelA();

    /**
     * @brief Called by the ISR when channel B changes state.
     * @note  IRAM_ATTR applied in the .cpp implementation.
     */
    void IRAM_ATTR handleChannelB();

    // Static instance registry — one slot per motor channel (max 2)
    static MotorEncoder* s_instances[2]; ///< @private

private:
    EncoderPinConfig _cfg;           ///< Pin assignments

    volatile int32_t _ticks;         ///< Quadrature tick counter (ISR-written)
    volatile uint8_t _lastEncoded;   ///< Previous 2-bit encoder state (A<<1 | B)

    // RPM computation state
    int32_t  _lastTicksRPM;          ///< Tick snapshot at last getRPM() call
    uint32_t _lastTimeRPM;           ///< millis() snapshot at last getRPM() call

    uint8_t  _index;                 ///< Instance index (0 or 1)
    bool     _initialised;           ///< True after begin() is called
};

// ============================================================
//  Motor — single H-bridge channel with PWM and direction control
// ============================================================

/**
 * @brief Single-channel motor driver interface for one TB6612FNG output channel.
 *
 * Controls motor direction via IN1/IN2 digital outputs and speed via the
 * LEDC PWM peripheral.  The STBY pin is shared between both channels and
 * is managed by the TB6612FNG_ESP32 wrapper class, but enable() / disable()
 * helpers are provided here for standalone use.
 *
 * LEDC channel assignment:
 *  - Each Motor instance must be assigned a unique LEDC channel (0–15).
 *  - Pass the desired channel number in MotorPinConfig::ledcChan.
 */
class Motor {
public:
    /**
     * @brief Construct an uninitialised Motor object.
     *
     * Call begin() before using any other method.
     */
    Motor();

    /**
     * @brief Initialise GPIO pins and configure the LEDC PWM channel.
     *
     * Sets IN1 and IN2 as OUTPUT, calls ledcSetup() and ledcAttachPin()
     * with LEDC_FREQ_HZ and LEDC_RESOLUTION, then puts the motor in COAST
     * state.
     *
     * @param cfg  Motor pin and LEDC-channel configuration.
     */
    void begin(const MotorPinConfig& cfg);

    /**
     * @brief Drive motor forward at the given speed.
     *
     * Sets IN1=HIGH, IN2=LOW, PWM=speed.
     *
     * @param speed  PWM duty cycle (0–255).  0 effectively coasts.
     */
    void forward(uint8_t speed);

    /**
     * @brief Drive motor backward at the given speed.
     *
     * Sets IN1=LOW, IN2=HIGH, PWM=speed.
     *
     * @param speed  PWM duty cycle (0–255).  0 effectively coasts.
     */
    void backward(uint8_t speed);

    /**
     * @brief Apply active (short-circuit) brake.
     *
     * Sets IN1=HIGH, IN2=HIGH, PWM=255.  Both motor terminals are shorted
     * to the same rail, creating a strong braking torque.
     */
    void brake();

    /**
     * @brief Allow motor to free-wheel (coast).
     *
     * Sets IN1=LOW, IN2=LOW, PWM=0.  Motor terminals are floating.
     */
    void coast();

    /**
     * @brief Set speed and direction with a single signed value.
     *
     * Convenience wrapper mapping a signed int to forward/backward/brake:
     *  - speed > 0  → forward(speed)
     *  - speed < 0  → backward(-speed)
     *  - speed == 0 → brake()
     *
     * @param speed  Signed PWM duty cycle in range [–255, +255].
     *               Values outside this range are clamped automatically.
     */
    void setSpeed(int16_t speed);

    /**
     * @brief Return the current motor direction / state.
     * @return Current MotorDirection enum value.
     */
    MotorDirection getDirection() const;

    /**
     * @brief Return the last PWM duty cycle written (0–255).
     * @return Current duty cycle.
     */
    uint8_t getSpeed() const;

private:
    /**
     * @brief Write IN1, IN2, and PWM duty cycle in one call.
     *
     * @param in1   Logic level for IN1 pin.
     * @param in2   Logic level for IN2 pin.
     * @param pwm   Duty cycle value (0–255) for the LEDC channel.
     * @param dir   MotorDirection to record in _direction.
     */
    void _apply(bool in1, bool in2, uint8_t pwm, MotorDirection dir);

    MotorPinConfig  _cfg;           ///< Pin and LEDC-channel configuration
    MotorDirection  _direction;     ///< Current direction / state
    uint8_t         _speed;         ///< Current PWM duty cycle
    bool            _initialised;   ///< True after begin() is called
};

// ============================================================
//  MotorPID — per-motor PID speed controller
// ============================================================

/**
 * @brief Discrete PID controller for closed-loop RPM regulation of one motor.
 *
 * The controller computes a PWM output given an RPM setpoint and a measured
 * RPM (from MotorEncoder::getRPM()).  Output is clamped to [0, 255] and the
 * integral term is anti-windup clamped to ±PID_INTEGRAL_LIMIT.
 *
 * Call update() at a fixed, known interval (e.g., from a hardware timer ISR
 * or in loop() with a millis()-gated block) for consistent behaviour.
 *
 * @code
 *   // Typical usage in loop():
 *   static uint32_t lastPID = 0;
 *   if (millis() - lastPID >= 50) {          // 20 Hz update
 *       lastPID = millis();
 *       float measuredRPM = encoderA.getRPM();
 *       int16_t pwm = pidA.update(measuredRPM);
 *       motorA.setSpeed(pwm);
 *   }
 * @endcode
 */
class MotorPID {
public:
    /**
     * @brief Construct a PID controller with default gains.
     *
     * Gains default to PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT.
     * Call setGains() to tune.
     */
    MotorPID();

    /**
     * @brief Set PID gains.
     *
     * @param kp  Proportional gain.
     * @param ki  Integral gain.
     * @param kd  Derivative gain.
     */
    void setGains(float kp, float ki, float kd);

    /**
     * @brief Set the RPM target (setpoint).
     *
     * Positive values target forward motion; negative values target reverse.
     * The controller handles sign internally and calls Motor::forward() or
     * Motor::backward() as appropriate when the output is applied by the user.
     *
     * @param rpm  Target output-shaft speed in RPM.
     */
    void setSetpoint(float rpm);

    /**
     * @brief Return the current RPM setpoint.
     * @return Setpoint in RPM.
     */
    float getSetpoint() const;

    /**
     * @brief Compute the next PID output given the current measured RPM.
     *
     * Should be called at a consistent interval.  Internally tracks the
     * time between calls using millis() for the derivative and integral
     * terms if a fixed dt is not provided, but a fixed dt is preferred.
     *
     * @param measuredRPM  Current output-shaft speed (from MotorEncoder::getRPM()).
     * @return             Signed PWM output in [–255, +255].
     *                     Positive → forward, negative → backward.
     *                     Pass directly to Motor::setSpeed().
     */
    int16_t update(float measuredRPM);

    /**
     * @brief Reset integrator, derivative state, and internal timestamps.
     *
     * Call when the setpoint changes significantly or after a stop, to
     * prevent integrator windup artefacts on the next run.
     */
    void reset();

    /**
     * @brief Return the last computed raw (unclipped) PID output.
     * @return Raw PID output (float).
     */
    float getLastOutput() const;

    /**
     * @brief Return the last computed error (setpoint − measurement).
     * @return Error in RPM.
     */
    float getLastError() const;

private:
    // Gains
    float _kp;             ///< Proportional gain
    float _ki;             ///< Integral gain
    float _kd;             ///< Derivative gain

    // Controller state
    float    _setpoint;    ///< Target RPM
    float    _integral;    ///< Accumulated integral term
    float    _lastError;   ///< Error at previous update() call (for derivative)
    float    _lastOutput;  ///< Raw PID output from last update()

    // Timing
    uint32_t _lastUpdateMs; ///< millis() timestamp of last update() call
    bool     _firstUpdate;  ///< True before first update() call
};

// ============================================================
//  TB6612FNG_ESP32 — top-level dual-motor driver
// ============================================================

/**
 * @brief Top-level driver combining two Motor channels, two MotorEncoder
 *        instances, and two MotorPID controllers for the TB6612FNG IC.
 *
 * Manages the shared STBY (standby) pin.  Motor A maps to the AIN/PWMA
 * side and Motor B to the BIN/PWMB side.
 *
 * Typical setup:
 * @code
 *   MotorPinConfig cfgA   = { .pwmPin=18, .in1Pin=16, .in2Pin=17, .ledcChan=0 };
 *   MotorPinConfig cfgB   = { .pwmPin=19, .in1Pin=21, .in2Pin=22, .ledcChan=1 };
 *   EncoderPinConfig encA = { .pinA=34, .pinB=35 };
 *   EncoderPinConfig encB = { .pinA=32, .pinB=33 };
 *
 *   TB6612FNG_ESP32 driver;
 *   driver.begin(cfgA, cfgB, encA, encB, stbyPin=4);
 *
 *   driver.enable();
 *   driver.motorA.forward(180);
 *   float rpmA = driver.encoderA.getRPM();
 * @endcode
 *
 * @note  Public member objects (motorA, motorB, encoderA, encoderB, pidA, pidB)
 *        are intentionally public so user sketches can access them directly,
 *        matching the idiomatic Arduino "struct of objects" style.
 */
class TB6612FNG_ESP32 {
public:
    // --- Public sub-objects (direct access is intentional) ---

    Motor        motorA;    ///< Motor channel A (AIN1, AIN2, PWMA)
    Motor        motorB;    ///< Motor channel B (BIN1, BIN2, PWMB)
    MotorEncoder encoderA;  ///< Quadrature encoder for Motor A
    MotorEncoder encoderB;  ///< Quadrature encoder for Motor B
    MotorPID     pidA;      ///< PID speed controller for Motor A
    MotorPID     pidB;      ///< PID speed controller for Motor B

    /**
     * @brief Construct the driver object (no hardware interaction yet).
     *
     * All hardware setup is deferred to begin().
     */
    TB6612FNG_ESP32();

    /**
     * @brief Initialise both motor channels, both encoders, and STBY pin.
     *
     * @param motorCfgA   Pin config for Motor A (PWMA, AIN1, AIN2, LEDC channel).
     * @param motorCfgB   Pin config for Motor B (PWMB, BIN1, BIN2, LEDC channel).
     * @param encoderCfgA Encoder pin config for Motor A (pinA, pinB).
     * @param encoderCfgB Encoder pin config for Motor B (pinA, pinB).
     * @param stbyPin     GPIO connected to the TB6612FNG STBY pin.
     */
    void begin(const MotorPinConfig&   motorCfgA,
               const MotorPinConfig&   motorCfgB,
               const EncoderPinConfig& encoderCfgA,
               const EncoderPinConfig& encoderCfgB,
               uint8_t                 stbyPin);

    /**
     * @brief Pull STBY HIGH — enables both motor channels.
     *
     * Must be called before motors will respond to drive commands.
     */
    void enable();

    /**
     * @brief Pull STBY LOW — disables both motor channels (coasts both motors).
     *
     * Motors will free-wheel while STBY is LOW regardless of IN1/IN2/PWM.
     */
    void disable();

    /**
     * @brief Return true if the driver is currently enabled (STBY=HIGH).
     * @return True = enabled, false = standby.
     */
    bool isEnabled() const;

    /**
     * @brief Brake both motors simultaneously.
     */
    void brakeAll();

    /**
     * @brief Coast (free-wheel) both motors simultaneously.
     */
    void coastAll();

    /**
     * @brief Reset both encoder tick counters to zero.
     */
    void resetEncoders();

    /**
     * @brief Run one PID update cycle for both channels and apply PWM output.
     *
     * Reads getRPM() from each encoder, calls pidA.update() and pidB.update(),
     * then applies the signed result to each Motor via setSpeed().
     *
     * Call this from loop() or a periodic timer at a fixed interval.
     */
    void updatePID();

private:
    uint8_t _stbyPin;       ///< GPIO for TB6612FNG STBY
    bool    _enabled;       ///< Current STBY state
    bool    _initialised;   ///< True after begin() is called
};

#endif // TB6612FNG_ESP32_H
