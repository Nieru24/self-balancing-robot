/**
 * @file TB6612FNG_ESP32.cpp
 * @brief Full implementation of the TB6612FNG_ESP32 library.
 *
 * Implements:
 *  - Motor       : LEDC PWM + IN1/IN2 direction control
 *  - MotorEncoder: AB quadrature decoding via hardware interrupts (×4)
 *  - MotorPID    : Discrete PID speed controller with anti-windup
 *  - TB6612FNG_ESP32 : Top-level dual-channel wrapper with STBY management
 *
 * ISR architecture
 * ----------------
 * Four file-scope ISR stubs (IRAM_ATTR) are defined at the bottom of this
 * file — one per encoder channel per motor (A_chanA, A_chanB, B_chanA,
 * B_chanB).  Each stub reads the instance pointer from
 * MotorEncoder::s_instances[] and delegates to handleChannelA() or
 * handleChannelB().  This keeps ISR bodies as short as possible and avoids
 * any vtable/virtual-dispatch overhead inside an ISR context.
 *
 * Quadrature decoding
 * -------------------
 * Both rising AND falling edges of both channels are captured (CHANGE mode),
 * giving ×4 resolution: 11 PPR × 56 (gear) × 4 = 2464 counts/output rev.
 *
 * The Gray-code state machine encodes the previous 2-bit AB state in
 * _lastEncoded and computes direction from the 4-bit transition value
 * (previousState << 2 | currentState).  The lookup table approach is both
 * ISR-safe and branch-free.
 *
 * @version 1.0.0
 * @date    2025
 * @license MIT
 */

#include "TB6612FNG_ESP32.h"

// ============================================================
//  Static instance registry for ISR dispatch
// ============================================================

/**
 * Holds pointers to at most two MotorEncoder instances so the file-scope
 * ISR stubs can forward interrupts to the correct object.
 * Index 0 = Motor A encoder, Index 1 = Motor B encoder.
 */
MotorEncoder* MotorEncoder::s_instances[2] = { nullptr, nullptr };

// ============================================================
//  Quadrature Gray-code lookup table
// ============================================================
//
// The 4-bit key is formed as: (prevA << 3) | (prevB << 2) | (currA << 1) | currB
// Valid forward  transitions produce +1, valid backward transitions −1,
// invalid/no-change transitions produce 0.
//
// Transition table (Gray code quadrature):
//  prev→curr  | direction
//  00→01      | +1  (key = 0b0001 = 1)
//  01→11      | +1  (key = 0b0111 = 7)
//  11→10      | +1  (key = 0b1110 = 14)
//  10→00      | +1  (key = 0b1000 = 8)  <- wrapping forward
//  00→10      | -1  (key = 0b0010 = 2)
//  10→11      | -1  (key = 0b1011 = 11)
//  11→01      | -1  (key = 0b1101 = 13)
//  01→00      | -1  (key = 0b0100 = 4)  <- wrapping backward
//
// All other keys (same state, double-step glitch) resolve to 0.

static const int8_t QEM[16] = {
//  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
    0, +1, -1,  0, -1,  0,  0, +1, +1,  0,  0, -1,  0, -1, +1,  0
};

// ============================================================
//
//  MotorEncoder implementation
//
// ============================================================

// ------------------------------------------------------------
//  Constructor
// ------------------------------------------------------------

MotorEncoder::MotorEncoder()
    : _cfg{0, 0}
    , _ticks(0)
    , _lastEncoded(0)
    , _lastTicksRPM(0)
    , _lastTimeRPM(0)
    , _index(0)
    , _initialised(false)
{}

// ------------------------------------------------------------
//  begin()
// ------------------------------------------------------------

void MotorEncoder::begin(const EncoderPinConfig& cfg, uint8_t index)
{
    // Guard: index must be 0 or 1 (two motor channels only).
    if (index > 1) return;

    _cfg          = cfg;
    _index        = index;
    _ticks        = 0;
    _lastEncoded  = 0;
    _lastTicksRPM = 0;
    _lastTimeRPM  = millis();
    _initialised  = true;

    // Register this instance in the static dispatch table.
    s_instances[index] = this;

    // Configure encoder pins as inputs.
    // The JGB37-520 encoder has built-in pull-ups on its PCB so INPUT is
    // correct here; use INPUT_PULLUP only if the encoder board lacks them.
    pinMode(_cfg.pinA, INPUT);
    pinMode(_cfg.pinB, INPUT);

    // Seed the lastEncoded state from the current pin levels so the first
    // ISR call does not miscount a spurious transition.
    uint8_t a = (uint8_t)digitalRead(_cfg.pinA);
    uint8_t b = (uint8_t)digitalRead(_cfg.pinB);
    _lastEncoded = (a << 1) | b;

    // Attach interrupts — CHANGE mode captures all four edges per cycle.
    if (index == 0) {
        attachInterrupt(digitalPinToInterrupt(_cfg.pinA), isrEncA_ChanA, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_cfg.pinB), isrEncA_ChanB, CHANGE);
    } else {
        attachInterrupt(digitalPinToInterrupt(_cfg.pinA), isrEncB_ChanA, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_cfg.pinB), isrEncB_ChanB, CHANGE);
    }
}

// ------------------------------------------------------------
//  getTicks()  — atomic 32-bit read
// ------------------------------------------------------------

int32_t MotorEncoder::getTicks() const
{
    // On Xtensa (ESP32), 32-bit aligned reads are not guaranteed atomic from
    // the compiler's perspective when an ISR can preempt.  Disable interrupts
    // for the shortest possible window.
    portDISABLE_INTERRUPTS();
    int32_t t = _ticks;
    portENABLE_INTERRUPTS();
    return t;
}

// ------------------------------------------------------------
//  resetTicks()
// ------------------------------------------------------------

void MotorEncoder::resetTicks()
{
    portDISABLE_INTERRUPTS();
    _ticks        = 0;
    _lastTicksRPM = 0;
    portENABLE_INTERRUPTS();

    // Also reset the RPM baseline time so the next getRPM() call
    // does not produce a huge spurious spike.
    _lastTimeRPM = millis();
}

// ------------------------------------------------------------
//  getRPM()
// ------------------------------------------------------------
//
// Formula (from spec):
//   RPM = (deltaTicks / COUNTS_PER_OUTPUT_REV) / (deltaTimeMs / 60000.0)
//
// Rearranged to a single multiply/divide for floating-point efficiency:
//   RPM = deltaTicks * 60000.0 / (COUNTS_PER_OUTPUT_REV * deltaTimeMs)

float MotorEncoder::getRPM()
{
    uint32_t now     = millis();
    uint32_t deltaMs = now - _lastTimeRPM;

    // Avoid divide-by-zero and very short sampling windows (< 10 ms gives
    // noisy results at low speeds).
    if (deltaMs < 10) return 0.0f;

    // Snapshot tick count atomically.
    portDISABLE_INTERRUPTS();
    int32_t currentTicks = _ticks;
    portENABLE_INTERRUPTS();

    int32_t deltaTicks = currentTicks - _lastTicksRPM;

    // Update baseline for the next call.
    _lastTicksRPM = currentTicks;
    _lastTimeRPM  = now;

    // Compute RPM.
    return (static_cast<float>(deltaTicks) * 60000.0f)
           / (static_cast<float>(COUNTS_PER_OUTPUT_REV) * static_cast<float>(deltaMs));
}

// ------------------------------------------------------------
//  getAngleDeg()
// ------------------------------------------------------------
//
// Cumulative angle: (ticks / COUNTS_PER_OUTPUT_REV) * 360°

float MotorEncoder::getAngleDeg() const
{
    portDISABLE_INTERRUPTS();
    int32_t t = _ticks;
    portENABLE_INTERRUPTS();

    return (static_cast<float>(t) / static_cast<float>(COUNTS_PER_OUTPUT_REV)) * 360.0f;
}

// ------------------------------------------------------------
//  getDistanceMM()
// ------------------------------------------------------------
//
// Linear distance = (ticks / COUNTS_PER_OUTPUT_REV) * π * wheelDiameterMM

float MotorEncoder::getDistanceMM(float wheelDiameterMM) const
{
    portDISABLE_INTERRUPTS();
    int32_t t = _ticks;
    portENABLE_INTERRUPTS();

    // π embedded as a compile-time constant for clarity.
    constexpr float PI_F = 3.14159265358979f;

    return (static_cast<float>(t) / static_cast<float>(COUNTS_PER_OUTPUT_REV))
           * PI_F * wheelDiameterMM;
}

// ------------------------------------------------------------
//  handleChannelA() / handleChannelB() — called from ISR stubs
// ------------------------------------------------------------
//
// Both handlers perform the same Gray-code lookup; they exist separately
// only so each interrupt source can call the correct one after reading
// its own pin first (avoids an additional digitalRead inside the ISR by
// using the fact that the triggering pin just changed).
//
// ×4 decoding: every time either channel changes state we read BOTH
// current pin levels, form a 4-bit transition key with the previous state,
// and increment or decrement _ticks accordingly.

void IRAM_ATTR MotorEncoder::handleChannelA()
{
    uint8_t a = (uint8_t)digitalRead(_cfg.pinA);
    uint8_t b = (uint8_t)digitalRead(_cfg.pinB);

    uint8_t encoded = (a << 1) | b;
    uint8_t key     = (_lastEncoded << 2) | encoded;

    _ticks      += QEM[key & 0x0F];
    _lastEncoded = encoded;
}

void IRAM_ATTR MotorEncoder::handleChannelB()
{
    uint8_t a = (uint8_t)digitalRead(_cfg.pinA);
    uint8_t b = (uint8_t)digitalRead(_cfg.pinB);

    uint8_t encoded = (a << 1) | b;
    uint8_t key     = (_lastEncoded << 2) | encoded;

    _ticks      += QEM[key & 0x0F];
    _lastEncoded = encoded;
}

// ============================================================
//  File-scope ISR stubs  (IRAM_ATTR — must live in IRAM)
// ============================================================
//
// One pair per encoder instance.  The stub simply looks up the instance
// pointer and calls the appropriate handler.  The actual work stays in the
// class method so it is testable and doesn't duplicate logic.

void IRAM_ATTR isrEncA_ChanA() {
    if (MotorEncoder::s_instances[0]) {
        MotorEncoder::s_instances[0]->handleChannelA();
    }
}

void IRAM_ATTR isrEncA_ChanB() {
    if (MotorEncoder::s_instances[0]) {
        MotorEncoder::s_instances[0]->handleChannelB();
    }
}

void IRAM_ATTR isrEncB_ChanA() {
    if (MotorEncoder::s_instances[1]) {
        MotorEncoder::s_instances[1]->handleChannelA();
    }
}

void IRAM_ATTR isrEncB_ChanB() {
    if (MotorEncoder::s_instances[1]) {
        MotorEncoder::s_instances[1]->handleChannelB();
    }
}

// ============================================================
//
//  Motor implementation
//
// ============================================================

// ------------------------------------------------------------
//  Constructor
// ------------------------------------------------------------

Motor::Motor()
    : _cfg{0, 0, 0, 0}
    , _direction(MotorDirection::COAST)
    , _speed(0)
    , _initialised(false)
{}

// ------------------------------------------------------------
//  begin()
// ------------------------------------------------------------

void Motor::begin(const MotorPinConfig& cfg)
{
    _cfg         = cfg;
    _initialised = true;

    // Configure direction pins.
    pinMode(_cfg.in1Pin, OUTPUT);
    pinMode(_cfg.in2Pin, OUTPUT);

    // Configure LEDC PWM channel.
    // ledcSetup(channel, frequency_Hz, resolution_bits)
    ledcSetup(_cfg.ledcChan, LEDC_FREQ_HZ, LEDC_RESOLUTION);
    ledcAttachPin(_cfg.pwmPin, _cfg.ledcChan);

    // Start in COAST state — safe default, no unexpected motion.
    coast();
}

// ------------------------------------------------------------
//  _apply() — private primitive that writes all three outputs atomically
// ------------------------------------------------------------

void Motor::_apply(bool in1, bool in2, uint8_t pwm, MotorDirection dir)
{
    // Write direction pins before PWM so the H-bridge sees a valid
    // IN1/IN2 combination before any current flows.
    digitalWrite(_cfg.in1Pin, in1 ? HIGH : LOW);
    digitalWrite(_cfg.in2Pin, in2 ? HIGH : LOW);
    ledcWrite(_cfg.ledcChan, pwm);

    _direction = dir;
    _speed     = pwm;
}

// ------------------------------------------------------------
//  forward()
// ------------------------------------------------------------

void Motor::forward(uint8_t speed)
{
    if (!_initialised) return;
    // TB6612FNG truth table: FORWARD → IN1=H, IN2=L, PWM=speed
    _apply(true, false, speed, MotorDirection::FORWARD);
}

// ------------------------------------------------------------
//  backward()
// ------------------------------------------------------------

void Motor::backward(uint8_t speed)
{
    if (!_initialised) return;
    // TB6612FNG truth table: BACKWARD → IN1=L, IN2=H, PWM=speed
    _apply(false, true, speed, MotorDirection::BACKWARD);
}

// ------------------------------------------------------------
//  brake()
// ------------------------------------------------------------

void Motor::brake()
{
    if (!_initialised) return;
    // TB6612FNG truth table: BRAKE → IN1=H, IN2=H, PWM=H (255)
    // Both motor terminals are driven to VM, creating a short-circuit brake.
    _apply(true, true, PWM_MAX, MotorDirection::BRAKE);
}

// ------------------------------------------------------------
//  coast()
// ------------------------------------------------------------

void Motor::coast()
{
    if (!_initialised) return;
    // TB6612FNG truth table: COAST → IN1=L, IN2=L, PWM=L (0)
    // Both terminals float; motor free-wheels.
    _apply(false, false, PWM_MIN, MotorDirection::COAST);
}

// ------------------------------------------------------------
//  setSpeed()  — signed convenience wrapper
// ------------------------------------------------------------

void Motor::setSpeed(int16_t speed)
{
    if (!_initialised) return;

    // Clamp to valid signed range before extracting magnitude.
    if (speed >  (int16_t)PWM_MAX) speed =  (int16_t)PWM_MAX;
    if (speed < -(int16_t)PWM_MAX) speed = -(int16_t)PWM_MAX;

    if (speed > 0) {
        forward(static_cast<uint8_t>(speed));
    } else if (speed < 0) {
        backward(static_cast<uint8_t>(-speed));
    } else {
        // speed == 0: active brake is safer than coast for a balancing robot
        // because coast leaves the wheels free to roll under gravity.
        brake();
    }
}

// ------------------------------------------------------------
//  Getters
// ------------------------------------------------------------

MotorDirection Motor::getDirection() const { return _direction; }
uint8_t        Motor::getSpeed()     const { return _speed;     }

// ============================================================
//
//  MotorPID implementation
//
// ============================================================

// ------------------------------------------------------------
//  Constructor
// ------------------------------------------------------------

MotorPID::MotorPID()
    : _kp(PID_KP_DEFAULT)
    , _ki(PID_KI_DEFAULT)
    , _kd(PID_KD_DEFAULT)
    , _setpoint(0.0f)
    , _integral(0.0f)
    , _lastError(0.0f)
    , _lastOutput(0.0f)
    , _lastUpdateMs(0)
    , _firstUpdate(true)
{}

// ------------------------------------------------------------
//  setGains()
// ------------------------------------------------------------

void MotorPID::setGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

// ------------------------------------------------------------
//  setSetpoint()
// ------------------------------------------------------------

void MotorPID::setSetpoint(float rpm)
{
    // Reset integrator when the setpoint changes significantly (> 10 RPM)
    // to prevent the wound-up integral from the previous setpoint causing
    // a large overshoot transient.
    if (fabsf(rpm - _setpoint) > 10.0f) {
        reset();
    }
    _setpoint = rpm;
}

float MotorPID::getSetpoint() const { return _setpoint; }

// ------------------------------------------------------------
//  reset()
// ------------------------------------------------------------

void MotorPID::reset()
{
    _integral     = 0.0f;
    _lastError    = 0.0f;
    _lastOutput   = 0.0f;
    _firstUpdate  = true;
    _lastUpdateMs = 0;
}

// ------------------------------------------------------------
//  update()  — core PID computation
// ------------------------------------------------------------
//
// Discrete PID (position form):
//
//   error     = setpoint − measurement
//   integral += error * dt                    (clamped to ±PID_INTEGRAL_LIMIT)
//   derivative = (error − lastError) / dt
//   output    = Kp*error + Ki*integral + Kd*derivative
//   output    → clamped to [−255, +255]
//
// dt is derived from millis() so the gains are in units of
//   Kp: PWM / RPM
//   Ki: PWM / (RPM·s)
//   Kd: PWM·s / RPM

int16_t MotorPID::update(float measuredRPM)
{
    uint32_t now = millis();

    // On the very first call we have no valid dt; skip the derivative and
    // integral update for this tick — just record baseline and return 0.
    if (_firstUpdate) {
        _lastUpdateMs = now;
        _lastError    = _setpoint - measuredRPM;
        _firstUpdate  = false;
        return 0;
    }

    // Delta time in seconds.
    uint32_t deltaMs = now - _lastUpdateMs;
    if (deltaMs == 0) {
        // Called faster than 1 ms resolution — return last output unchanged.
        return static_cast<int16_t>(
            constrain(_lastOutput, -(float)PWM_MAX, (float)PWM_MAX));
    }
    float dt = static_cast<float>(deltaMs) * 0.001f; // ms → seconds

    // --- Error ---
    float error = _setpoint - measuredRPM;

    // --- Integral with anti-windup clamp ---
    _integral += error * dt;
    if      (_integral >  PID_INTEGRAL_LIMIT) _integral =  PID_INTEGRAL_LIMIT;
    else if (_integral < -PID_INTEGRAL_LIMIT) _integral = -PID_INTEGRAL_LIMIT;

    // --- Derivative (on error, not measurement, to avoid derivative kick
    //     on setpoint changes) ---
    float derivative = (error - _lastError) / dt;

    // --- PID sum ---
    float output = (_kp * error) + (_ki * _integral) + (_kd * derivative);

    // --- Update state ---
    _lastError    = error;
    _lastOutput   = output;
    _lastUpdateMs = now;

    // --- Clamp output to valid PWM range ---
    if      (output >  (float)PWM_MAX) output =  (float)PWM_MAX;
    else if (output < -(float)PWM_MAX) output = -(float)PWM_MAX;

    return static_cast<int16_t>(output);
}

// ------------------------------------------------------------
//  Diagnostic getters
// ------------------------------------------------------------

float MotorPID::getLastOutput() const { return _lastOutput; }
float MotorPID::getLastError()  const { return _lastError;  }

// ============================================================
//
//  TB6612FNG_ESP32 implementation
//
// ============================================================

// ------------------------------------------------------------
//  Constructor
// ------------------------------------------------------------

TB6612FNG_ESP32::TB6612FNG_ESP32()
    : _stbyPin(0)
    , _enabled(false)
    , _initialised(false)
{}

// ------------------------------------------------------------
//  begin()
// ------------------------------------------------------------

void TB6612FNG_ESP32::begin(const MotorPinConfig&   motorCfgA,
                             const MotorPinConfig&   motorCfgB,
                             const EncoderPinConfig& encoderCfgA,
                             const EncoderPinConfig& encoderCfgB,
                             uint8_t                 stbyPin)
{
    _stbyPin     = stbyPin;
    _initialised = true;

    // Configure and hold STBY LOW during init so no unexpected motion occurs
    // while pins are still being configured.
    pinMode(_stbyPin, OUTPUT);
    digitalWrite(_stbyPin, LOW);
    _enabled = false;

    // Initialise both motor channels.
    motorA.begin(motorCfgA);
    motorB.begin(motorCfgB);

    // Initialise both encoders.
    // Index 0 = Motor A, Index 1 = Motor B — must match ISR stub assignment.
    encoderA.begin(encoderCfgA, 0);
    encoderB.begin(encoderCfgB, 1);

    // PID controllers are already default-initialised by their constructors;
    // no additional setup is required here unless the user wants custom gains.
}

// ------------------------------------------------------------
//  enable() / disable()
// ------------------------------------------------------------

void TB6612FNG_ESP32::enable()
{
    if (!_initialised) return;
    digitalWrite(_stbyPin, HIGH);
    _enabled = true;
}

void TB6612FNG_ESP32::disable()
{
    if (!_initialised) return;
    // Pull STBY LOW — the TB6612FNG will coast both outputs regardless of
    // IN1/IN2/PWM states.  Coast the motors in software as well so our
    // internal state stays consistent.
    coastAll();
    digitalWrite(_stbyPin, LOW);
    _enabled = false;
}

bool TB6612FNG_ESP32::isEnabled() const { return _enabled; }

// ------------------------------------------------------------
//  brakeAll() / coastAll()
// ------------------------------------------------------------

void TB6612FNG_ESP32::brakeAll()
{
    motorA.brake();
    motorB.brake();
}

void TB6612FNG_ESP32::coastAll()
{
    motorA.coast();
    motorB.coast();
}

// ------------------------------------------------------------
//  resetEncoders()
// ------------------------------------------------------------

void TB6612FNG_ESP32::resetEncoders()
{
    encoderA.resetTicks();
    encoderB.resetTicks();
}

// ------------------------------------------------------------
//  updatePID()
// ------------------------------------------------------------
//
// Reads RPM from both encoders, runs one PID tick for each channel,
// and applies the signed PWM output to the corresponding motor.
//
// Expected call pattern (from loop() or a timer):
//
//   const uint32_t PID_INTERVAL_MS = 50;  // 20 Hz
//   static uint32_t lastPID = 0;
//   if (millis() - lastPID >= PID_INTERVAL_MS) {
//       lastPID = millis();
//       driver.updatePID();
//   }

void TB6612FNG_ESP32::updatePID()
{
    if (!_initialised || !_enabled) return;

    // Channel A
    float   rpmA = encoderA.getRPM();
    int16_t pwmA = pidA.update(rpmA);
    motorA.setSpeed(pwmA);

    // Channel B
    float   rpmB = encoderB.getRPM();
    int16_t pwmB = pidB.update(rpmB);
    motorB.setSpeed(pwmB);
}
