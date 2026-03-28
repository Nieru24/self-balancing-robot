/**
 * @file TB6612FNG_ESP32.cpp
 * @brief Full implementation of the TB6612FNG_ESP32 library.
 */

#include "TB6612FNG_ESP32.h"

// ============================================================
//  Forward Declarations for ISRs
// ============================================================
void IRAM_ATTR isrEncA_ChanA();
void IRAM_ATTR isrEncA_ChanB();
void IRAM_ATTR isrEncB_ChanA();
void IRAM_ATTR isrEncB_ChanB();

// ============================================================
//  Static instance registry for ISR dispatch
// ============================================================
MotorEncoder* MotorEncoder::s_instances[2] = { nullptr, nullptr };

// ============================================================
//  Quadrature Gray-code lookup table
// ============================================================
static const int8_t QEM[16] = {
    0, +1, -1,  0, -1,  0,  0, +1, +1,  0,  0, -1,  0, -1, +1,  0
};

// ============================================================
//  MotorEncoder implementation
// ============================================================

MotorEncoder::MotorEncoder()
    : _cfg{0, 0}
    , _ticks(0)
    , _lastEncoded(0)
    , _lastTicksRPM(0)
    , _lastTimeRPM(0)
    , _index(0)
    , _initialised(false)
{}

void MotorEncoder::begin(const EncoderPinConfig& cfg, uint8_t index)
{
    if (index > 1) return;

    _cfg          = cfg;
    _index        = index;
    _ticks        = 0;
    _lastEncoded  = 0;
    _lastTimeRPM  = millis();
    _initialised  = true;

    s_instances[index] = this;

    pinMode(_cfg.pinA, INPUT);
    pinMode(_cfg.pinB, INPUT);

    uint8_t a = (uint8_t)digitalRead(_cfg.pinA);
    uint8_t b = (uint8_t)digitalRead(_cfg.pinB);
    _lastEncoded = (a << 1) | b;

    if (index == 0) {
        attachInterrupt(digitalPinToInterrupt(_cfg.pinA), isrEncA_ChanA, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_cfg.pinB), isrEncA_ChanB, CHANGE);
    } else {
        attachInterrupt(digitalPinToInterrupt(_cfg.pinA), isrEncB_ChanA, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_cfg.pinB), isrEncB_ChanB, CHANGE);
    }
}

int32_t MotorEncoder::getTicks() const
{
    portDISABLE_INTERRUPTS();
    int32_t t = _ticks;
    portENABLE_INTERRUPTS();
    return t;
}

void MotorEncoder::resetTicks()
{
    portDISABLE_INTERRUPTS();
    _ticks        = 0;
    _lastTicksRPM = 0;
    portENABLE_INTERRUPTS();
    _lastTimeRPM = millis();
}

float MotorEncoder::getRPM()
{
    uint32_t now     = millis();
    uint32_t deltaMs = now - _lastTimeRPM;
    if (deltaMs < 10) return 0.0f;

    portDISABLE_INTERRUPTS();
    int32_t currentTicks = _ticks;
    portENABLE_INTERRUPTS();

    int32_t deltaTicks = currentTicks - _lastTicksRPM;
    _lastTicksRPM = currentTicks;
    _lastTimeRPM  = now;

    return (static_cast<float>(deltaTicks) * 60000.0f)
           / (static_cast<float>(COUNTS_PER_OUTPUT_REV) * static_cast<float>(deltaMs));
}

float MotorEncoder::getAngleDeg() const
{
    return (static_cast<float>(getTicks()) / static_cast<float>(COUNTS_PER_OUTPUT_REV)) * 360.0f;
}

float MotorEncoder::getDistanceMM(float wheelDiameterMM) const
{
    constexpr float PI_F = 3.1415926535f;
    return (static_cast<float>(getTicks()) / static_cast<float>(COUNTS_PER_OUTPUT_REV))
           * PI_F * wheelDiameterMM;
}

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
//  ISR Stubs
// ============================================================
void IRAM_ATTR isrEncA_ChanA() { if (MotorEncoder::s_instances[0]) MotorEncoder::s_instances[0]->handleChannelA(); }
void IRAM_ATTR isrEncA_ChanB() { if (MotorEncoder::s_instances[0]) MotorEncoder::s_instances[0]->handleChannelB(); }
void IRAM_ATTR isrEncB_ChanA() { if (MotorEncoder::s_instances[1]) MotorEncoder::s_instances[1]->handleChannelA(); }
void IRAM_ATTR isrEncB_ChanB() { if (MotorEncoder::s_instances[1]) MotorEncoder::s_instances[1]->handleChannelB(); }

// ============================================================
//  Motor implementation
// ============================================================

Motor::Motor() : _direction(MotorDirection::COAST), _speed(0), _initialised(false) {}

void Motor::begin(const MotorPinConfig& cfg)
{
    _cfg         = cfg;
    _initialised = true;
    pinMode(_cfg.in1Pin, OUTPUT);
    pinMode(_cfg.in2Pin, OUTPUT);

    // Updated for ESP32 3.0: ledcAttach handles setup and pin assignment
    ledcAttach(_cfg.pwmPin, LEDC_FREQ_HZ, LEDC_RESOLUTION);
    coast();
}

void Motor::_apply(bool in1, bool in2, uint8_t pwm, MotorDirection dir)
{
    digitalWrite(_cfg.in1Pin, in1 ? HIGH : LOW);
    digitalWrite(_cfg.in2Pin, in2 ? HIGH : LOW);
    
    // Updated for ESP32 3.0: Write directly to the Pin
    ledcWrite(_cfg.pwmPin, pwm);

    _direction = dir;
    _speed     = pwm;
}

void Motor::forward(uint8_t speed)  { if(_initialised) _apply(true, false, speed, MotorDirection::FORWARD); }
void Motor::backward(uint8_t speed) { if(_initialised) _apply(false, true, speed, MotorDirection::BACKWARD); }
void Motor::brake()                 { if(_initialised) _apply(true, true, PWM_MAX, MotorDirection::BRAKE); }
void Motor::coast()                 { if(_initialised) _apply(false, false, PWM_MIN, MotorDirection::COAST); }

void Motor::setSpeed(int16_t speed)
{
    if (!_initialised) return;
    speed = constrain(speed, -255, 255);
    if (speed > 0)       forward(static_cast<uint8_t>(speed));
    else if (speed < 0)  backward(static_cast<uint8_t>(-speed));
    else                 brake();
}

MotorDirection Motor::getDirection() const { return _direction; }
uint8_t        Motor::getSpeed()     const { return _speed;     }

// ============================================================
//  MotorPID implementation
// ============================================================

MotorPID::MotorPID()
    : _kp(PID_KP_DEFAULT), _ki(PID_KI_DEFAULT), _kd(PID_KD_DEFAULT)
    , _setpoint(0.0f), _integral(0.0f), _lastError(0.0f), _firstUpdate(true)
{}

void MotorPID::setGains(float kp, float ki, float kd) { _kp = kp; _ki = ki; _kd = kd; }

void MotorPID::setSetpoint(float rpm)
{
    if (fabsf(rpm - _setpoint) > 10.0f) reset();
    _setpoint = rpm;
}

float MotorPID::getSetpoint() const { return _setpoint; }

void MotorPID::reset()
{
    _integral = 0.0f; _lastError = 0.0f; _firstUpdate = true;
}

int16_t MotorPID::update(float measuredRPM)
{
    uint32_t now = millis();
    if (_firstUpdate) {
        _lastUpdateMs = now; _lastError = _setpoint - measuredRPM; _firstUpdate = false;
        return 0;
    }

    uint32_t deltaMs = now - _lastUpdateMs;
    if (deltaMs == 0) return static_cast<int16_t>(_lastOutput);
    
    float dt = static_cast<float>(deltaMs) * 0.001f;
    float error = _setpoint - measuredRPM;
    _integral = constrain(_integral + (error * dt), -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
    float derivative = (error - _lastError) / dt;
    float output = (_kp * error) + (_ki * _integral) + (_kd * derivative);

    _lastError = error; _lastUpdateMs = now; _lastOutput = output;
    return static_cast<int16_t>(constrain(output, -255, 255));
}

float MotorPID::getLastOutput() const { return _lastOutput; }
float MotorPID::getLastError()  const { return _lastError;  }

// ============================================================
//  TB6612FNG_ESP32 implementation
// ============================================================

TB6612FNG_ESP32::TB6612FNG_ESP32() : _enabled(false), _initialised(false) {}

void TB6612FNG_ESP32::begin(const MotorPinConfig& motorCfgA, const MotorPinConfig& motorCfgB,
                             const EncoderPinConfig& encoderCfgA, const EncoderPinConfig& encoderCfgB,
                             uint8_t stbyPin)
{
    _stbyPin = stbyPin; _initialised = true;
    pinMode(_stbyPin, OUTPUT);
    digitalWrite(_stbyPin, LOW);
    
    motorA.begin(motorCfgA);
    motorB.begin(motorCfgB);
    encoderA.begin(encoderCfgA, 0);
    encoderB.begin(encoderCfgB, 1);
}

void TB6612FNG_ESP32::enable()  { if(_initialised) { digitalWrite(_stbyPin, HIGH); _enabled = true; } }
void TB6612FNG_ESP32::disable() { if(_initialised) { coastAll(); digitalWrite(_stbyPin, LOW); _enabled = false; } }
bool TB6612FNG_ESP32::isEnabled() const { return _enabled; }
void TB6612FNG_ESP32::brakeAll() { motorA.brake(); motorB.brake(); }
void TB6612FNG_ESP32::coastAll() { motorA.coast(); motorB.coast(); }
void TB6612FNG_ESP32::resetEncoders() { encoderA.resetTicks(); encoderB.resetTicks(); }

void TB6612FNG_ESP32::updatePID()
{
    if (!_initialised || !_enabled) return;
    motorA.setSpeed(pidA.update(encoderA.getRPM()));
    motorB.setSpeed(pidB.update(encoderB.getRPM()));
}