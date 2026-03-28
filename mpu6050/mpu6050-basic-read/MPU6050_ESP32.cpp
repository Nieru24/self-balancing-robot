/**
 * @file MPU6050_ESP32.cpp
 * @brief Implementation of the MPU6050_ESP32 driver for ESP32 / Arduino.
 *
 * All register references are from:
 *   "MPU-6000 and MPU-6050 Register Map and Descriptions, Rev 4.2"
 *   (InvenSense document RM-MPU-6000A-00)
 *
 * Design notes
 * ------------
 * - Every I2C transaction goes through writeRegister() / readRegisters().
 *   Those two primitives are the only places Wire is touched directly,
 *   making it trivial to swap the transport layer in the future.
 *
 * - Floating-point is used only after the raw int16_t data is fully
 *   corrected (offset subtraction stays in integer arithmetic).
 *
 * - The complementary filter uses micros() for dt to cope with loops
 *   that run faster or slower than 1 ms.  32-bit wrap-around is handled
 *   correctly by unsigned subtraction.
 *
 * - No dynamic memory is allocated anywhere in this file.
 *
 * SPDX-License-Identifier: MIT
 */

#include "MPU6050_ESP32.h"
#include <math.h>   // atan2f, sqrtf — provided by the ESP-IDF / newlib toolchain

// ---------------------------------------------------------------------------
// Internal constants (not exposed in the public API)
// ---------------------------------------------------------------------------

/** Maximum time (ms) to wait for the DEVICE_RESET bit to self-clear. */
static constexpr uint16_t RESET_TIMEOUT_MS    = 100;

/** Time (ms) to wait after reset before the device is ready to accept config. */
static constexpr uint8_t  POST_RESET_DELAY_MS = 10;

/**
 * Accel Z gravity compensation in raw LSB for each FSR.
 * When the sensor is flat (Z-up), raw Z reads +full-scale / 1g equivalent.
 * Subtracting this removes gravity so the calibrated Z offset targets 0.
 * Values: ±2g→16384, ±4g→8192, ±8g→4096, ±16g→2048
 */
static constexpr int16_t ACCEL_Z_GRAVITY_LSB_2G  = 16384;
static constexpr int16_t ACCEL_Z_GRAVITY_LSB_4G  =  8192;
static constexpr int16_t ACCEL_Z_GRAVITY_LSB_8G  =  4096;
static constexpr int16_t ACCEL_Z_GRAVITY_LSB_16G =  2048;

/** Convert degrees to radians (for atan2 output conversion). */
static constexpr float RAD_TO_DEG_F = 57.29577951f;  // 180 / π

// ===========================================================================
// Construction
// ===========================================================================

MPU6050_ESP32::MPU6050_ESP32(uint8_t address)
    : _wire(&Wire),
      _address(address),
      _accelFSR(AccelFSR::FSR_2G),
      _gyroFSR(GyroFSR::FSR_250DPS),
      _dlpfMode(DLPFMode::DLPF_0),
      _offsets{0, 0, 0, 0, 0, 0},
      _cfRoll(0.0f),
      _cfPitch(0.0f),
      _lastUpdateUs(0),
      _filterInitialised(false),
      _intActiveLow(false)
{
    // Nothing else to do — Wire is not started until begin() is called.
}

// ===========================================================================
// Initialisation
// ===========================================================================

bool MPU6050_ESP32::begin(int sdaPin, int sclPin, uint32_t clkHz)
{
    // ---- 1. Start the I2C peripheral ----------------------------------------
    _wire->begin(sdaPin, sclPin);
    _wire->setClock(clkHz);

    // ---- 2. Verify device identity ------------------------------------------
    // WHO_AM_I (0x75) always returns 0x68 regardless of the AD0 pin level.
    // A wrong value means either the device is absent or the bus is stuck.
    if (!isConnected()) {
        return false;
    }

    // ---- 3. Full device reset -----------------------------------------------
    // Setting bit 7 (DEVICE_RESET) of PWR_MGMT_1 resets all registers to
    // their power-on defaults, including clearing the SLEEP bit temporarily.
    // The bit self-clears when the reset completes (~10 ms typical).
    if (!writeRegister(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_RESET)) {
        return false;
    }

    // Poll until DEVICE_RESET clears (or timeout).
    const uint32_t resetDeadlineMs = millis() + RESET_TIMEOUT_MS;
    uint8_t pwrReg = MPU6050_PWR_RESET;
    while (pwrReg & MPU6050_PWR_RESET) {
        if (millis() > resetDeadlineMs) {
            return false;   // Reset hung — hardware issue
        }
        delay(1);
        uint8_t readBuf = 0;
        if (!readRegisters(MPU6050_REG_PWR_MGMT_1, &readBuf, 1)) {
            return false;
        }
        pwrReg = readBuf;
    }

    // Give the oscillator a moment to stabilise.
    delay(POST_RESET_DELAY_MS);

    // ---- 4. Wake device and select recommended clock source -----------------
    // After reset the device is in sleep mode (SLEEP bit = 1).
    // We clear SLEEP and simultaneously switch to the PLL-backed X-gyro clock,
    // which InvenSense recommends for better frequency stability.
    // PWR_MGMT_1: SLEEP=0, CLKSEL=001 → value 0x01
    if (!writeRegister(MPU6050_REG_PWR_MGMT_1,
                       static_cast<uint8_t>(ClockSource::PLL_XGYRO))) {
        return false;
    }

    // ---- 5. Apply safe default configuration --------------------------------
    // Sample rate divider: 0 → output rate equals gyro internal rate.
    if (!writeRegister(MPU6050_REG_SMPLRT_DIV, 0x00)) { return false; }

    // CONFIG: EXT_SYNC_SET=0 (disabled), DLPF=0 (widest bandwidth, disabled).
    // Caller should call setDLPF() to pick a bandwidth appropriate for their
    // application (DLPF_3 is recommended for a balancing robot).
    if (!writeRegister(MPU6050_REG_CONFIG,
                       static_cast<uint8_t>(DLPFMode::DLPF_0))) { return false; }

    // GYRO_CONFIG: FS_SEL=0 → ±250 °/s (most sensitive, good for startup).
    if (!writeRegister(MPU6050_REG_GYRO_CONFIG,
                       static_cast<uint8_t>(GyroFSR::FSR_250DPS))) { return false; }

    // ACCEL_CONFIG: AFS_SEL=0 → ±2g (most sensitive).
    if (!writeRegister(MPU6050_REG_ACCEL_CONFIG,
                       static_cast<uint8_t>(AccelFSR::FSR_2G))) { return false; }

    // Interrupts disabled by default — user calls configureInterrupt() if needed.
    if (!writeRegister(MPU6050_REG_INT_ENABLE, 0x00)) { return false; }

    // Sync internal FSR tracking to the defaults written above.
    _accelFSR = AccelFSR::FSR_2G;
    _gyroFSR  = GyroFSR::FSR_250DPS;
    _dlpfMode = DLPFMode::DLPF_0;

    return true;
}

// ---------------------------------------------------------------------------

bool MPU6050_ESP32::isConnected() const
{
    uint8_t whoAmI = 0;
    if (!readRegisters(MPU6050_REG_WHO_AM_I, &whoAmI, 1)) {
        return false;
    }
    return (whoAmI == MPU6050_WHO_AM_I_VAL);
}

// ===========================================================================
// Configuration
// ===========================================================================

bool MPU6050_ESP32::setAccelFSR(AccelFSR fsr)
{
    // The AFS_SEL field occupies bits [4:3].  All other bits in ACCEL_CONFIG
    // are either reserved or relate to self-test (unused here), so we can
    // write the enum value directly — enum values were defined to be the
    // exact bit patterns needed.
    if (!writeRegister(MPU6050_REG_ACCEL_CONFIG, static_cast<uint8_t>(fsr))) {
        return false;
    }
    _accelFSR = fsr;
    return true;
}

// ---------------------------------------------------------------------------

bool MPU6050_ESP32::setGyroFSR(GyroFSR fsr)
{
    // FS_SEL bits [4:3] of GYRO_CONFIG (0x1B).  Same reasoning as setAccelFSR.
    if (!writeRegister(MPU6050_REG_GYRO_CONFIG, static_cast<uint8_t>(fsr))) {
        return false;
    }
    _gyroFSR = fsr;
    return true;
}

// ---------------------------------------------------------------------------

bool MPU6050_ESP32::setDLPF(DLPFMode mode)
{
    // CONFIG register (0x1A): bits [5:3] = EXT_SYNC_SET (leave 0),
    // bits [2:0] = DLPF_CFG.  We only modify the lower 3 bits; the upper
    // bits should stay 0 for normal operation, so a direct write is safe.
    if (!writeRegister(MPU6050_REG_CONFIG, static_cast<uint8_t>(mode))) {
        return false;
    }
    _dlpfMode = mode;
    return true;
}

// ---------------------------------------------------------------------------

bool MPU6050_ESP32::setSampleRateDivider(uint8_t divider)
{
    return writeRegister(MPU6050_REG_SMPLRT_DIV, divider);
}

// ---------------------------------------------------------------------------

bool MPU6050_ESP32::setClockSource(ClockSource source)
{
    // Read-modify-write: preserve SLEEP, CYCLE, TEMP_DIS bits while
    // updating only the CLKSEL field [2:0].
    uint8_t currentReg = 0;
    if (!readRegisters(MPU6050_REG_PWR_MGMT_1, &currentReg, 1)) {
        return false;
    }
    const uint8_t newReg = static_cast<uint8_t>(
        (currentReg & 0xF8u) | (static_cast<uint8_t>(source) & 0x07u)
    );
    return writeRegister(MPU6050_REG_PWR_MGMT_1, newReg);
}

// ===========================================================================
// Raw data access
// ===========================================================================

bool MPU6050_ESP32::getRawData(RawData &raw)
{
    // Burst-read 14 consecutive bytes starting at ACCEL_XOUT_H (0x3B).
    // Register layout (RM §4.17–4.19):
    //   [0,1]   ACCEL_XOUT_H/L
    //   [2,3]   ACCEL_YOUT_H/L
    //   [4,5]   ACCEL_ZOUT_H/L
    //   [6,7]   TEMP_OUT_H/L
    //   [8,9]   GYRO_XOUT_H/L
    //   [10,11] GYRO_YOUT_H/L
    //   [12,13] GYRO_ZOUT_H/L
    uint8_t buf[MPU6050_BURST_READ_LEN];
    if (!readRegisters(MPU6050_REG_ACCEL_XOUT_H, buf, MPU6050_BURST_READ_LEN)) {
        return false;
    }

    raw.accelX = combineBytes(buf[0],  buf[1]);
    raw.accelY = combineBytes(buf[2],  buf[3]);
    raw.accelZ = combineBytes(buf[4],  buf[5]);
    raw.temp   = combineBytes(buf[6],  buf[7]);
    raw.gyroX  = combineBytes(buf[8],  buf[9]);
    raw.gyroY  = combineBytes(buf[10], buf[11]);
    raw.gyroZ  = combineBytes(buf[12], buf[13]);

    return true;
}

// ===========================================================================
// Scaled / physical-unit data
// ===========================================================================

bool MPU6050_ESP32::getScaledData(ScaledData &data)
{
    RawData raw;
    if (!getRawData(raw)) {
        return false;
    }

    // Apply calibration in integer domain before converting to float.
    applyOffsets(raw);

    const float aScale = accelSensitivity();
    const float gScale = gyroSensitivity();

    data.accelX = static_cast<float>(raw.accelX) / aScale;
    data.accelY = static_cast<float>(raw.accelY) / aScale;
    data.accelZ = static_cast<float>(raw.accelZ) / aScale;

    // Temperature formula from RM §4.18:
    //   Temperature (°C) = TEMP_OUT / 340 + 36.53
    data.tempC  = static_cast<float>(raw.temp) / MPU6050_TEMP_SENSITIVITY
                  + MPU6050_TEMP_OFFSET;

    data.gyroX  = static_cast<float>(raw.gyroX) / gScale;
    data.gyroY  = static_cast<float>(raw.gyroY) / gScale;
    data.gyroZ  = static_cast<float>(raw.gyroZ) / gScale;

    return true;
}

// ---------------------------------------------------------------------------

bool MPU6050_ESP32::getTemperatureC(float &tempC)
{
    // We still do the full burst read — it is the only way to safely clock
    // the temperature bytes out of the device.  The overhead is only 14 bytes
    // over I2C and avoids a narrower, more complex partial read.
    RawData raw;
    if (!getRawData(raw)) {
        return false;
    }
    tempC = static_cast<float>(raw.temp) / MPU6050_TEMP_SENSITIVITY
            + MPU6050_TEMP_OFFSET;
    return true;
}

// ===========================================================================
// Calibration
// ===========================================================================

bool MPU6050_ESP32::calibrate(uint16_t numSamples)
{
    // Accumulate sums in 32-bit signed integers to avoid int16_t overflow.
    // With 500 samples and ±32767 per sample, worst case is ~16 M — fits
    // comfortably in int32_t (max ~2.1 billion).
    int32_t sumAX = 0, sumAY = 0, sumAZ = 0;
    int32_t sumGX = 0, sumGY = 0, sumGZ = 0;

    // Temporarily zero out offsets so we are averaging raw hardware values,
    // not already-corrected values.
    CalibrationOffsets savedOffsets = _offsets;
    resetCalibration();

    for (uint16_t sampleIdx = 0; sampleIdx < numSamples; ++sampleIdx) {
        RawData raw;
        if (!getRawData(raw)) {
            // Restore previous offsets so the sensor is still usable.
            _offsets = savedOffsets;
            return false;
        }
        sumAX += raw.accelX;
        sumAY += raw.accelY;
        sumAZ += raw.accelZ;
        sumGX += raw.gyroX;
        sumGY += raw.gyroY;
        sumGZ += raw.gyroZ;

        // Small yield to keep the ESP32 watchdog happy during long runs.
        // delay(0) triggers the FreeRTOS scheduler without adding real delay.
        if ((sampleIdx & 0x1Fu) == 0) {   // every 32 samples
            delay(0);
        }
    }

    // Compute integer means.
    _offsets.accelX = static_cast<int16_t>(sumAX / numSamples);
    _offsets.accelY = static_cast<int16_t>(sumAY / numSamples);
    _offsets.gyroX  = static_cast<int16_t>(sumGX / numSamples);
    _offsets.gyroY  = static_cast<int16_t>(sumGY / numSamples);
    _offsets.gyroZ  = static_cast<int16_t>(sumGZ / numSamples);

    // Accel Z: subtract one full-scale LSB worth of gravity so that a
    // stationary, Z-up sensor reads 0 g after offset correction.
    // The gravity compensation value depends on the active FSR.
    int16_t gravityLSB = ACCEL_Z_GRAVITY_LSB_2G;   // default
    switch (_accelFSR) {
        case AccelFSR::FSR_2G:  gravityLSB = ACCEL_Z_GRAVITY_LSB_2G;  break;
        case AccelFSR::FSR_4G:  gravityLSB = ACCEL_Z_GRAVITY_LSB_4G;  break;
        case AccelFSR::FSR_8G:  gravityLSB = ACCEL_Z_GRAVITY_LSB_8G;  break;
        case AccelFSR::FSR_16G: gravityLSB = ACCEL_Z_GRAVITY_LSB_16G; break;
    }
    _offsets.accelZ = static_cast<int16_t>(sumAZ / numSamples) - gravityLSB;

    return true;
}

// ---------------------------------------------------------------------------

void MPU6050_ESP32::setCalibrationOffsets(const CalibrationOffsets &offsets)
{
    _offsets = offsets;
}

// ---------------------------------------------------------------------------

void MPU6050_ESP32::getCalibrationOffsets(CalibrationOffsets &offsets) const
{
    offsets = _offsets;
}

// ---------------------------------------------------------------------------

void MPU6050_ESP32::resetCalibration()
{
    _offsets = {0, 0, 0, 0, 0, 0};
}

// ===========================================================================
// Complementary filter
// ===========================================================================

bool MPU6050_ESP32::getAngles(Angles &angles, float alpha)
{
    ScaledData data;
    if (!getScaledData(data)) {
        return false;
    }

    // --- Accelerometer-derived angles ----------------------------------------
    // These are correct only when the sensor is quasi-static (no linear accel).
    // atan2 handles all four quadrants and avoids division by zero.
    //
    // Roll  = rotation about X axis → function of Y and Z accel components.
    // Pitch = rotation about Y axis → function of X and Z accel components.
    //
    // Reference: Freescale AN3461 "Tilt Sensing Using a Three-Axis Accelerometer"
    const float accelRollRad  = atan2f(data.accelY, data.accelZ);
    const float accelPitchRad = atan2f(-data.accelX,
                                       sqrtf(data.accelY * data.accelY
                                             + data.accelZ * data.accelZ));

    const float accelRollDeg  = accelRollRad  * RAD_TO_DEG_F;
    const float accelPitchDeg = accelPitchRad * RAD_TO_DEG_F;

    // --- Time delta ----------------------------------------------------------
    const uint32_t nowUs = micros();

    if (!_filterInitialised) {
        // Bootstrap the filter from accelerometer data on first call.
        // Gyro integration would produce a large error for whatever dt the
        // very first call calculates, so we skip it entirely.
        _cfRoll           = accelRollDeg;
        _cfPitch          = accelPitchDeg;
        _lastUpdateUs     = nowUs;
        _filterInitialised = true;

        angles.roll  = _cfRoll;
        angles.pitch = _cfPitch;
        return true;
    }

    // Unsigned subtraction wraps correctly even when micros() rolls over
    // the 32-bit boundary (~71.6 minutes of continuous runtime).
    const float dtSeconds = static_cast<float>(nowUs - _lastUpdateUs) * 1.0e-6f;
    _lastUpdateUs = nowUs;

    // Guard against spuriously large dt (e.g. after Serial.print blocking)
    // by clamping to a sensible maximum.  100 ms is already very slow for
    // a balancing robot but prevents a single bad dt from winding up the filter.
    const float dtClamped = (dtSeconds > 0.1f) ? 0.1f : dtSeconds;

    // --- Complementary filter update -----------------------------------------
    // Gyro rates are in °/s; integrating over dt gives the angular change.
    // Note: sign convention depends on your physical mounting orientation.
    //   _cfRoll  uses gyroX (rotation sensed on the X axis)
    //   _cfPitch uses gyroY (rotation sensed on the Y axis)
    // Swap or negate if the robot's IMU mounting differs from the canonical
    // X-forward, Y-left, Z-up frame.
    _cfRoll  = alpha * (_cfRoll  + data.gyroX * dtClamped)
               + (1.0f - alpha) * accelRollDeg;

    _cfPitch = alpha * (_cfPitch + data.gyroY * dtClamped)
               + (1.0f - alpha) * accelPitchDeg;

    angles.roll  = _cfRoll;
    angles.pitch = _cfPitch;
    return true;
}

// ---------------------------------------------------------------------------

void MPU6050_ESP32::resetFilter()
{
    _filterInitialised = false;
    _cfRoll            = 0.0f;
    _cfPitch           = 0.0f;
    _lastUpdateUs      = 0;
}

// ===========================================================================
// Interrupt support
// ===========================================================================

bool MPU6050_ESP32::configureInterrupt(bool enable,
                                       bool activeLow,
                                       bool openDrain,
                                       bool latchEnable)
{
    // Build INT_PIN_CFG byte (0x37).
    uint8_t pinCfg = 0x00;
    if (activeLow)   { pinCfg |= MPU6050_INT_LEVEL_LOW;  }
    if (openDrain)   { pinCfg |= MPU6050_INT_OPEN_DRAIN; }
    if (latchEnable) { pinCfg |= MPU6050_INT_LATCH_EN;   }
    // When latching, clear only on INT_STATUS read; otherwise any read clears.
    if (latchEnable) { pinCfg |= MPU6050_INT_RD_CLEAR;   }

    if (!writeRegister(MPU6050_REG_INT_PIN_CFG, pinCfg)) {
        return false;
    }

    // INT_ENABLE register (0x38): bit 0 = DATA_RDY_EN.
    const uint8_t intEnable = enable ? MPU6050_INT_DATA_RDY : 0x00u;
    if (!writeRegister(MPU6050_REG_INT_ENABLE, intEnable)) {
        return false;
    }

    // Cache the polarity so attachDataReadyInterrupt() can choose the
    // correct Arduino interrupt edge automatically.
    _intActiveLow = activeLow;
    return true;
}

// ---------------------------------------------------------------------------

void MPU6050_ESP32::attachDataReadyInterrupt(uint8_t intPin,
                                              void (*isr)(),
                                              int mode)
{
    pinMode(intPin, _intActiveLow ? INPUT_PULLUP : INPUT);
    ::attachInterrupt(digitalPinToInterrupt(intPin), isr, mode);
}

// ---------------------------------------------------------------------------

void MPU6050_ESP32::detachDataReadyInterrupt(uint8_t intPin)
{
    ::detachInterrupt(digitalPinToInterrupt(intPin));
}

// ---------------------------------------------------------------------------

bool MPU6050_ESP32::getInterruptStatus(uint8_t &status)
{
    return readRegisters(MPU6050_REG_INT_STATUS, &status, 1);
}

// ===========================================================================
// Low-level register access
// ===========================================================================

bool MPU6050_ESP32::writeRegister(uint8_t reg, uint8_t value) const
{
    _wire->beginTransmission(_address);
    _wire->write(reg);
    _wire->write(value);
    // endTransmission() returns 0 on success; any other value is an error.
    return (_wire->endTransmission() == 0);
}

// ---------------------------------------------------------------------------

bool MPU6050_ESP32::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) const
{
    // Send the register address with a repeated-start (stop=false keeps the
    // bus active so the subsequent read transaction succeeds).
    _wire->beginTransmission(_address);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) {
        return false;
    }

    // Request `length` bytes.  The ESP32 I2C driver on Arduino supports up to
    // 128 bytes per transaction by default; our maximum burst is 14, well within that.
    const uint8_t received = _wire->requestFrom(
        static_cast<uint8_t>(_address),
        length,
        static_cast<uint8_t>(true)   // send STOP after reading
    );

    if (received != length) {
        return false;
    }

    for (uint8_t byteIdx = 0; byteIdx < length; ++byteIdx) {
        buffer[byteIdx] = static_cast<uint8_t>(_wire->read());
    }
    return true;
}

// ===========================================================================
// Private helpers
// ===========================================================================

int16_t MPU6050_ESP32::combineBytes(uint8_t high, uint8_t low)
{
    // Cast to int16_t via uint16_t to avoid implementation-defined behaviour
    // when the MSB is set (which would make the result negative).
    return static_cast<int16_t>(
        (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low)
    );
}

// ---------------------------------------------------------------------------

float MPU6050_ESP32::accelSensitivity() const
{
    // Sensitivity values from RM Table 1 / §4.17.
    // LSB/g for each AFS_SEL setting.
    switch (_accelFSR) {
        case AccelFSR::FSR_2G:  return 16384.0f;
        case AccelFSR::FSR_4G:  return  8192.0f;
        case AccelFSR::FSR_8G:  return  4096.0f;
        case AccelFSR::FSR_16G: return  2048.0f;
        default:                return 16384.0f;   // safe fallback
    }
}

// ---------------------------------------------------------------------------

float MPU6050_ESP32::gyroSensitivity() const
{
    // Sensitivity values from RM Table 1 / §4.19.
    // LSB/(°/s) for each FS_SEL setting.
    switch (_gyroFSR) {
        case GyroFSR::FSR_250DPS:  return 131.0f;
        case GyroFSR::FSR_500DPS:  return  65.5f;
        case GyroFSR::FSR_1000DPS: return  32.8f;
        case GyroFSR::FSR_2000DPS: return  16.4f;
        default:                   return 131.0f;  // safe fallback
    }
}

// ---------------------------------------------------------------------------

void MPU6050_ESP32::applyOffsets(RawData &raw) const
{
    // Subtract biases computed during calibrate().
    // All arithmetic stays in int16_t; saturating at the int16_t boundary is
    // acceptable — an offset that large would indicate a hardware fault.
    raw.accelX = static_cast<int16_t>(raw.accelX - _offsets.accelX);
    raw.accelY = static_cast<int16_t>(raw.accelY - _offsets.accelY);
    raw.accelZ = static_cast<int16_t>(raw.accelZ - _offsets.accelZ);
    raw.gyroX  = static_cast<int16_t>(raw.gyroX  - _offsets.gyroX);
    raw.gyroY  = static_cast<int16_t>(raw.gyroY  - _offsets.gyroY);
    raw.gyroZ  = static_cast<int16_t>(raw.gyroZ  - _offsets.gyroZ);
    // Temperature has no calibration offset; raw.temp is left unchanged.
}