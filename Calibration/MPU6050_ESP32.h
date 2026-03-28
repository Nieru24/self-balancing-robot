/**
 * @file MPU6050_ESP32.h
 * @brief Ground-up MPU-6050 IMU driver for ESP32 over I2C (Arduino framework).
 *
 * Targets the InvenSense MPU-6050 6-axis IMU. Covers:
 *   - Configurable I2C address and ESP32 pin mapping
 *   - Accelerometer / gyroscope full-scale range selection
 *   - Digital Low-Pass Filter (DLPF) configuration
 *   - Efficient 14-byte burst read (accel + temp + gyro)
 *   - Scaled output in g-force, °/s, and °C
 *   - Software calibration (offset averaging)
 *   - Complementary filter for roll / pitch angle fusion
 *   - Data-Ready interrupt configuration helper
 *
 * Register references are from:
 *   "MPU-6000 and MPU-6050 Register Map and Descriptions, Rev 4.2"
 *   (InvenSense document RM-MPU-6000A-00)
 *
 * @author  Your Name
 * @version 1.0.0
 * @date    2026-03-28
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// I2C addresses
// ---------------------------------------------------------------------------

/** I2C address when AD0 pin is pulled LOW (default). */
constexpr uint8_t MPU6050_ADDR_LOW  = 0x68;

/** I2C address when AD0 pin is pulled HIGH. */
constexpr uint8_t MPU6050_ADDR_HIGH = 0x69;

// ---------------------------------------------------------------------------
// Register map  (RM-MPU-6000A-00, Section 3)
// ---------------------------------------------------------------------------

/** Self-test registers (factory trim values). */
constexpr uint8_t MPU6050_REG_SELF_TEST_X     = 0x0D; ///< Accel X self-test
constexpr uint8_t MPU6050_REG_SELF_TEST_Y     = 0x0E; ///< Accel Y self-test
constexpr uint8_t MPU6050_REG_SELF_TEST_Z     = 0x0F; ///< Accel Z self-test
constexpr uint8_t MPU6050_REG_SELF_TEST_A     = 0x10; ///< Accel combined self-test

/** Sample rate divider (SMPLRT_DIV). Sample rate = Gyro rate / (1 + SMPLRT_DIV). */
constexpr uint8_t MPU6050_REG_SMPLRT_DIV      = 0x19;

/**
 * Configuration register (CONFIG).
 * Bits [5:3] = EXT_SYNC_SET, Bits [2:0] = DLPF_CFG.
 */
constexpr uint8_t MPU6050_REG_CONFIG          = 0x1A;

/**
 * Gyroscope configuration (GYRO_CONFIG).
 * Bits [4:3] = FS_SEL (full-scale range).
 */
constexpr uint8_t MPU6050_REG_GYRO_CONFIG     = 0x1B;

/**
 * Accelerometer configuration (ACCEL_CONFIG).
 * Bits [4:3] = AFS_SEL (full-scale range).
 */
constexpr uint8_t MPU6050_REG_ACCEL_CONFIG    = 0x1C;

/** FIFO enable register. */
constexpr uint8_t MPU6050_REG_FIFO_EN         = 0x23;

/** I2C master control. */
constexpr uint8_t MPU6050_REG_I2C_MST_CTRL    = 0x24;

/**
 * Interrupt pin / bypass enable configuration (INT_PIN_CFG).
 * Bit 1 = I2C_BYPASS_EN.
 */
constexpr uint8_t MPU6050_REG_INT_PIN_CFG     = 0x37;

/**
 * Interrupt enable register (INT_ENABLE).
 * Bit 0 = DATA_RDY_EN.
 */
constexpr uint8_t MPU6050_REG_INT_ENABLE      = 0x38;

/**
 * Interrupt status register (INT_STATUS).
 * Bit 0 = DATA_RDY_INT (cleared on read).
 */
constexpr uint8_t MPU6050_REG_INT_STATUS      = 0x3A;

// ---------------------------------------------------------------------------
// Sensor data output registers (burst-read 14 bytes from 0x3B to 0x48)
// ---------------------------------------------------------------------------

/** First byte of 14-byte sensor data block. */
constexpr uint8_t MPU6050_REG_ACCEL_XOUT_H    = 0x3B; ///< Accel X high byte
constexpr uint8_t MPU6050_REG_ACCEL_XOUT_L    = 0x3C; ///< Accel X low byte
constexpr uint8_t MPU6050_REG_ACCEL_YOUT_H    = 0x3D; ///< Accel Y high byte
constexpr uint8_t MPU6050_REG_ACCEL_YOUT_L    = 0x3E; ///< Accel Y low byte
constexpr uint8_t MPU6050_REG_ACCEL_ZOUT_H    = 0x3F; ///< Accel Z high byte
constexpr uint8_t MPU6050_REG_ACCEL_ZOUT_L    = 0x40; ///< Accel Z low byte
constexpr uint8_t MPU6050_REG_TEMP_OUT_H      = 0x41; ///< Temperature high byte
constexpr uint8_t MPU6050_REG_TEMP_OUT_L      = 0x42; ///< Temperature low byte
constexpr uint8_t MPU6050_REG_GYRO_XOUT_H     = 0x43; ///< Gyro X high byte
constexpr uint8_t MPU6050_REG_GYRO_XOUT_L     = 0x44; ///< Gyro X low byte
constexpr uint8_t MPU6050_REG_GYRO_YOUT_H     = 0x45; ///< Gyro Y high byte
constexpr uint8_t MPU6050_REG_GYRO_YOUT_L     = 0x46; ///< Gyro Y low byte
constexpr uint8_t MPU6050_REG_GYRO_ZOUT_H     = 0x47; ///< Gyro Z high byte
constexpr uint8_t MPU6050_REG_GYRO_ZOUT_L     = 0x48; ///< Gyro Z low byte

/** Number of bytes in one burst sensor read (accel + temp + gyro). */
constexpr uint8_t MPU6050_BURST_READ_LEN      = 14;

// ---------------------------------------------------------------------------
// Signal path reset register (SIGNAL_PATH_RESET)
// ---------------------------------------------------------------------------
constexpr uint8_t MPU6050_REG_SIGNAL_PATH_RST = 0x68;

// ---------------------------------------------------------------------------
// User control register (USER_CTRL)
// ---------------------------------------------------------------------------
constexpr uint8_t MPU6050_REG_USER_CTRL       = 0x6A;

// ---------------------------------------------------------------------------
// Power management registers
// ---------------------------------------------------------------------------

/**
 * Power Management 1 (PWR_MGMT_1).
 * Bit 6 = SLEEP, Bits [2:0] = CLKSEL.
 * Reset value: 0x40 (device starts in sleep mode).
 */
constexpr uint8_t MPU6050_REG_PWR_MGMT_1      = 0x6B;

/**
 * Power Management 2 (PWR_MGMT_2).
 * Bits [7:6] = LP_WAKE_CTRL, Bits [5:0] = individual axis standby bits.
 */
constexpr uint8_t MPU6050_REG_PWR_MGMT_2      = 0x6C;

// ---------------------------------------------------------------------------
// FIFO registers
// ---------------------------------------------------------------------------
constexpr uint8_t MPU6050_REG_FIFO_COUNT_H    = 0x72; ///< FIFO byte count high
constexpr uint8_t MPU6050_REG_FIFO_COUNT_L    = 0x73; ///< FIFO byte count low
constexpr uint8_t MPU6050_REG_FIFO_R_W        = 0x74; ///< FIFO read/write port

// ---------------------------------------------------------------------------
// Who Am I register
// ---------------------------------------------------------------------------

/** WHO_AM_I register — expected value 0x68 (bits [6:1] of I2C address). */
constexpr uint8_t MPU6050_REG_WHO_AM_I        = 0x75;

/** Expected response from WHO_AM_I (value is fixed at 0x68 regardless of AD0). */
constexpr uint8_t MPU6050_WHO_AM_I_VAL        = 0x68;

// ---------------------------------------------------------------------------
// PWR_MGMT_1 bit masks
// ---------------------------------------------------------------------------
constexpr uint8_t MPU6050_PWR_RESET           = 0x80; ///< DEVICE_RESET bit
constexpr uint8_t MPU6050_PWR_SLEEP           = 0x40; ///< SLEEP bit
constexpr uint8_t MPU6050_PWR_CYCLE           = 0x20; ///< CYCLE bit
constexpr uint8_t MPU6050_PWR_TEMP_DIS        = 0x08; ///< TEMP_DIS bit
constexpr uint8_t MPU6050_CLKSEL_PLL_XGYRO   = 0x01; ///< PLL with X-axis gyro ref (recommended)

// ---------------------------------------------------------------------------
// INT_PIN_CFG bit masks
// ---------------------------------------------------------------------------
constexpr uint8_t MPU6050_INT_LEVEL_LOW       = 0x80; ///< INT pin active LOW
constexpr uint8_t MPU6050_INT_OPEN_DRAIN      = 0x40; ///< INT pin open-drain
constexpr uint8_t MPU6050_INT_LATCH_EN        = 0x20; ///< Latch INT until cleared
constexpr uint8_t MPU6050_INT_RD_CLEAR        = 0x10; ///< Clear INT on any read
constexpr uint8_t MPU6050_I2C_BYPASS_EN       = 0x02; ///< Bypass I2C master

// ---------------------------------------------------------------------------
// INT_ENABLE / INT_STATUS bit masks
// ---------------------------------------------------------------------------
constexpr uint8_t MPU6050_INT_DATA_RDY        = 0x01; ///< Data Ready interrupt bit

// ---------------------------------------------------------------------------
// Temperature conversion constants  (RM §4.18)
// Formula: Temp_°C = RAW / 340.0 + 36.53
// ---------------------------------------------------------------------------
constexpr float MPU6050_TEMP_SENSITIVITY      = 340.0f; ///< LSB / °C
constexpr float MPU6050_TEMP_OFFSET           = 36.53f; ///< °C offset

// ---------------------------------------------------------------------------
// Default calibration sample count
// ---------------------------------------------------------------------------
constexpr uint16_t MPU6050_DEFAULT_CAL_SAMPLES = 500;

// ---------------------------------------------------------------------------
// Complementary filter default alpha (gyro trust weight)
// ---------------------------------------------------------------------------
constexpr float MPU6050_CF_ALPHA_DEFAULT      = 0.98f;

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/**
 * @brief Accelerometer full-scale range selection.
 *
 * Maps to AFS_SEL bits [4:3] of ACCEL_CONFIG (0x1C).
 * Sensitivity (LSB/g): ±2g=16384, ±4g=8192, ±8g=4096, ±16g=2048.
 */
enum class AccelFSR : uint8_t {
    FSR_2G  = 0x00, ///< ±2g  — 16384 LSB/g
    FSR_4G  = 0x08, ///< ±4g  — 8192  LSB/g
    FSR_8G  = 0x10, ///< ±8g  — 4096  LSB/g
    FSR_16G = 0x18  ///< ±16g — 2048  LSB/g
};

/**
 * @brief Gyroscope full-scale range selection.
 *
 * Maps to FS_SEL bits [4:3] of GYRO_CONFIG (0x1B).
 * Sensitivity (LSB/°/s): ±250=131, ±500=65.5, ±1000=32.8, ±2000=16.4.
 */
enum class GyroFSR : uint8_t {
    FSR_250DPS  = 0x00, ///< ±250 °/s  — 131.0 LSB/°/s
    FSR_500DPS  = 0x08, ///< ±500 °/s  — 65.5  LSB/°/s
    FSR_1000DPS = 0x10, ///< ±1000 °/s — 32.8  LSB/°/s
    FSR_2000DPS = 0x18  ///< ±2000 °/s — 16.4  LSB/°/s
};

/**
 * @brief Digital Low-Pass Filter bandwidth selection.
 *
 * Maps to DLPF_CFG bits [2:0] of CONFIG (0x1A).
 * Affects both accel and gyro bandwidth/delay.
 *
 * | Mode    | Accel BW (Hz) | Gyro BW (Hz) | Gyro delay (ms) |
 * |---------|---------------|--------------|-----------------|
 * | DLPF_0  | 260           | 256          | 0.98            |
 * | DLPF_1  | 184           | 188          | 1.9             |
 * | DLPF_2  | 94            | 98           | 2.8             |
 * | DLPF_3  | 44            | 42           | 4.8             |
 * | DLPF_4  | 21            | 20           | 8.3             |
 * | DLPF_5  | 10            | 10           | 13.4            |
 * | DLPF_6  | 5             | 5            | 18.6            |
 */
enum class DLPFMode : uint8_t {
    DLPF_0 = 0x00, ///< Accel 260 Hz / Gyro 256 Hz (DLPF disabled)
    DLPF_1 = 0x01, ///< Accel 184 Hz / Gyro 188 Hz
    DLPF_2 = 0x02, ///< Accel  94 Hz / Gyro  98 Hz
    DLPF_3 = 0x03, ///< Accel  44 Hz / Gyro  42 Hz  (good for balancing robots)
    DLPF_4 = 0x04, ///< Accel  21 Hz / Gyro  20 Hz
    DLPF_5 = 0x05, ///< Accel  10 Hz / Gyro  10 Hz
    DLPF_6 = 0x06  ///< Accel   5 Hz / Gyro   5 Hz
};

/**
 * @brief Clock source selection for PWR_MGMT_1 CLKSEL bits.
 *
 * InvenSense recommends using PLL with gyro reference for better stability.
 */
enum class ClockSource : uint8_t {
    INTERNAL_8MHZ = 0x00, ///< Internal 8 MHz oscillator
    PLL_XGYRO     = 0x01, ///< PLL with X-axis gyro reference (recommended)
    PLL_YGYRO     = 0x02, ///< PLL with Y-axis gyro reference
    PLL_ZGYRO     = 0x03, ///< PLL with Z-axis gyro reference
    PLL_EXT_32K   = 0x04, ///< PLL with external 32.768 kHz reference
    PLL_EXT_19M   = 0x05, ///< PLL with external 19.2 MHz reference
    STOP          = 0x07  ///< Stops the clock, keeps timing generator in reset
};

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/**
 * @brief Raw 16-bit sensor readings directly from registers.
 */
struct RawData {
    int16_t accelX; ///< Raw accelerometer X
    int16_t accelY; ///< Raw accelerometer Y
    int16_t accelZ; ///< Raw accelerometer Z
    int16_t temp;   ///< Raw temperature
    int16_t gyroX;  ///< Raw gyroscope X
    int16_t gyroY;  ///< Raw gyroscope Y
    int16_t gyroZ;  ///< Raw gyroscope Z
};

/**
 * @brief Scaled, calibration-corrected sensor data in physical units.
 */
struct ScaledData {
    float accelX;  ///< Acceleration X in g
    float accelY;  ///< Acceleration Y in g
    float accelZ;  ///< Acceleration Z in g
    float tempC;   ///< Temperature in °C
    float gyroX;   ///< Angular rate X in °/s
    float gyroY;   ///< Angular rate Y in °/s
    float gyroZ;   ///< Angular rate Z in °/s
};

/**
 * @brief Fused angle estimates from the complementary filter.
 */
struct Angles {
    float roll;    ///< Roll angle in degrees  (rotation about X axis)
    float pitch;   ///< Pitch angle in degrees (rotation about Y axis)
};

/**
 * @brief Calibration offsets averaged over N samples at rest.
 *
 * Stored as raw LSB values; subtracted before scaling.
 */
struct CalibrationOffsets {
    int16_t accelX; ///< Accel X bias in raw LSB
    int16_t accelY; ///< Accel Y bias in raw LSB
    int16_t accelZ; ///< Accel Z bias in raw LSB (gravity component removed)
    int16_t gyroX;  ///< Gyro X bias in raw LSB
    int16_t gyroY;  ///< Gyro Y bias in raw LSB
    int16_t gyroZ;  ///< Gyro Z bias in raw LSB
};

// ---------------------------------------------------------------------------
// Main driver class
// ---------------------------------------------------------------------------

/**
 * @brief MPU-6050 driver for ESP32 using the Arduino Wire (I2C) interface.
 *
 * ### Typical usage (self-balancing robot)
 * @code
 * MPU6050_ESP32 imu;
 *
 * void setup() {
 *     // SDA=21, SCL=22, address 0x68, 400 kHz
 *     if (!imu.begin(21, 22)) { while(true); }
 *     imu.setAccelFSR(AccelFSR::FSR_2G);
 *     imu.setGyroFSR(GyroFSR::FSR_500DPS);
 *     imu.setDLPF(DLPFMode::DLPF_3);
 *     imu.calibrate(500);   // average 500 samples at rest
 * }
 *
 * void loop() {
 *     Angles ang;
 *     imu.getAngles(ang);
 *     Serial.printf("Roll: %.2f  Pitch: %.2f\n", ang.roll, ang.pitch);
 * }
 * @endcode
 */
class MPU6050_ESP32 {
public:
    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    /**
     * @brief Construct driver with a fixed I2C address.
     *
     * Does not start I2C or communicate with the device.
     * Call begin() to initialise.
     *
     * @param address I2C address: MPU6050_ADDR_LOW (0x68) or MPU6050_ADDR_HIGH (0x69).
     */
    explicit MPU6050_ESP32(uint8_t address = MPU6050_ADDR_LOW);

    // -----------------------------------------------------------------------
    // Initialisation
    // -----------------------------------------------------------------------

    /**
     * @brief Initialise the I2C bus and wake the MPU-6050 from sleep.
     *
     * Performs the following in order:
     *   1. Calls Wire.begin(sda, scl) and sets the I2C clock.
     *   2. Reads WHO_AM_I (0x75) to verify device identity.
     *   3. Resets the device (PWR_MGMT_1 bit 7) and waits for it to clear.
     *   4. Clears the SLEEP bit and selects the PLL+X-gyro clock source.
     *   5. Applies default FSR, DLPF, and sample-rate settings.
     *
     * @param sdaPin  GPIO number for I2C SDA.
     * @param sclPin  GPIO number for I2C SCL.
     * @param clkHz   I2C clock frequency in Hz (default 400000).
     * @return true   Device found and initialised successfully.
     * @return false  WHO_AM_I mismatch or reset timed out.
     */
    bool begin(int sdaPin = 21, int sclPin = 22, uint32_t clkHz = 400000UL);

    /**
     * @brief Verify the device is present and responding on the I2C bus.
     *
     * Reads WHO_AM_I (0x75) and checks for the expected value 0x68.
     *
     * @return true  Device is present and responding.
     * @return false No ACK or wrong WHO_AM_I value.
     */
    bool isConnected() const;

    // -----------------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------------

    /**
     * @brief Set the accelerometer full-scale range.
     *
     * Writes AFS_SEL bits [4:3] of ACCEL_CONFIG (0x1C) and updates the
     * internal sensitivity divider used for scaling.
     *
     * @param fsr Desired range (AccelFSR::FSR_2G … FSR_16G).
     * @return true  Register write acknowledged.
     * @return false I2C error.
     */
    bool setAccelFSR(AccelFSR fsr);

    /**
     * @brief Set the gyroscope full-scale range.
     *
     * Writes FS_SEL bits [4:3] of GYRO_CONFIG (0x1B) and updates the
     * internal sensitivity divider used for scaling.
     *
     * @param fsr Desired range (GyroFSR::FSR_250DPS … FSR_2000DPS).
     * @return true  Register write acknowledged.
     * @return false I2C error.
     */
    bool setGyroFSR(GyroFSR fsr);

    /**
     * @brief Configure the Digital Low-Pass Filter bandwidth.
     *
     * Writes DLPF_CFG bits [2:0] of CONFIG (0x1A).
     * Also sets the sample-rate divider to 0 (output rate = gyro rate).
     *
     * @param mode Desired DLPF bandwidth (DLPFMode::DLPF_0 … DLPF_6).
     * @return true  Register write acknowledged.
     * @return false I2C error.
     */
    bool setDLPF(DLPFMode mode);

    /**
     * @brief Set the sample rate divider (SMPLRT_DIV, register 0x19).
     *
     * Sample rate = Gyroscope output rate / (1 + divider).
     * With DLPF enabled, gyro output rate = 1 kHz; otherwise 8 kHz.
     *
     * @param divider Value 0–255.
     * @return true  Write acknowledged.
     * @return false I2C error.
     */
    bool setSampleRateDivider(uint8_t divider);

    /**
     * @brief Set the I2C clock source selection.
     *
     * Writes CLKSEL bits [2:0] of PWR_MGMT_1 (0x6B).
     * PLL_XGYRO is recommended for reduced jitter.
     *
     * @param source Desired clock source.
     * @return true  Write acknowledged.
     * @return false I2C error.
     */
    bool setClockSource(ClockSource source);

    // -----------------------------------------------------------------------
    // Raw data access
    // -----------------------------------------------------------------------

    /**
     * @brief Burst-read all 14 sensor bytes from register 0x3B in one transaction.
     *
     * Populates all fields of @p raw: accel X/Y/Z, temperature, gyro X/Y/Z.
     * No offset correction is applied here; see getScaledData() for corrected values.
     *
     * @param[out] raw Destination structure for raw 16-bit values.
     * @return true  Exactly 14 bytes received.
     * @return false I2C error or short read.
     */
    bool getRawData(RawData &raw);

    // -----------------------------------------------------------------------
    // Scaled / physical-unit data
    // -----------------------------------------------------------------------

    /**
     * @brief Return calibration-corrected, scaled sensor data in physical units.
     *
     * Internally calls getRawData(), subtracts stored offsets, then divides
     * by the appropriate sensitivity divisor for the selected FSR.
     *
     * @param[out] data Destination for scaled values (g, °/s, °C).
     * @return true  Data successfully read and converted.
     * @return false I2C error.
     */
    bool getScaledData(ScaledData &data);

    /**
     * @brief Read temperature only (single burst still reads all 14 bytes internally).
     *
     * @param[out] tempC Temperature in degrees Celsius.
     * @return true  Read successful.
     * @return false I2C error.
     */
    bool getTemperatureC(float &tempC);

    // -----------------------------------------------------------------------
    // Calibration
    // -----------------------------------------------------------------------

    /**
     * @brief Compute and store accel and gyro offsets by averaging samples at rest.
     *
     * The sensor must be stationary and level during calibration (Z-axis pointing up).
     * Averages @p numSamples raw readings, then stores the mean as bias offsets.
     * For the accel Z axis, one full-scale LSB (gravity) is removed before storing
     * so that a stationary, level sensor reads [0, 0, 1g] after correction.
     *
     * @param numSamples Number of samples to average (default 500, max 32767).
     * @return true  Calibration completed and offsets stored.
     * @return false I2C error during sampling.
     */
    bool calibrate(uint16_t numSamples = MPU6050_DEFAULT_CAL_SAMPLES);

    /**
     * @brief Manually set calibration offsets (e.g. loaded from NVS/flash).
     *
     * @param offsets Struct of raw-LSB offsets for all six axes.
     */
    void setCalibrationOffsets(const CalibrationOffsets &offsets);

    /**
     * @brief Retrieve the currently stored calibration offsets.
     *
     * @param[out] offsets Destination for the current offset values.
     */
    void getCalibrationOffsets(CalibrationOffsets &offsets) const;

    /**
     * @brief Reset calibration offsets to zero (disable offset correction).
     */
    void resetCalibration();

    // -----------------------------------------------------------------------
    // Complementary filter
    // -----------------------------------------------------------------------

    /**
     * @brief Compute roll and pitch angles using a complementary filter.
     *
     * Fuses gyro integration with accel-derived tilt to correct gyro drift.
     * dt is computed automatically from the time elapsed since the previous call.
     * On the very first call the angles are initialised from the accelerometer only.
     *
     * Formula:
     * @code
     *   angle = alpha * (angle + gyroRate * dt) + (1 - alpha) * accelAngle
     * @endcode
     *
     * @param[out] angles  Destination for roll and pitch in degrees.
     * @param alpha        Complementary filter coefficient (0–1).
     *                     Higher values trust gyro more (less noise, slower correction).
     *                     Default 0.98 is suitable for most balancing applications.
     * @return true  Data read and angles updated.
     * @return false I2C error.
     */
    bool getAngles(Angles &angles, float alpha = MPU6050_CF_ALPHA_DEFAULT);

    /**
     * @brief Reset the complementary filter state.
     *
     * Forces re-initialisation from accelerometer on the next getAngles() call.
     * Call this after an extended pause to prevent large dt integration errors.
     */
    void resetFilter();

    // -----------------------------------------------------------------------
    // Interrupt support
    // -----------------------------------------------------------------------

    /**
     * @brief Configure the MPU-6050 Data Ready interrupt output.
     *
     * Writes INT_ENABLE (0x38) and INT_PIN_CFG (0x37).
     *
     * @param enable      true to enable Data Ready interrupt, false to disable.
     * @param activeLow   true = INT pin is active LOW, false = active HIGH (default).
     * @param openDrain   true = INT pin is open-drain, false = push-pull (default).
     * @param latchEnable true = INT pin held until INT_STATUS read, false = 50 µs pulse.
     * @return true  Registers written successfully.
     * @return false I2C error.
     */
    bool configureInterrupt(bool enable,
                            bool activeLow   = false,
                            bool openDrain   = false,
                            bool latchEnable = false);

    /**
     * @brief Attach an ISR to the ESP32 GPIO connected to the MPU-6050 INT pin.
     *
     * Wraps Arduino attachInterrupt() with the correct mode derived from
     * the pin configuration set in configureInterrupt().
     *
     * @note Call configureInterrupt() before this method.
     *
     * @param intPin    GPIO number connected to MPU-6050 INT.
     * @param isr       Interrupt service routine (IRAM_ATTR recommended).
     * @param mode      Arduino interrupt mode: RISING, FALLING, CHANGE, etc.
     *                  Defaults to RISING (INT active HIGH, pulse mode).
     */
    void attachDataReadyInterrupt(uint8_t intPin,
                                  void (*isr)(),
                                  int mode = RISING);

    /**
     * @brief Detach the Data Ready interrupt from the specified GPIO.
     *
     * @param intPin GPIO number to detach.
     */
    void detachDataReadyInterrupt(uint8_t intPin);

    /**
     * @brief Read and return the INT_STATUS register (0x3A).
     *
     * Reading this register clears the Data Ready interrupt flag on the device.
     *
     * @param[out] status Raw value of INT_STATUS.
     * @return true  Read successful.
     * @return false I2C error.
     */
    bool getInterruptStatus(uint8_t &status);

    // -----------------------------------------------------------------------
    // Low-level register access (advanced / debug use)
    // -----------------------------------------------------------------------

    /**
     * @brief Write a single byte to a register.
     *
     * @param reg   Register address.
     * @param value Byte to write.
     * @return true  ACK received from device.
     * @return false NACK or bus error.
     */
    bool writeRegister(uint8_t reg, uint8_t value) const;

    /**
     * @brief Read one or more consecutive bytes starting from @p reg.
     *
     * @param reg     Starting register address.
     * @param buffer  Destination array (must be at least @p length bytes).
     * @param length  Number of bytes to read.
     * @return true   Exactly @p length bytes received.
     * @return false  I2C error or short read.
     */
    bool readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) const;

private:
    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    /**
     * @brief Combine two consecutive bytes (high, low) into a signed 16-bit integer.
     *
     * @param high High byte (MSB).
     * @param low  Low byte (LSB).
     * @return Signed 16-bit result.
     */
    static int16_t combineBytes(uint8_t high, uint8_t low);

    /**
     * @brief Return the accel sensitivity divisor for the current FSR setting.
     *
     * @return LSB/g value: 16384, 8192, 4096, or 2048.
     */
    float accelSensitivity() const;

    /**
     * @brief Return the gyro sensitivity divisor for the current FSR setting.
     *
     * @return LSB/(°/s) value: 131.0, 65.5, 32.8, or 16.4.
     */
    float gyroSensitivity() const;

    /**
     * @brief Apply stored calibration offsets to a raw data struct in-place.
     *
     * @param[in,out] raw RawData to correct.
     */
    void applyOffsets(RawData &raw) const;

    // -----------------------------------------------------------------------
    // Private data members
    // -----------------------------------------------------------------------

    /** Pointer to the Wire instance used for all I2C transactions. */
    TwoWire *_wire;

    /** I2C address of this device instance (0x68 or 0x69). */
    uint8_t _address;

    /** Currently selected accelerometer full-scale range. */
    AccelFSR _accelFSR;

    /** Currently selected gyroscope full-scale range. */
    GyroFSR _gyroFSR;

    /** Currently selected DLPF mode. */
    DLPFMode _dlpfMode;

    /** Stored calibration offsets (raw LSB); zeroed on construction. */
    CalibrationOffsets _offsets;

    /** Complementary filter: roll angle state (degrees). */
    float _cfRoll;

    /** Complementary filter: pitch angle state (degrees). */
    float _cfPitch;

    /** Timestamp of last getAngles() call (microseconds, from micros()). */
    uint32_t _lastUpdateUs;

    /** True after the first getAngles() call initialises the filter from accel. */
    bool _filterInitialised;

    /** Cached INT_PIN_CFG active-low flag for attachDataReadyInterrupt(). */
    bool _intActiveLow;
};