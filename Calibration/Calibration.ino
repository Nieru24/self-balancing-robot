/**
 * @file    examples/Calibration/Calibration.ino
 * @brief   MPU-6050 calibration utility for the MPU6050_ESP32 library.
 *
 * PURPOSE
 * -------
 * Collects 1 000 raw samples from the MPU-6050 while the sensor is
 * completely stationary and level (Z-axis pointing up).  It then:
 *
 *   1. Runs the library's built-in calibrate() method (averages samples,
 *      removes gravity from the Z-accel channel, stores offsets internally).
 *   2. Reads the computed offsets back via getCalibrationOffsets().
 *   3. Validates the result by printing 20 live scaled readings — a healthy
 *      sensor should show accel ≈ (0, 0, 1) g and gyro ≈ (0, 0, 0) °/s.
 *   4. Prints a ready-to-paste CalibrationOffsets initialiser that you can
 *      drop directly into any project sketch.
 *
 * HARDWARE SETUP
 * --------------
 *   MPU-6050 VCC  → 3.3 V
 *   MPU-6050 GND  → GND
 *   MPU-6050 SDA  → GPIO 21  (ESP32 default)
 *   MPU-6050 SCL  → GPIO 22  (ESP32 default)
 *   MPU-6050 AD0  → GND      (I2C address 0x68)
 *   MPU-6050 INT  → not connected for this sketch
 *
 * PROCEDURE
 * ---------
 *   1. Place the robot / board on a flat, vibration-free surface.
 *      The MPU-6050 must be level and completely still for the entire run.
 *   2. Flash this sketch.
 *   3. Open the Serial Monitor at 115200 baud.
 *   4. The sketch waits 5 s (SETTLE_DELAY_MS) after reset before sampling
 *      so you have time to set the board down after pressing the flash button.
 *   5. A progress bar is printed every 100 samples so you can confirm the
 *      sensor is running.
 *   6. After sampling, validation readings and the copy-paste block are
 *      printed.  Copy the CalibrationOffsets block into your project.
 *
 * FSR NOTE
 * --------
 * Calibration is run at ±2 g / ±250 °/s (the library defaults and the most
 * sensitive setting).  These offsets are valid ONLY when your production
 * sketch uses the same FSR.  If you change FSR, re-run this sketch.
 *
 * EXPECTED SERIAL OUTPUT (example values — yours will differ)
 * -----------------------------------------------------------
 *   MPU-6050 Calibration Utility
 *   ==============================
 *   Sensor found. Settling for 5 s — keep the board still...
 *   5... 4... 3... 2... 1...
 *
 *   Collecting 1000 samples at ±2g / ±250 °/s...
 *   [##########]  100 / 1000
 *   [####################]  200 / 1000
 *   ...
 *   [##################################################] 1000 / 1000
 *   Done.
 *
 *   --- Computed offsets (raw LSB) ---
 *   Accel X :   -142
 *   Accel Y :     58
 *   Accel Z :   -310   (gravity already removed)
 *   Gyro  X :     27
 *   Gyro  Y :    -11
 *   Gyro  Z :      4
 *
 *   --- Validation (20 live readings with offsets applied) ---
 *   #  1  aX=  0.003g  aY= -0.001g  aZ=  1.002g  gX= -0.02°/s  gY=  0.08°/s  gZ=  0.03°/s
 *   ...
 *
 *   --- Copy-paste block for your project sketch ---
 *   // FSR: accel ±2g  |  gyro ±250 °/s
 *   CalibrationOffsets savedOffsets = {
 *       .accelX = -142,
 *       .accelY =   58,
 *       .accelZ = -310,
 *       .gyroX  =   27,
 *       .gyroY  =  -11,
 *       .gyroZ  =    4
 *   };
 *   imu.setCalibrationOffsets(savedOffsets);
 *
 * SPDX-License-Identifier: MIT
 */

#include <Arduino.h>
#include "MPU6050_ESP32.h"

// ---------------------------------------------------------------------------
// Hardware pin assignments (ESP32 defaults)
// ---------------------------------------------------------------------------

static constexpr int PIN_SDA = 21;
static constexpr int PIN_SCL = 22;

// ---------------------------------------------------------------------------
// Calibration parameters
// ---------------------------------------------------------------------------

/** Number of samples to average.  More samples → lower noise floor in offsets. */
static constexpr uint16_t CAL_SAMPLES = 1000;

/**
 * Time (ms) to wait after Serial init before starting calibration.
 * Gives you a chance to put the board down flat after the flash is done.
 */
static constexpr uint32_t SETTLE_DELAY_MS = 5000;

/** How many live validation readings to print after calibration. */
static constexpr uint8_t VALIDATION_SAMPLES = 20;

/** Delay between validation readings (ms). */
static constexpr uint16_t VALIDATION_INTERVAL_MS = 50;

// ---------------------------------------------------------------------------
// FSR settings used during calibration.
// Must match the FSR your production sketch uses.
// ---------------------------------------------------------------------------

static constexpr AccelFSR CAL_ACCEL_FSR = AccelFSR::FSR_2G;
static constexpr GyroFSR  CAL_GYRO_FSR  = GyroFSR::FSR_250DPS;

// ---------------------------------------------------------------------------
// Global driver instance
// ---------------------------------------------------------------------------

MPU6050_ESP32 imu(MPU6050_ADDR_LOW);   // AD0 → GND

// ===========================================================================
// Helper: progress bar
// ===========================================================================

/**
 * @brief Print a fixed-width ASCII progress bar to Serial.
 *
 * Example output:  [##########          ]  500 / 1000
 *
 * @param done    Samples collected so far.
 * @param total   Total samples requested.
 * @param width   Number of '#' characters at 100 % (default 50).
 */
static void printProgress(uint16_t done, uint16_t total, uint8_t width = 50)
{
    const uint8_t filled = static_cast<uint8_t>(
        (static_cast<uint32_t>(done) * width) / total
    );

    Serial.print('[');
    for (uint8_t col = 0; col < width; ++col) {
        Serial.print(col < filled ? '#' : ' ');
    }
    Serial.print(']');
    Serial.printf("  %4u / %u\n", done, total);
}

// ===========================================================================
// Helper: countdown timer
// ===========================================================================

/**
 * @brief Print a seconds countdown then a newline.
 *
 * @param seconds Total seconds to count down from.
 */
static void printCountdown(uint8_t seconds)
{
    for (uint8_t remaining = seconds; remaining > 0; --remaining) {
        Serial.printf("%u... ", remaining);
        delay(1000);
    }
    Serial.println();
}

// ===========================================================================
// Helper: separator line
// ===========================================================================

static void printSeparator()
{
    Serial.println(F("--------------------------------------------------"));
}

// ===========================================================================
// Helper: column-aligned signed integer
// ===========================================================================

/**
 * @brief Print a signed 16-bit integer right-aligned in a field of fieldWidth.
 *
 * Using Serial.printf with %6d directly works on ESP32/Arduino's newlib
 * printf, but this wrapper makes the intent explicit and avoids any
 * platform inconsistency with 16-bit integer promotion.
 *
 * @param value      The value to print.
 * @param fieldWidth Minimum character width (space-padded on the left).
 */
static void printAligned(int16_t value, uint8_t fieldWidth = 6)
{
    Serial.printf("%*d", fieldWidth, static_cast<int>(value));
}

// ===========================================================================
// setup()
// ===========================================================================

void setup()
{
    Serial.begin(115200);

    // Give the host-side Serial Monitor time to connect and the USB-UART
    // bridge time to enumerate.  On ESP32 DevKit boards (CP2102 / CH340)
    // Serial is always true so while(!Serial) never blocks — we use a plain
    // timed wait instead so output is never lost.
    delay(2000);

    Serial.println();
    Serial.println(F("MPU-6050 Calibration Utility"));
    Serial.println(F("=============================="));
    Serial.flush();   // ensure the banner is in the TX buffer before blocking

    // ---- Initialise the IMU -------------------------------------------------
    if (!imu.begin(PIN_SDA, PIN_SCL)) {
        Serial.println(F("ERROR: MPU-6050 not found on the I2C bus."));
        Serial.println(F("Check wiring: SDA->GPIO21, SCL->GPIO22, VCC->3.3V, GND->GND."));
        Serial.println(F("Halting."));
        Serial.flush();
        while (true) { delay(1000); }
    }

    Serial.println(F("Sensor found. Settling for 5 s - keep the board still..."));
    Serial.flush();

    // ---- Apply the target FSR before sampling --------------------------------
    // Offsets are only valid for the FSR at which they were collected.
    if (!imu.setAccelFSR(CAL_ACCEL_FSR)) {
        Serial.println(F("ERROR: Could not set accelerometer FSR. Halting."));
        Serial.flush();
        while (true) { delay(1000); }
    }
    if (!imu.setGyroFSR(CAL_GYRO_FSR)) {
        Serial.println(F("ERROR: Could not set gyroscope FSR. Halting."));
        Serial.flush();
        while (true) { delay(1000); }
    }

    // Use DLPF_1 (184 Hz accel / 188 Hz gyro) during calibration.
    // This reduces high-frequency vibration noise in the averaged offsets
    // without meaningfully slowing the sample rate for a 1 000-sample run.
    if (!imu.setDLPF(DLPFMode::DLPF_1)) {
        Serial.println(F("ERROR: Could not configure DLPF. Halting."));
        Serial.flush();
        while (true) { delay(1000); }
    }

    // ---- Countdown so the user can set the board down flat ------------------
    printCountdown(static_cast<uint8_t>(SETTLE_DELAY_MS / 1000));

    // ===========================================================================
    // Phase 1 — Collect and average samples
    // ===========================================================================

    Serial.printf("\nCollecting %u samples at ", CAL_SAMPLES);

    switch (CAL_ACCEL_FSR) {
        case AccelFSR::FSR_2G:  Serial.print(F("±2g"));  break;
        case AccelFSR::FSR_4G:  Serial.print(F("±4g"));  break;
        case AccelFSR::FSR_8G:  Serial.print(F("±8g"));  break;
        case AccelFSR::FSR_16G: Serial.print(F("±16g")); break;
    }
    Serial.print(F(" / "));
    switch (CAL_GYRO_FSR) {
        case GyroFSR::FSR_250DPS:  Serial.print(F("±250 °/s"));  break;
        case GyroFSR::FSR_500DPS:  Serial.print(F("±500 °/s"));  break;
        case GyroFSR::FSR_1000DPS: Serial.print(F("±1000 °/s")); break;
        case GyroFSR::FSR_2000DPS: Serial.print(F("±2000 °/s")); break;
    }
    Serial.println(F("..."));

    // Accumulate raw samples manually so we can print live progress.
    // calibrate() does the same arithmetic internally; we replicate it here
    // to drive the progress bar, then let calibrate() perform the final
    // computation so the offsets are stored correctly inside the driver.
    //
    // Two-pass approach:
    //   Pass 1 (below)  — print progress while sampling; discard results.
    //   Pass 2 (calibrate()) — re-sample and store; fast, no progress needed.
    //
    // The sensor is physically stable so both passes produce near-identical
    // means.  The tiny noise difference (< 1 LSB) is inconsequential.

    static constexpr uint16_t PROGRESS_INTERVAL = 100;   // report every N samples

    for (uint16_t sampleIdx = 1; sampleIdx <= CAL_SAMPLES; ++sampleIdx) {
        RawData throwaway;
        if (!imu.getRawData(throwaway)) {
            Serial.printf("\nERROR: I2C read failed at sample %u. Halting.\n",
                          sampleIdx);
            Serial.flush();
            while (true) { delay(1000); }
        }

        if ((sampleIdx % PROGRESS_INTERVAL) == 0) {
            printProgress(sampleIdx, CAL_SAMPLES);
        }

        // Yield to FreeRTOS watchdog every 32 samples (mirrors calibrate() logic).
        if ((sampleIdx & 0x1Fu) == 0) {
            delay(0);
        }
    }

    Serial.println(F("Done."));

    // ===========================================================================
    // Phase 2 — Library calibrate() computes and stores offsets
    // ===========================================================================
    // The brief extra settle time between the progress pass and this pass is
    // harmless; the sensor is still stationary.

    if (!imu.calibrate(CAL_SAMPLES)) {
        Serial.println(F("\nERROR: calibrate() failed mid-run (I2C error). Halting."));
        Serial.flush();
        while (true) { delay(1000); }
    }

    // ===========================================================================
    // Phase 3 — Read back and display computed offsets
    // ===========================================================================

    CalibrationOffsets offsets;
    imu.getCalibrationOffsets(offsets);

    Serial.println();
    printSeparator();
    Serial.println(F("Computed offsets (raw LSB)"));
    printSeparator();

    Serial.print(F("Accel X : ")); printAligned(offsets.accelX); Serial.println();
    Serial.print(F("Accel Y : ")); printAligned(offsets.accelY); Serial.println();
    Serial.print(F("Accel Z : ")); printAligned(offsets.accelZ);
    Serial.println(F("   (gravity already removed)"));
    Serial.print(F("Gyro  X : ")); printAligned(offsets.gyroX);  Serial.println();
    Serial.print(F("Gyro  Y : ")); printAligned(offsets.gyroY);  Serial.println();
    Serial.print(F("Gyro  Z : ")); printAligned(offsets.gyroZ);  Serial.println();

    // ===========================================================================
    // Phase 4 — Sanity check: print scaled live readings
    // ===========================================================================
    // With offsets applied the corrected output should be very close to:
    //   accelX ≈ 0 g   accelY ≈ 0 g   accelZ ≈ 1 g
    //   gyroX  ≈ 0°/s  gyroY  ≈ 0°/s  gyroZ  ≈ 0°/s
    //
    // If any axis is far from these targets, suspect vibration during
    // sampling, incorrect FSR, or a hardware defect.

    Serial.println();
    printSeparator();
    Serial.printf("Validation: %u live readings (offsets applied)\n",
                  VALIDATION_SAMPLES);
    Serial.println(F("Target: aX≈0g  aY≈0g  aZ≈1g  gX≈0°/s  gY≈0°/s  gZ≈0°/s"));
    printSeparator();

    for (uint8_t readingIdx = 1; readingIdx <= VALIDATION_SAMPLES; ++readingIdx) {
        ScaledData scaled;
        if (!imu.getScaledData(scaled)) {
            Serial.printf("#%2u  ERROR: I2C read failed.\n", readingIdx);
        } else {
            Serial.printf(
                "#%2u  aX=%7.3fg  aY=%7.3fg  aZ=%7.3fg"
                "  gX=%7.2f\xc2\xb0/s  gY=%7.2f\xc2\xb0/s  gZ=%7.2f\xc2\xb0/s\n",
                // \xc2\xb0 is the UTF-8 encoding of the degree symbol °
                readingIdx,
                static_cast<double>(scaled.accelX),
                static_cast<double>(scaled.accelY),
                static_cast<double>(scaled.accelZ),
                static_cast<double>(scaled.gyroX),
                static_cast<double>(scaled.gyroY),
                static_cast<double>(scaled.gyroZ)
            );
        }
        delay(VALIDATION_INTERVAL_MS);
    }

    // ===========================================================================
    // Phase 5 — Copy-paste block
    // ===========================================================================

    Serial.println();
    printSeparator();
    Serial.println(F("Copy-paste block for your project sketch"));
    printSeparator();
    Serial.println(F("// Paste this into your setup() after imu.begin()."));
    Serial.println(F("// Re-run Calibration.ino if you change the FSR settings."));
    Serial.println();

    // Print the FSR selection lines so the reader knows which FSR these
    // offsets belong to.
    Serial.print(F("imu.setAccelFSR(AccelFSR::"));
    switch (CAL_ACCEL_FSR) {
        case AccelFSR::FSR_2G:  Serial.print(F("FSR_2G"));  break;
        case AccelFSR::FSR_4G:  Serial.print(F("FSR_4G"));  break;
        case AccelFSR::FSR_8G:  Serial.print(F("FSR_8G"));  break;
        case AccelFSR::FSR_16G: Serial.print(F("FSR_16G")); break;
    }
    Serial.println(F(");"));

    Serial.print(F("imu.setGyroFSR(GyroFSR::"));
    switch (CAL_GYRO_FSR) {
        case GyroFSR::FSR_250DPS:  Serial.print(F("FSR_250DPS"));  break;
        case GyroFSR::FSR_500DPS:  Serial.print(F("FSR_500DPS"));  break;
        case GyroFSR::FSR_1000DPS: Serial.print(F("FSR_1000DPS")); break;
        case GyroFSR::FSR_2000DPS: Serial.print(F("FSR_2000DPS")); break;
    }
    Serial.println(F(");"));
    Serial.println();

    // Designated-initialiser struct literal — valid C++20 and also accepted
    // by GCC/G++ (which ESP32 Arduino uses) as a C99-style extension in C++11.
    Serial.println(F("CalibrationOffsets savedOffsets = {"));
    Serial.printf( "    .accelX = %6d,\n", static_cast<int>(offsets.accelX));
    Serial.printf( "    .accelY = %6d,\n", static_cast<int>(offsets.accelY));
    Serial.printf( "    .accelZ = %6d,\n", static_cast<int>(offsets.accelZ));
    Serial.printf( "    .gyroX  = %6d,\n", static_cast<int>(offsets.gyroX));
    Serial.printf( "    .gyroY  = %6d,\n", static_cast<int>(offsets.gyroY));
    Serial.printf( "    .gyroZ  = %6d\n",  static_cast<int>(offsets.gyroZ));
    Serial.println(F("};"));
    Serial.println(F("imu.setCalibrationOffsets(savedOffsets);"));

    Serial.println();
    Serial.println(F("Calibration complete. You may now flash your main sketch."));
    printSeparator();
    Serial.flush();   // guarantee all output is sent before loop() starts
}

// ===========================================================================
// loop()
// ===========================================================================

void loop()
{
    // Calibration is a one-shot operation performed entirely in setup().
    // The loop blinks the built-in LED at 1 Hz as a visual "done" indicator.
    // GPIO 2 is the built-in LED on most ESP32 DevKit boards; adjust if yours
    // differs (e.g. GPIO 13 on SparkFun ESP32 Thing, GPIO 38 on S3 DevKitC).
    static constexpr uint8_t LED_PIN       = 2;
    static constexpr uint16_t BLINK_HZ_MS  = 500;   // half-period → 1 Hz blink

    static bool ledInitialised = false;
    if (!ledInitialised) {
        pinMode(LED_PIN, OUTPUT);
        ledInitialised = true;
    }

    digitalWrite(LED_PIN, HIGH);
    delay(BLINK_HZ_MS);
    digitalWrite(LED_PIN, LOW);
    delay(BLINK_HZ_MS);
}
