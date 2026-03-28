/**
 * @file    examples/BasicRead/BasicRead.ino
 * @brief   Read scaled accelerometer (g), gyroscope (°/s), and temperature
 *          (°C) from the MPU-6050 with calibration offsets pre-applied.
 *
 * -------------------------------------------------------------------------
 * WHAT THIS SKETCH DEMONSTRATES
 * -------------------------------------------------------------------------
 *  1. Minimal but correct driver initialisation on ESP32 default I2C pins.
 *  2. How to load hard-coded calibration offsets (obtained from
 *     examples/Calibration/Calibration.ino) so every reading is
 *     bias-corrected before it reaches your application code.
 *  3. Reading all seven physical quantities in one 14-byte burst via
 *     getScaledData() and printing them in a fixed-width, human-readable
 *     table at a steady 10 Hz rate.
 *  4. Non-blocking loop design: no delay() in the hot path; timing is
 *     handled with a millis() gate so the loop remains responsive.
 *  5. Graceful I2C error recovery: read failures are counted and reported
 *     rather than silently ignored or causing a hard halt.
 *
 * -------------------------------------------------------------------------
 * HARDWARE SETUP
 * -------------------------------------------------------------------------
 *   MPU-6050 VCC  ->  3.3 V   (do NOT use 5 V -- logic is 3.3 V)
 *   MPU-6050 GND  ->  GND
 *   MPU-6050 SDA  ->  GPIO 21  (ESP32 DevKit default)
 *   MPU-6050 SCL  ->  GPIO 22  (ESP32 DevKit default)
 *   MPU-6050 AD0  ->  GND      (sets I2C address to 0x68)
 *   MPU-6050 INT  ->  not connected for this sketch
 *
 *   Pull-up resistors: most GY-521 breakout boards include 4.7 kOhm
 *   pull-ups on SDA/SCL.  If you are using a bare MPU-6050 chip, add
 *   4.7 kOhm from SDA->3.3 V and SCL->3.3 V.
 *
 * -------------------------------------------------------------------------
 * CALIBRATION
 * -------------------------------------------------------------------------
 *   Run examples/Calibration/Calibration.ino first.
 *   Copy the CalibrationOffsets block it prints and paste the six offset
 *   values into the USER_OFFSETS section below, replacing the zeros.
 *
 *   If you skip this step the sketch still runs, but the raw hardware bias
 *   of the sensor (typically +/-50-150 LSB on accel, +/-10-50 LSB on gyro)
 *   will appear as a small constant error in every reading.
 *
 * -------------------------------------------------------------------------
 * EXPECTED SERIAL OUTPUT  (example -- your values will differ)
 * -------------------------------------------------------------------------
 *   MPU-6050 BasicRead
 *   ==================
 *   Accel FSR : +/-2g   Gyro FSR : +/-250dps   DLPF : 42Hz
 *   Offsets   :  aX=  -142  aY=    58  aZ=  -310  gX=    27  gY=   -11  gZ=     4
 *   Loop rate : 10 Hz  (print every 100 ms)
 *   ------------------------------------------------------------------
 *     [ms]     aX(g)   aY(g)   aZ(g)  gX(d/s) gY(d/s) gZ(d/s)   T(C)
 *   ------------------------------------------------------------------
 *      1024   -0.002   0.003   0.999   -0.08    0.15   -0.04   31.42
 *      1124    0.001  -0.001   1.001    0.11   -0.09    0.02   31.44
 *   ...
 *
 * SPDX-License-Identifier: MIT
 */

#include <Arduino.h>
#include "MPU6050_ESP32.h"

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

static constexpr int      PIN_SDA  = 21;       ///< ESP32 default I2C SDA
static constexpr int      PIN_SCL  = 22;       ///< ESP32 default I2C SCL
static constexpr uint32_t I2C_FREQ = 400000UL; ///< 400 kHz Fast Mode

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

/**
 * Full-scale ranges.
 * Must match what was used during calibration.
 * Change both here AND re-run Calibration.ino if you need a wider range.
 *
 *   AccelFSR::FSR_2G   -> +/-2g,    16384 LSB/g   (finest resolution)
 *   AccelFSR::FSR_4G   -> +/-4g,     8192 LSB/g
 *   AccelFSR::FSR_8G   -> +/-8g,     4096 LSB/g
 *   AccelFSR::FSR_16G  -> +/-16g,    2048 LSB/g
 *
 *   GyroFSR::FSR_250DPS  -> +/-250 deg/s,  131.0 LSB/(deg/s)  (finest)
 *   GyroFSR::FSR_500DPS  -> +/-500 deg/s,   65.5 LSB/(deg/s)
 *   GyroFSR::FSR_1000DPS -> +/-1000 deg/s,  32.8 LSB/(deg/s)
 *   GyroFSR::FSR_2000DPS -> +/-2000 deg/s,  16.4 LSB/(deg/s)
 */
static constexpr AccelFSR ACCEL_FSR = AccelFSR::FSR_2G;
static constexpr GyroFSR  GYRO_FSR  = GyroFSR::FSR_250DPS;

/**
 * Digital Low-Pass Filter bandwidth.
 * DLPF_3 (42 Hz) is a good starting point for robotics: enough bandwidth
 * to track motion, narrow enough to suppress vibration and power-supply noise.
 *
 *   DLPF_0 -> disabled (260 Hz accel / 256 Hz gyro) -- widest, most noise
 *   DLPF_1 -> 188 Hz accel / 188 Hz gyro
 *   DLPF_2 ->  98 Hz accel /  98 Hz gyro
 *   DLPF_3 ->  42 Hz accel /  42 Hz gyro  <-- default for this sketch
 *   DLPF_4 ->  20 Hz accel /  20 Hz gyro
 *   DLPF_5 ->  10 Hz accel /  10 Hz gyro
 *   DLPF_6 ->   5 Hz accel /   5 Hz gyro  -- smoothest, most lag
 */
static constexpr DLPFMode DLPF_MODE = DLPFMode::DLPF_3;

// ============================================================================
// CALIBRATION OFFSETS
// ============================================================================
//
// Paste the six values printed by Calibration.ino here.
// Units are raw LSB.  All zeros = no bias correction.
//
// Example output from Calibration.ino (your numbers will be different):
//
//   CalibrationOffsets savedOffsets = {
//       .accelX =   -142,
//       .accelY =     58,
//       .accelZ =   -310,
//       .gyroX  =     27,
//       .gyroY  =    -11,
//       .gyroZ  =      4
//   };
//
// Copy those six numbers into the fields below:

static const CalibrationOffsets USER_OFFSETS = {
    .accelX =   -695,
    .accelY =     96,
    .accelZ =   -278,
    .gyroX  =   -114,
    .gyroY  =   -275,
    .gyroZ  =   -189
};

// ============================================================================
// PRINT RATE
// ============================================================================

/**
 * Milliseconds between printed data rows.
 *
 *   100 ms = 10 Hz  -- comfortable for Serial Monitor watching   (default)
 *    50 ms = 20 Hz  -- good for plotting in Serial Plotter
 *    10 ms = 100 Hz -- fast logging; may need higher baud rate
 *
 * The sensor produces data at up to 1 kHz (with DLPF enabled).  Printing
 * every iteration would flood the Serial Monitor and stall the I2C bus.
 * This gate lets the loop run at full speed while output remains readable.
 */
static constexpr uint32_t PRINT_INTERVAL_MS = 100;

/**
 * Number of data rows to print before reprinting the column header.
 * Keeps units visible as the output scrolls.
 */
static constexpr uint16_t HEADER_REPEAT_ROWS = 30;

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

MPU6050_ESP32 imu(MPU6050_ADDR_LOW);  // AD0 -> GND -> I2C address 0x68

// ============================================================================
// FILE-SCOPE STATE
// ============================================================================

static uint32_t lastPrintMs    = 0;  ///< millis() at last printed row
static uint32_t readCount      = 0;  ///< successful getScaledData() calls
static uint32_t errorCount     = 0;  ///< failed    getScaledData() calls
static uint32_t errorCountPrev = 0;  ///< errorCount snapshot at last print
static uint16_t rowsSinceHdr   = 0;  ///< rows printed since last header

// Most recent successfully read data -- printed at the timed interval.
// Declared here (not inside loop) so it retains its last good value if
// a single read fails; the error notice flags the stale-data situation.
static ScaledData latestData = {};

// ============================================================================
// HELPER: printSigned()
// ============================================================================

/**
 * @brief Print a float right-aligned in a fixed field with an explicit sign.
 *
 * ESP32 newlib printf does not reliably honour the space flag (' ') for
 * positive floats in all toolchain versions, so we handle the sign manually.
 * This keeps the sign column stable so all data rows stay visually aligned.
 *
 * @param value     Value to print (sign handled internally).
 * @param width     Total field width INCLUDING the sign character.
 * @param decimals  Digits after the decimal point.
 */
static void printSigned(float value, uint8_t width, uint8_t decimals)
{
    const bool negative = (value < 0.0f);
    const float absVal  = negative ? -value : value;

    // The format string targets (width - 1) characters for the numeric part;
    // the sign character fills the remaining one.
    char fmt[12];
    snprintf(fmt, sizeof(fmt), "%%%u.%uf", static_cast<unsigned>(width - 1u),
             static_cast<unsigned>(decimals));

    Serial.print(negative ? '-' : ' ');
    Serial.printf(fmt, static_cast<double>(absVal));
}

// ============================================================================
// HELPER: printHeader()
// ============================================================================

/**
 * @brief Print the column-header separator block.
 *
 * Called once at startup and then every HEADER_REPEAT_ROWS rows so units
 * stay visible in a scrolling terminal.
 */
static void printHeader()
{
    Serial.println(F("------------------------------------------------------------------"));
    Serial.println(F("   [ms]     aX(g)   aY(g)   aZ(g)  gX(d/s) gY(d/s) gZ(d/s)  T(C)"));
    Serial.println(F("------------------------------------------------------------------"));
    Serial.flush();
}

// ============================================================================
// HELPER: printDataRow()
// ============================================================================

/**
 * @brief Print one row of sensor data in a fixed-width table format.
 *
 * Column layout:
 *
 *   Field    Width   Format        Notes
 *   ------   -----   ------        -----
 *   [ms]       7     %7lu          millis() timestamp
 *   aX         7     sign + 6.3f   acceleration X in g
 *   aY         7     sign + 6.3f   acceleration Y in g
 *   aZ         7     sign + 6.3f   acceleration Z in g
 *   gX         8     sign + 7.2f   angular rate X in deg/s
 *   gY         8     sign + 7.2f   angular rate Y in deg/s
 *   gZ         8     sign + 7.2f   angular rate Z in deg/s
 *   T          7     %7.2f         temperature in deg C (always positive)
 *
 * @param d  Scaled, calibration-corrected sensor data.
 */
static void printDataRow(const ScaledData &d)
{
    // Timestamp
    Serial.printf("%7lu", millis());

    // Accelerometer (g) -- 3 decimal places fits +/-2g neatly in 7 chars
    printSigned(d.accelX, 7, 3);
    printSigned(d.accelY, 7, 3);
    printSigned(d.accelZ, 7, 3);

    // Gyroscope (deg/s) -- 2 decimal places fits +/-250 dps neatly in 8 chars
    printSigned(d.gyroX, 8, 2);
    printSigned(d.gyroY, 8, 2);
    printSigned(d.gyroZ, 8, 2);

    // Temperature (deg C) -- always positive in any realistic environment
    Serial.printf("  %5.2f", static_cast<double>(d.tempC));

    Serial.println();
}

// ============================================================================
// HELPER: printConfig()
// ============================================================================

/**
 * @brief Print a one-time configuration summary to Serial.
 *
 * Gives the user immediate confirmation of the active FSR, DLPF bandwidth,
 * and whether real calibration offsets are loaded (non-zero values) or the
 * placeholder zeros are still in place.
 */
static void printConfig()
{
    // --- FSR and DLPF labels -------------------------------------------------
    Serial.print(F("Accel FSR : "));
    switch (ACCEL_FSR) {
        case AccelFSR::FSR_2G:  Serial.print(F("+/-2g"));  break;
        case AccelFSR::FSR_4G:  Serial.print(F("+/-4g"));  break;
        case AccelFSR::FSR_8G:  Serial.print(F("+/-8g"));  break;
        case AccelFSR::FSR_16G: Serial.print(F("+/-16g")); break;
    }
    Serial.print(F("   Gyro FSR : "));
    switch (GYRO_FSR) {
        case GyroFSR::FSR_250DPS:  Serial.print(F("+/-250dps"));  break;
        case GyroFSR::FSR_500DPS:  Serial.print(F("+/-500dps"));  break;
        case GyroFSR::FSR_1000DPS: Serial.print(F("+/-1000dps")); break;
        case GyroFSR::FSR_2000DPS: Serial.print(F("+/-2000dps")); break;
    }
    Serial.print(F("   DLPF : "));
    switch (DLPF_MODE) {
        case DLPFMode::DLPF_0: Serial.print(F("disabled")); break;
        case DLPFMode::DLPF_1: Serial.print(F("188Hz"));    break;
        case DLPFMode::DLPF_2: Serial.print(F("98Hz"));     break;
        case DLPFMode::DLPF_3: Serial.print(F("42Hz"));     break;
        case DLPFMode::DLPF_4: Serial.print(F("20Hz"));     break;
        case DLPFMode::DLPF_5: Serial.print(F("10Hz"));     break;
        case DLPFMode::DLPF_6: Serial.print(F("5Hz"));      break;
    }
    Serial.println();

    // --- Offset summary ------------------------------------------------------
    // Check whether any offset is non-zero; if all are zero, print a reminder.
    const bool hasOffsets = (USER_OFFSETS.accelX != 0 || USER_OFFSETS.accelY != 0 ||
                             USER_OFFSETS.accelZ != 0 || USER_OFFSETS.gyroX  != 0 ||
                             USER_OFFSETS.gyroY  != 0 || USER_OFFSETS.gyroZ  != 0);

    Serial.printf(
        "Offsets   :  aX=%5d  aY=%5d  aZ=%5d  gX=%5d  gY=%5d  gZ=%5d",
        static_cast<int>(USER_OFFSETS.accelX),
        static_cast<int>(USER_OFFSETS.accelY),
        static_cast<int>(USER_OFFSETS.accelZ),
        static_cast<int>(USER_OFFSETS.gyroX),
        static_cast<int>(USER_OFFSETS.gyroY),
        static_cast<int>(USER_OFFSETS.gyroZ)
    );

    if (!hasOffsets) {
        Serial.print(F("  <-- all zeros: run Calibration.ino first for best accuracy"));
    }
    Serial.println();

    // --- Loop rate -----------------------------------------------------------
    Serial.printf("Loop rate : %lu Hz  (print every %lu ms)\n",
                  1000UL / PRINT_INTERVAL_MS,
                  PRINT_INTERVAL_MS);
}

// ============================================================================
// setup()
// ============================================================================

void setup()
{
    Serial.begin(115200);

    // 2-second wait for the USB-UART bridge to enumerate and for the Serial
    // Monitor to open.  On ESP32 DevKit (CP2102/CH340), Serial is always true
    // so while(!Serial) never blocks; a fixed delay is more reliable.
    delay(2000);

    Serial.println();
    Serial.println(F("MPU-6050 BasicRead"));
    Serial.println(F("=================="));
    Serial.flush();

    // ---- 1. Initialise I2C bus and wake the MPU-6050 from sleep mode --------
    if (!imu.begin(PIN_SDA, PIN_SCL, I2C_FREQ)) {
        Serial.println(F("ERROR: MPU-6050 not found on the I2C bus."));
        Serial.println(F("Check: SDA->GPIO21  SCL->GPIO22  VCC->3.3V  AD0->GND"));
        Serial.flush();
        while (true) { delay(1000); }
    }

    // ---- 2. Set full-scale ranges -------------------------------------------
    if (!imu.setAccelFSR(ACCEL_FSR)) {
        Serial.println(F("ERROR: setAccelFSR() failed. Halting."));
        Serial.flush();
        while (true) { delay(1000); }
    }
    if (!imu.setGyroFSR(GYRO_FSR)) {
        Serial.println(F("ERROR: setGyroFSR() failed. Halting."));
        Serial.flush();
        while (true) { delay(1000); }
    }

    // ---- 3. Set DLPF bandwidth ----------------------------------------------
    if (!imu.setDLPF(DLPF_MODE)) {
        Serial.println(F("ERROR: setDLPF() failed. Halting."));
        Serial.flush();
        while (true) { delay(1000); }
    }

    // ---- 4. Load calibration offsets ----------------------------------------
    // setCalibrationOffsets() stores the values inside the driver.
    // Every subsequent getScaledData() call subtracts them before scaling.
    imu.setCalibrationOffsets(USER_OFFSETS);

    // ---- 5. Print configuration summary and column header -------------------
    printConfig();
    printHeader();

    lastPrintMs = millis();
}

// ============================================================================
// loop()
// ============================================================================

void loop()
{
    // -------------------------------------------------------------------------
    // Read the sensor on every loop iteration.
    //
    // getScaledData() issues one 14-byte I2C burst (~200 µs at 400 kHz),
    // subtracts the stored calibration offsets in integer arithmetic, then
    // divides by the FSR sensitivity to produce the seven float values.
    //
    // Reading every iteration (not just at the print interval) ensures the
    // data is always as fresh as possible when we do print it.
    // -------------------------------------------------------------------------
    ScaledData freshData;
    if (imu.getScaledData(freshData)) {
        latestData = freshData;   // overwrite only on a clean read
        ++readCount;
    } else {
        ++errorCount;
        // Do not print the error here -- that would flood the output if the
        // bus is intermittently glitchy.  The notice is printed at the next
        // scheduled interval instead.
    }

    // -------------------------------------------------------------------------
    // Timed print gate -- nothing below executes until PRINT_INTERVAL_MS
    // has elapsed since the last printed row.
    // -------------------------------------------------------------------------
    const uint32_t nowMs = millis();
    if (nowMs - lastPrintMs < PRINT_INTERVAL_MS) {
        return;
    }
    lastPrintMs = nowMs;

    // ---- Report new I2C errors (if any) appeared in this interval -----------
    if (errorCount > errorCountPrev) {
        Serial.printf(
            "  *** %lu I2C read error(s) in last interval  (total: %lu) ***\n",
            errorCount - errorCountPrev,
            errorCount
        );
        errorCountPrev = errorCount;
    }

    // ---- Print the latest data row ------------------------------------------
    // readCount > 0 guards against printing the zero-initialised latestData
    // if every single read since boot has failed (very unlikely, but correct).
    if (readCount > 0) {
        printDataRow(latestData);

        // Reprint the column header every HEADER_REPEAT_ROWS rows.
        ++rowsSinceHdr;
        if (rowsSinceHdr >= HEADER_REPEAT_ROWS) {
            printHeader();
            rowsSinceHdr = 0;
        }
    }
}
