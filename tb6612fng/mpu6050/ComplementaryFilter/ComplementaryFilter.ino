/**
 * @file    examples/ComplementaryFilter/ComplementaryFilter.ino
 * @brief   Complementary-filter roll and pitch angles for a self-balancing
 *          robot, with Serial Monitor output and Serial Plotter support.
 *
 * =========================================================================
 * WHAT THIS SKETCH DEMONSTRATES
 * =========================================================================
 *  1. Correct sensor and filter configuration for a self-balancing robot:
 *       - FSR_2G / FSR_500DPS — widest dynamic range for balance movements
 *       - DLPF_2 (98 Hz) — fast enough to track a falling robot, narrow
 *         enough to suppress motor vibration fed back through the frame
 *       - alpha = 0.98 — trusts the gyro 98% per step, lets the
 *         accelerometer correct slow drift over ~50 ms
 *
 *  2. How getAngles() auto-computes dt from micros(), bootstraps from the
 *     accelerometer on the first call, and clamps stale dt values so a
 *     paused loop never causes a large spurious angle jump.
 *
 *  3. A "balance angle" concept: the robot is upright when pitch ≈ 0 deg
 *     (or some small trim offset).  This sketch prints a visual tilt bar
 *     and a balance status label alongside the numeric angles so you can
 *     physically tilt the robot and immediately see the filter respond.
 *
 *  4. Serial Plotter mode: define PLOTTER_MODE 1 to switch the output to
 *     a comma-separated format that Arduino Serial Plotter can graph.
 *
 *  5. Non-blocking loop: no delay() in the hot path; the loop runs as fast
 *     as the I2C bus allows so getAngles() always has the finest dt
 *     resolution.  A millis() gate throttles the print rate independently.
 *
 *  6. Filter re-initialisation guard: if the loop stalls for more than
 *     STALL_RESET_MS (e.g., you open a Serial dialog), resetFilter() is
 *     called so the filter re-bootstraps cleanly rather than integrating
 *     a giant dt spike.
 *
 * =========================================================================
 * HARDWARE SETUP
 * =========================================================================
 *   MPU-6050 VCC  ->  3.3 V
 *   MPU-6050 GND  ->  GND
 *   MPU-6050 SDA  ->  GPIO 21  (ESP32 default)
 *   MPU-6050 SCL  ->  GPIO 22  (ESP32 default)
 *   MPU-6050 AD0  ->  GND      (I2C address 0x68)
 *   MPU-6050 INT  ->  not connected
 *
 *   Mount the IMU so that:
 *     X-axis  points forward  (direction of travel)
 *     Y-axis  points left
 *     Z-axis  points up  (when the robot is upright)
 *
 *   If your mounting differs, swap or negate the axes in AXIS REMAPPING below.
 *
 * =========================================================================
 * CALIBRATION
 * =========================================================================
 *   Run examples/Calibration/Calibration.ino first.
 *   Paste the six offset values into USER_OFFSETS below.
 *   Gyro bias correction is especially important here: even a 1 LSB gyro
 *   bias integrates to ~0.46 deg/s of continuous drift at FSR_500DPS,
 *   which will slowly walk the pitch estimate away from true vertical.
 *
 * =========================================================================
 * BALANCE ANGLE TRIM
 * =========================================================================
 *   PITCH_TRIM_DEG compensates for the IMU not being mounted perfectly
 *   level on the robot chassis.  Set it so that the "Balance  OK" label
 *   appears when the robot is visually balanced and standing still.
 *   Typical range: -5 to +5 degrees.
 *
 * =========================================================================
 * SERIAL OUTPUT — TABLE MODE  (PLOTTER_MODE 0)
 * =========================================================================
 *   Complementary Filter — Self-Balancing Robot
 *   ============================================
 *   Alpha: 0.980   DLPF: 98Hz   FSR: +/-2g / +/-500dps
 *   Offsets: aX=  -142  aY=    58  aZ=  -310  gX=    27  gY=   -11  gZ=     4
 *   Loop rate target: 1000 Hz   Print rate: 50 Hz
 *   ------------------------------------------------------------------
 *    [ms]   Roll(deg)  Pitch(deg)  Tilt bar             Status
 *   ------------------------------------------------------------------
 *     1024     -0.23      1.47    ====[|]====          Balance  OK
 *     1044      0.11     -8.20    ======[|  ]          Tilting LEFT
 *     1064      0.08     12.55    ===  [|]====         Tilting RIGHT
 *
 * =========================================================================
 * SERIAL OUTPUT — PLOTTER MODE  (PLOTTER_MODE 1)
 * =========================================================================
 *   Roll:0.12,Pitch:1.47,PitchTrimmed:-0.03
 *   Roll:0.10,Pitch:1.45,PitchTrimmed:-0.05
 *
 * SPDX-License-Identifier: MIT
 */

#include <Arduino.h>
#include "MPU6050_ESP32.h"

// ============================================================================
// OUTPUT MODE
// ============================================================================

/**
 * 0 = human-readable table (Serial Monitor)
 * 1 = label:value,label:value CSV (Arduino Serial Plotter)
 *
 * Switch to 1 when you want to graph roll/pitch in real time.
 * The Serial Plotter opens from Tools -> Serial Plotter (Ctrl+Shift+L).
 */
#define PLOTTER_MODE  0

// ============================================================================
// HARDWARE PINS
// ============================================================================

static constexpr int      PIN_SDA  = 21;
static constexpr int      PIN_SCL  = 22;
static constexpr uint32_t I2C_FREQ = 400000UL;  // 400 kHz Fast Mode

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

/**
 * FSR_2G  gives the finest accel resolution for tilt sensing near vertical.
 * FSR_500DPS covers the angular rates a falling robot produces (~200 deg/s)
 * without saturating, while still offering twice the resolution of FSR_1000DPS.
 */
static constexpr AccelFSR ACCEL_FSR = AccelFSR::FSR_2G;
static constexpr GyroFSR  GYRO_FSR  = GyroFSR::FSR_500DPS;

/**
 * DLPF_2 (98 Hz) is the recommended setting for a self-balancing robot:
 *   - Passes tilt dynamics well below the motor PWM frequency (~10-20 kHz)
 *   - Rejects DC motor commutation noise transmitted through the chassis
 *   - Group delay ~2.8 ms — negligible relative to a 10 ms balance loop
 *
 * Use DLPF_1 (188 Hz) if you need faster transient response on a stiff robot.
 * Use DLPF_3 (42 Hz)  if the chassis resonates and you see angle oscillation.
 */
static constexpr DLPFMode DLPF_MODE = DLPFMode::DLPF_2;

// ============================================================================
// COMPLEMENTARY FILTER COEFFICIENT
// ============================================================================

/**
 * alpha blends gyro integration (fast, noisy) with accel tilt (slow, accurate).
 *
 *   angle = alpha * (angle + gyroRate * dt) + (1 - alpha) * accelAngle
 *
 * At alpha = 0.98 and a 1 ms loop period:
 *   - Gyro drift time constant = dt / (1 - alpha) = 0.001 / 0.02 = 50 ms
 *     (accel corrects any accumulated gyro drift within ~50 ms)
 *   - Accel noise is attenuated by factor (1 - alpha) = 0.02 per step
 *
 * Raise alpha (toward 1.0) if the angle is jittery but responds correctly.
 * Lower alpha (toward 0.95) if the angle drifts slowly and never self-corrects.
 *
 * For a fast-responding robot that falls in < 200 ms, do NOT go below 0.95 —
 * a low alpha lets accelerometer noise drive the angle estimate and the PID
 * will react to phantom tilts.
 */
static constexpr float CF_ALPHA = 0.98f;

// ============================================================================
// BALANCE ANGLE TRIM
// ============================================================================

/**
 * Mechanical offset in degrees added to pitch before balance evaluation.
 *
 * If the robot is physically balanced but the pitch angle reads +2.5 deg,
 * set PITCH_TRIM_DEG = -2.5 so the "Balance OK" window is centred on true
 * vertical.  Adjust by observing the pitch reading when the robot is
 * standing still and balanced by hand.
 *
 * This trim does NOT affect the raw pitch value printed in the table —
 * it only affects the trimmed pitch column and the status label, so you
 * can always see both the raw and corrected values simultaneously.
 */
static constexpr float PITCH_TRIM_DEG = 0.0f;

/**
 * Half-width of the "Balance OK" dead-band around zero trimmed pitch.
 * |trimmedPitch| < BALANCE_DEADBAND_DEG  ->  "Balance  OK"
 * Typical self-balancing robots use 1–3 degrees.
 */
static constexpr float BALANCE_DEADBAND_DEG = 2.0f;

// ============================================================================
// CALIBRATION OFFSETS
// ============================================================================
// Paste values from Calibration.ino here.  All zeros = no correction.

static const CalibrationOffsets USER_OFFSETS = {
    .accelX =   -516,
    .accelY =  -1142,
    .accelZ =    810,
    .gyroX  =    254,
    .gyroY  =     61,
    .gyroZ  =    133
};

// ============================================================================
// TIMING PARAMETERS
// ============================================================================

/**
 * How often to print or plot (milliseconds).
 *
 * Table mode (PLOTTER_MODE 0):  20 ms = 50 Hz — fast enough to watch the
 *   filter respond to motion without flooding the Serial Monitor.
 *
 * Plotter mode (PLOTTER_MODE 1): 10 ms = 100 Hz — smoother graph curves.
 *   The Serial Plotter can handle this rate easily.
 *
 * The getAngles() call itself runs on every loop iteration regardless of
 * this setting — the filter dt is always as fine as the I2C round-trip.
 */
#if PLOTTER_MODE
static constexpr uint32_t PRINT_INTERVAL_MS = 10;
#else
static constexpr uint32_t PRINT_INTERVAL_MS = 20;
#endif

/**
 * If the loop stalls for longer than this many milliseconds (e.g. while
 * you open the Serial Monitor or pause in a debugger), resetFilter() is
 * called before the next getAngles() so the filter re-bootstraps from the
 * accelerometer rather than integrating a huge dt spike.
 * 200 ms is chosen to be well above any normal Serial.print blocking but
 * well below the ~500 ms it would take a falling robot to tip past recovery.
 */
static constexpr uint32_t STALL_RESET_MS = 200;

/** How many table rows to print before reprinting the column header. */
static constexpr uint16_t HEADER_REPEAT_ROWS = 25;

// ============================================================================
// AXIS REMAPPING
// ============================================================================
/**
 * If your MPU-6050 is mounted in a non-standard orientation, adjust the
 * signs and axis assignments here rather than editing getAngles() internals.
 *
 * The standard orientation assumed by the driver's complementary filter is:
 *   X-axis  forward,  Y-axis  left,  Z-axis  up.
 *
 * Common remapping examples:
 *   Mounted upside-down        ->  negate accelZ and gyroX, gyroY
 *   Rotated 90 deg around Z    ->  swap X<->Y and negate one axis
 *
 * Because getAngles() uses the driver's internal filter, axis remapping
 * must happen BEFORE the filter runs.  The cleanest approach is to
 * post-process the Angles struct returned by getAngles():
 *
 *   angles.pitch = -angles.pitch;   // flip for upside-down mount
 *
 * For a self-balancing robot the PITCH axis is the critical one (forward/
 * backward tilt).  Verify the sign is correct: tilting the robot forward
 * (nose down) should INCREASE pitch.  If it decreases, negate it here.
 */
static constexpr float PITCH_SIGN = +1.0f;  // set to -1.0f to flip
static constexpr float ROLL_SIGN  = +1.0f;  // set to -1.0f to flip

// ============================================================================
// GLOBAL OBJECTS AND STATE
// ============================================================================

MPU6050_ESP32 imu(MPU6050_ADDR_LOW);

static uint32_t lastPrintMs   = 0;
static uint32_t lastLoopMs    = 0;  // used for stall detection
static uint32_t loopCount     = 0;  // getAngles() calls since last print
static uint32_t errorCount    = 0;  // failed getAngles() calls
static uint32_t errorPrev     = 0;  // snapshot for delta error reporting
static uint16_t rowsSinceHdr  = 0;

// Most recent fused angles — held between print intervals.
static Angles latestAngles = {0.0f, 0.0f};
// Flag: has at least one successful getAngles() call completed?
static bool    anglesValid  = false;

// ============================================================================
// TILT BAR  (table mode only)
// ============================================================================

/**
 * @brief Render a 20-character ASCII tilt bar centred on zero pitch.
 *
 * The bar shows the trimmed pitch angle as a bracket [|] whose position
 * slides left or right within a fixed-width field.  The centre of the bar
 * represents 0 deg (upright).  The full scale of the bar is +/-TILT_BAR_SCALE.
 *
 * Examples:
 *   pitch =  0.0 deg  ->   "==========[|]========="   (centred)
 *   pitch = +5.0 deg  ->   "============[|]=======  " (shifted right)
 *   pitch = -5.0 deg  ->   "  =======[|]==========" (shifted left)
 *
 * @param trimmedPitch  Pitch after PITCH_TRIM_DEG correction (degrees).
 */
static void printTiltBar(float trimmedPitch)
{
    // Maximum pitch mapped to the edge of the bar (degrees).
    static constexpr float TILT_BAR_SCALE = 15.0f;
    // Total width of the bar in characters (excluding bracket chars).
    static constexpr uint8_t BAR_WIDTH = 20;
    // Width of one side (half the bar).
    static constexpr uint8_t HALF = BAR_WIDTH / 2;

    // Clamp and map pitch to a position in [-HALF, +HALF].
    float clamped = trimmedPitch;
    if (clamped >  TILT_BAR_SCALE) clamped =  TILT_BAR_SCALE;
    if (clamped < -TILT_BAR_SCALE) clamped = -TILT_BAR_SCALE;

    // cursor: 0 = left edge, HALF = centre (upright), BAR_WIDTH = right edge.
    // We convert pitch to an integer column position.
    // Positive pitch -> robot tilting right -> bracket moves right of centre.
    const int8_t offset   = static_cast<int8_t>((clamped / TILT_BAR_SCALE) * HALF + 0.5f);
    const int8_t cursorPos = static_cast<int8_t>(HALF + offset);  // 0..BAR_WIDTH

    // Print the bar character by character.
    Serial.print(' ');
    for (uint8_t col = 0; col <= BAR_WIDTH + 1; ++col) {
        if (col == static_cast<uint8_t>(cursorPos)) {
            Serial.print(F("[|]"));
            col += 2;   // bracket occupies 3 chars; skip the next two
        } else {
            const bool inBar = (col > 0) && (col < BAR_WIDTH + 1);
            Serial.print(inBar ? '=' : ' ');
        }
    }
    Serial.print(' ');
}

// ============================================================================
// HELPERS: header and data row  (table mode)
// ============================================================================

static void printHeader()
{
    Serial.println(F("------------------------------------------------------------------"));
    Serial.println(F("   [ms]   Roll(deg)  Pitch(deg)  Tilt bar              Status"));
    Serial.println(F("------------------------------------------------------------------"));
    Serial.flush();
}

/**
 * @brief Print one data row in table mode.
 *
 * @param angles     Fused roll/pitch from getAngles(), with sign applied.
 * @param loopsHz    Approximate getAngles() call rate in the last interval.
 */
static void printTableRow(const Angles &angles, uint32_t loopsHz)
{
    const float trimmedPitch = PITCH_SIGN * angles.pitch + PITCH_TRIM_DEG;
    const float roll         = ROLL_SIGN  * angles.roll;

    // Timestamp + angles
    Serial.printf("%7lu  %9.2f  %10.2f  ",
                  millis(),
                  static_cast<double>(roll),
                  static_cast<double>(PITCH_SIGN * angles.pitch));

    // Visual tilt bar
    printTiltBar(trimmedPitch);

    // Status label
    if (fabsf(trimmedPitch) < BALANCE_DEADBAND_DEG) {
        Serial.print(F("  Balance  OK"));
    } else if (trimmedPitch > 0.0f) {
        Serial.print(F("  Tilting RIGHT"));
    } else {
        Serial.print(F("  Tilting LEFT "));
    }

    // Append effective loop rate in a narrow column (useful for tuning dt).
    Serial.printf("  [%4lu Hz]", loopsHz);

    Serial.println();
}

// ============================================================================
// HELPER: plotter row
// ============================================================================

/**
 * @brief Print one line in label:value CSV format for Serial Plotter.
 *
 * Arduino Serial Plotter expects either bare comma-separated numbers or the
 * "label:value" format (IDE 2.x).  Using labels keeps the legend readable.
 *
 * Channels:
 *   Roll         — roll angle in degrees
 *   Pitch        — raw pitch from the filter
 *   PitchTrimmed — pitch after PITCH_TRIM_DEG correction
 *   DeadbandHi   — +BALANCE_DEADBAND_DEG reference line
 *   DeadbandLo   — -BALANCE_DEADBAND_DEG reference line
 *
 * The two deadband lines appear as constant horizontal guides in the plotter
 * so you can immediately see when the pitch leaves the balance window.
 *
 * @param angles  Fused angles from getAngles().
 */
static void printPlotterRow(const Angles &angles)
{
    const float pitchRaw     = PITCH_SIGN * angles.pitch;
    const float pitchTrimmed = pitchRaw + PITCH_TRIM_DEG;
    const float rollFinal    = ROLL_SIGN  * angles.roll;

    Serial.printf(
        "Roll:%.2f,Pitch:%.2f,PitchTrimmed:%.2f,DeadbandHi:%.2f,DeadbandLo:%.2f\n",
        static_cast<double>(rollFinal),
        static_cast<double>(pitchRaw),
        static_cast<double>(pitchTrimmed),
        static_cast<double>( BALANCE_DEADBAND_DEG),
        static_cast<double>(-BALANCE_DEADBAND_DEG)
    );
}

// ============================================================================
// HELPER: printConfig()  (table mode only)
// ============================================================================

static void printConfig()
{
    // Alpha and filter info
    Serial.printf("Alpha: %.3f   ", static_cast<double>(CF_ALPHA));

    Serial.print(F("DLPF: "));
    switch (DLPF_MODE) {
        case DLPFMode::DLPF_0: Serial.print(F("disabled")); break;
        case DLPFMode::DLPF_1: Serial.print(F("188Hz"));    break;
        case DLPFMode::DLPF_2: Serial.print(F("98Hz"));     break;
        case DLPFMode::DLPF_3: Serial.print(F("42Hz"));     break;
        case DLPFMode::DLPF_4: Serial.print(F("20Hz"));     break;
        case DLPFMode::DLPF_5: Serial.print(F("10Hz"));     break;
        case DLPFMode::DLPF_6: Serial.print(F("5Hz"));      break;
    }

    Serial.print(F("   FSR: "));
    switch (ACCEL_FSR) {
        case AccelFSR::FSR_2G:  Serial.print(F("+/-2g"));  break;
        case AccelFSR::FSR_4G:  Serial.print(F("+/-4g"));  break;
        case AccelFSR::FSR_8G:  Serial.print(F("+/-8g"));  break;
        case AccelFSR::FSR_16G: Serial.print(F("+/-16g")); break;
    }
    Serial.print(F(" / "));
    switch (GYRO_FSR) {
        case GyroFSR::FSR_250DPS:  Serial.print(F("+/-250dps"));  break;
        case GyroFSR::FSR_500DPS:  Serial.print(F("+/-500dps"));  break;
        case GyroFSR::FSR_1000DPS: Serial.print(F("+/-1000dps")); break;
        case GyroFSR::FSR_2000DPS: Serial.print(F("+/-2000dps")); break;
    }
    Serial.println();

    // Offsets
    const bool hasOffsets = (USER_OFFSETS.accelX | USER_OFFSETS.accelY |
                             USER_OFFSETS.accelZ | USER_OFFSETS.gyroX  |
                             USER_OFFSETS.gyroY  | USER_OFFSETS.gyroZ) != 0;
    Serial.printf(
        "Offsets  :  aX=%5d  aY=%5d  aZ=%5d  gX=%5d  gY=%5d  gZ=%5d",
        static_cast<int>(USER_OFFSETS.accelX),
        static_cast<int>(USER_OFFSETS.accelY),
        static_cast<int>(USER_OFFSETS.accelZ),
        static_cast<int>(USER_OFFSETS.gyroX),
        static_cast<int>(USER_OFFSETS.gyroY),
        static_cast<int>(USER_OFFSETS.gyroZ)
    );
    if (!hasOffsets) {
        Serial.print(F("  <-- zeros: run Calibration.ino for best results"));
    }
    Serial.println();

    // Trim and deadband
    Serial.printf("Pitch trim: %.2f deg   Balance deadband: +/-%.2f deg\n",
                  static_cast<double>(PITCH_TRIM_DEG),
                  static_cast<double>(BALANCE_DEADBAND_DEG));

    // Loop rate
    Serial.printf("Print rate: %lu Hz   Stall reset threshold: %lu ms\n",
                  1000UL / PRINT_INTERVAL_MS,
                  STALL_RESET_MS);
}

// ============================================================================
// setup()
// ============================================================================

void setup()
{
    Serial.begin(115200);
    delay(2000);   // allow USB-UART to enumerate; Serial Monitor to open

#if !PLOTTER_MODE
    Serial.println();
    Serial.println(F("Complementary Filter -- Self-Balancing Robot"));
    Serial.println(F("============================================"));
    Serial.flush();
#endif

    // ---- 1. Initialise I2C and wake the MPU-6050 ----------------------------
    if (!imu.begin(PIN_SDA, PIN_SCL, I2C_FREQ)) {
        Serial.println(F("ERROR: MPU-6050 not found."));
        Serial.println(F("Check: SDA->GPIO21  SCL->GPIO22  VCC->3.3V  AD0->GND"));
        Serial.flush();
        while (true) { delay(1000); }
    }

    // ---- 2. Full-scale range ------------------------------------------------
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

    // ---- 3. DLPF bandwidth --------------------------------------------------
    if (!imu.setDLPF(DLPF_MODE)) {
        Serial.println(F("ERROR: setDLPF() failed. Halting."));
        Serial.flush();
        while (true) { delay(1000); }
    }

    // ---- 4. Calibration offsets ---------------------------------------------
    imu.setCalibrationOffsets(USER_OFFSETS);

    // ---- 5. Print config and initial header (table mode only) ---------------
#if !PLOTTER_MODE
    printConfig();
    printHeader();
    Serial.flush();
#endif

    lastPrintMs = millis();
    lastLoopMs  = millis();
}

// ============================================================================
// loop()
// ============================================================================

void loop()
{
    const uint32_t nowMs = millis();

    // -------------------------------------------------------------------------
    // Stall detection: if the loop has not run getAngles() for STALL_RESET_MS,
    // the filter's internal dt accumulator will be stale.  Rather than letting
    // a huge dt spike cause a large angle jump on the next call, we explicitly
    // reset the filter so it re-bootstraps cleanly from the accelerometer.
    // -------------------------------------------------------------------------
    if (nowMs - lastLoopMs > STALL_RESET_MS) {
        imu.resetFilter();
        // Also reset the loop timing so the first post-stall print interval
        // does not fire immediately.
        lastPrintMs = nowMs;

#if !PLOTTER_MODE
        Serial.printf("\n[%7lu ms] Filter reset after stall (> %lu ms idle).\n",
                      nowMs, STALL_RESET_MS);
        Serial.flush();
#endif
    }
    lastLoopMs = nowMs;

    // -------------------------------------------------------------------------
    // Call getAngles() on every loop iteration.
    //
    // Internally this:
    //   1. Calls getScaledData() — one 14-byte I2C burst (~200 µs at 400 kHz).
    //   2. Computes accel-derived roll and pitch via atan2.
    //   3. Computes dt = micros() - lastUpdateUs (microsecond resolution).
    //   4. Applies: angle = alpha*(angle + gyroRate*dt) + (1-alpha)*accelAngle
    //   5. Returns updated roll and pitch in degrees.
    //
    // Running this every iteration (not just at print time) gives the finest
    // possible dt and the smoothest angle tracking.  The print gate below
    // controls how often the result appears on Serial — independently.
    // -------------------------------------------------------------------------
    Angles freshAngles;
    if (imu.getAngles(freshAngles, CF_ALPHA)) {
        latestAngles = freshAngles;
        anglesValid  = true;
        ++loopCount;
    } else {
        ++errorCount;
    }

    // -------------------------------------------------------------------------
    // Timed print gate
    // -------------------------------------------------------------------------
    if (nowMs - lastPrintMs < PRINT_INTERVAL_MS) {
        return;
    }

    // Compute effective loop rate over the last interval for diagnostics.
    const uint32_t elapsedMs  = nowMs - lastPrintMs;
    const uint32_t effectiveHz = (elapsedMs > 0)
                                 ? (loopCount * 1000UL) / elapsedMs
                                 : 0;
    loopCount   = 0;
    lastPrintMs = nowMs;

    // ---- Report new I2C errors ----------------------------------------------
    if (errorCount > errorPrev) {
#if !PLOTTER_MODE
        Serial.printf("  *** %lu I2C error(s) in last interval (total: %lu) ***\n",
                      errorCount - errorPrev, errorCount);
#endif
        errorPrev = errorCount;
    }

    // ---- Print data ---------------------------------------------------------
    if (!anglesValid) {
        // No successful read yet — should only happen if begin() succeeded
        // but every subsequent getAngles() call failed (very unusual).
#if !PLOTTER_MODE
        Serial.println(F("  Waiting for first valid angle reading..."));
#endif
        return;
    }

#if PLOTTER_MODE
    printPlotterRow(latestAngles);
#else
    printTableRow(latestAngles, effectiveHz);

    // Reprint the column header periodically so it stays on screen.
    ++rowsSinceHdr;
    if (rowsSinceHdr >= HEADER_REPEAT_ROWS) {
        printHeader();
        rowsSinceHdr = 0;
    }
#endif
}
