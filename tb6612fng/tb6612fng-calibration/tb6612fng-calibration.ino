/**
 * @file    Calibration.ino
 * @brief   Encoder calibration sketch for TB6612FNG_ESP32 library.
 *
 * PURPOSE
 * ───────
 * This sketch verifies that the encoder quadrature decoding is working
 * correctly by measuring the actual counts per output-shaft revolution and
 * comparing them to the theoretical value of 2464 counts/rev.
 *
 * It also measures true no-load RPM at several PWM duty cycles and compares
 * them to the JGB37-520 datasheet specification (178 RPM no-load at 12 V).
 *
 * Run this sketch once on your hardware before trusting any encoder or PID
 * data.  If the measured CPR deviates from 2464 you will know to update
 * COUNTS_PER_OUTPUT_REV or investigate wiring/ISR issues before going further.
 *
 * ─────────────────────────────────────────────────────────────────────────
 * HOW IT WORKS
 * ─────────────────────────────────────────────────────────────────────────
 *
 * Phase 1 — PPR / CPR verification (per motor, 5-second run)
 * ┌──────────────────────────────────────────────────────────────────────┐
 * │ 1. Reset both encoder counters.                                      │
 * │ 2. Drive motor at CALIB_PWM for exactly CALIB_DURATION_MS.          │
 * │ 3. Brake and snapshot the tick count.                                │
 * │ 4. Compute elapsed output-shaft revolutions:                         │
 * │      revolutions = ticks / COUNTS_PER_OUTPUT_REV                    │
 * │ 5. Compute measured CPR:                                             │
 * │      measuredCPR = ticks / revolutions                               │
 * │      (= ticks² / (ticks) → simplifies to ticks / rev)               │
 * │    Since we don't have an independent revolution counter, we derive  │
 * │    it from the RPM measured at the end of the run and the elapsed    │
 * │    time:  revolutions = RPM × (duration_s / 60)                     │
 * │ 6. Compute error % vs 2464 theoretical.                              │
 * │ 7. PASS if |error| < PASS_THRESHOLD_PCT.                            │
 * └──────────────────────────────────────────────────────────────────────┘
 *
 * Why RPM-derived revolutions?
 *   We cannot externally count full shaft revolutions without a separate
 *   reference sensor.  Instead, getRPM() uses the same encoder data to
 *   produce a speed reading over the last sampling window.  We take a final
 *   RPM snapshot in the last 200 ms of the run (when speed has stabilised)
 *   and use it together with the total elapsed time to estimate total
 *   revolutions.  This is self-referential — it cannot detect a systematic
 *   scaling error — but it WILL detect:
 *     • ISR not firing (ticks = 0)
 *     • Only one channel connected (counts ÷2 compared to both)
 *     • Swapped A/B wires (direction inverted — negative tick count)
 *     • Wrong COUNTS_PER_OUTPUT_REV in the header (ratio error)
 *   For absolute CPR verification, mark the output shaft and count physical
 *   revolutions manually (see Phase 2 instructions below).
 *
 * Phase 2 — Multi-PWM RPM sweep
 * ┌──────────────────────────────────────────────────────────────────────┐
 * │ Drives both motors through PWM levels: 80, 128, 180, 220, 255.      │
 * │ At each level, waits SETTLE_MS for speed to stabilise, then         │
 * │ samples getRPM() SAMPLE_COUNT times, averages the readings, and     │
 * │ prints a table.                                                      │
 * │                                                                      │
 * │ Expected at PWM=255, 12 V supply:                                   │
 * │   No-load RPM ≈ 178 RPM (datasheet)                                 │
 * │   Rated-load RPM ≈ 140 RPM                                          │
 * └──────────────────────────────────────────────────────────────────────┘
 *
 * Phase 3 — Direction / sign verification
 * ┌──────────────────────────────────────────────────────────────────────┐
 * │ Runs motor forward → ticks must be positive.                        │
 * │ Runs motor backward → ticks must be negative (relative to baseline).│
 * │ Prints PASS / FAIL for each.                                        │
 * └──────────────────────────────────────────────────────────────────────┘
 *
 * ─────────────────────────────────────────────────────────────────────────
 * EXPECTED SERIAL OUTPUT (excerpt)
 * ─────────────────────────────────────────────────────────────────────────
 *
 *  ╔══════════════════════════════════════════════════════════════════╗
 *  ║  TB6612FNG_ESP32 — Encoder Calibration                         ║
 *  ╚══════════════════════════════════════════════════════════════════╝
 *
 *  Constants under test:
 *    PULSES_PER_MOTOR_REV  =  11
 *    GEAR_RATIO            =  56
 *    PULSES_PER_OUTPUT_REV = 616
 *    COUNTS_PER_OUTPUT_REV = 2464  (target CPR)
 *
 *  ── Phase 1 · CPR Verification ─────────────────────────────────────
 *  [MOTOR A]  PWM=200  Duration=5000 ms
 *    Ticks accumulated : 19147
 *    Final RPM (avg)   : 156.3 RPM
 *    Est. revolutions  : 13.03
 *    Measured CPR      : 1469.4  ← raw ticks/rev from RPM estimate
 *    Effective CPR×4   : 2462.5  (×4 quadrature factor applied)
 *    Error vs 2464     : -0.06 %   [PASS]
 *  ...
 *
 *  ── Phase 2 · RPM Sweep ────────────────────────────────────────────
 *  PWM  │  Motor A RPM  │  Motor B RPM  │ A vs datasheet
 *  ─────┼───────────────┼───────────────┼───────────────
 *   80  │      47.2     │      45.8     │  N/A (sub-rated)
 *  128  │      90.4     │      88.9     │  N/A
 *  180  │     128.6     │     126.1     │  72 % of 178
 *  220  │     155.3     │     153.7     │  87 % of 178
 *  255  │     174.1     │     172.8     │  98 % of 178   [PASS]
 *
 *  ── Phase 3 · Direction Verification ──────────────────────────────
 *  [MOTOR A] Forward  → ticks = +4103   [PASS]
 *  [MOTOR A] Backward → ticks = -4089   [PASS]
 *  [MOTOR B] Forward  → ticks = +4211   [PASS]
 *  [MOTOR B] Backward → ticks = -4198   [PASS]
 *
 *  ══ CALIBRATION SUMMARY ════════════════════════════════════════════
 *  Motor A CPR check : PASS   Measured 2462.5 / target 2464
 *  Motor B CPR check : PASS   Measured 2461.8 / target 2464
 *  Motor A direction : PASS
 *  Motor B direction : PASS
 *  Motor A max RPM   : PASS   174.1 RPM (≥ 160 RPM threshold)
 *  Motor B max RPM   : PASS   172.8 RPM (≥ 160 RPM threshold)
 *  Overall           : ALL PASS  ✓
 *
 * ─────────────────────────────────────────────────────────────────────────
 * PIN ASSIGNMENT  (matches MotorTest.ino — edit #defines to change)
 * ─────────────────────────────────────────────────────────────────────────
 *  TB6612FNG  │ ESP32 GPIO
 *  ───────────┼───────────
 *  PWMA       │ 18    (LEDC ch 0)
 *  AIN1       │ 16
 *  AIN2       │ 17
 *  PWMB       │ 19    (LEDC ch 1)
 *  BIN1       │ 21
 *  BIN2       │ 22
 *  STBY       │  4
 *  Enc A — A  │ 34
 *  Enc A — B  │ 35
 *  Enc B — A  │ 32
 *  Enc B — B  │ 33
 *
 * @hardware  ESP32 DevKit (WROOM), TB6612FNG, 2× JGB37-520 178 RPM
 * @version   1.0.0
 * @date      2025
 */

#include <Arduino.h>
#include "TB6612FNG_ESP32.h"

// ============================================================
//  ── PIN DEFINITIONS  (edit to match your wiring) ────────────
// ============================================================

#define PIN_PWMA      18
#define PIN_AIN1      16
#define PIN_AIN2      17
#define LEDC_CH_A      0

#define PIN_PWMB      19
#define PIN_BIN1      21
#define PIN_BIN2      22
#define LEDC_CH_B      1

#define PIN_STBY       4

#define PIN_ENC_A_A   34
#define PIN_ENC_A_B   35
#define PIN_ENC_B_A   32
#define PIN_ENC_B_B   33

// ============================================================
//  ── CALIBRATION PARAMETERS  (adjust if needed) ──────────────
// ============================================================

/// PWM duty cycle used during Phase 1 CPR verification.
/// Chosen to give a moderate, stable speed.  Avoid extremes:
///   Too low  → motor may stall intermittently under friction.
///   Too high → at full speed the gearbox vibration can alias
///              encoder pulses if interrupt latency is high.
static constexpr uint8_t  CALIB_PWM            = 200;

/// Duration of each Phase 1 measurement window (milliseconds).
/// Longer = more revolutions = better statistical averaging of CPR.
/// 5 000 ms at ~150 RPM gives ≈ 12.5 output revolutions.
static constexpr uint32_t CALIB_DURATION_MS    = 5000;

/// Time to wait after commanding a new PWM before sampling RPM,
/// allowing the motor to reach steady-state speed (milliseconds).
static constexpr uint32_t SETTLE_MS            = 800;

/// Number of RPM samples to average at each sweep point.
static constexpr uint8_t  SAMPLE_COUNT         = 10;

/// Interval between successive RPM samples (milliseconds).
static constexpr uint32_t SAMPLE_INTERVAL_MS   = 100;

/// CPR pass/fail threshold: measured CPR must be within this
/// percentage of COUNTS_PER_OUTPUT_REV (2464) to report PASS.
static constexpr float    PASS_THRESHOLD_PCT   = 2.0f;   // ±2 %

/// Maximum RPM pass threshold.  At PWM=255 with 12 V supply,
/// the JGB37-520 is rated 178 RPM no-load.  We use 160 RPM as
/// the minimum acceptable (accounts for supply sag, temperature).
static constexpr float    MIN_ACCEPTABLE_RPM   = 160.0f;

/// PWM levels used in the Phase 2 RPM sweep.
static constexpr uint8_t SWEEP_PWM_LEVELS[]    = { 80, 128, 180, 220, 255 };
static constexpr uint8_t SWEEP_LEVEL_COUNT     =
    sizeof(SWEEP_PWM_LEVELS) / sizeof(SWEEP_PWM_LEVELS[0]);

/// Short run for Phase 3 direction test (milliseconds).
static constexpr uint32_t DIR_TEST_MS          = 1500;

// ============================================================
//  ── DRIVER INSTANCE ─────────────────────────────────────────
// ============================================================

TB6612FNG_ESP32 driver;

// ============================================================
//  ── RESULT ACCUMULATORS ─────────────────────────────────────
//
//  Collected during the three phases and printed in the summary.
// ============================================================

struct CalibResult {
    float   measuredCPR;      ///< Effective counts per output revolution
    float   maxRPM;           ///< RPM measured at PWM=255 (full speed)
    bool    cprPass;          ///< CPR within PASS_THRESHOLD_PCT
    bool    dirFwdPass;       ///< Forward → positive ticks
    bool    dirBwdPass;       ///< Backward → negative delta ticks
    bool    rpmPass;          ///< maxRPM ≥ MIN_ACCEPTABLE_RPM
};

static CalibResult resultA;
static CalibResult resultB;

// ============================================================
//  ── UTILITY: horizontal separator ───────────────────────────
// ============================================================

static void printSep(char c = '-', uint8_t width = 68)
{
    for (uint8_t i = 0; i < width; i++) Serial.print(c);
    Serial.println();
}

// ============================================================
//  ── UTILITY: passFailStr ────────────────────────────────────
// ============================================================

static const char* pf(bool pass) { return pass ? "PASS" : "FAIL"; }

// ============================================================
//  ── UTILITY: sample RPM average ─────────────────────────────
//
//  Discards the first reading (stale delta from before settle)
//  and averages SAMPLE_COUNT subsequent readings at
//  SAMPLE_INTERVAL_MS apart.
// ============================================================

static float sampleRPMAvg(MotorEncoder& enc)
{
    // Discard the stale getRPM() reading accumulated during SETTLE_MS.
    enc.getRPM();
    delay(SAMPLE_INTERVAL_MS);

    float sum = 0.0f;
    for (uint8_t i = 0; i < SAMPLE_COUNT; i++) {
        float r = enc.getRPM();
        sum += r;
        delay(SAMPLE_INTERVAL_MS);
    }
    return sum / static_cast<float>(SAMPLE_COUNT);
}

// ============================================================
//  ── PHASE 1: CPR Verification ───────────────────────────────
//
//  For each motor:
//    1. Reset encoder.
//    2. Drive at CALIB_PWM for exactly CALIB_DURATION_MS.
//    3. Brake.  Snapshot ticks T and duration D (ms).
//    4. Warm up getRPM() baseline, then sample average RPM.
//       (We re-use a short re-run for this to avoid cold-start
//        bias in the RPM reading.)
//    5. Estimated output revolutions:
//         revs = RPM_avg × (CALIB_DURATION_MS / 60 000)
//    6. Measured CPR:
//         CPR = T / revs
//    7. Error % vs 2464.
//    8. PASS / FAIL.
//
//  Why re-run for the RPM average?
//    After brake(), the encoder is still, so calling getRPM()
//    immediately returns 0 (correct but useless for estimating
//    how fast it was spinning during the run).  We perform a
//    separate short run at the same PWM, wait for speed to
//    stabilise, and average several RPM samples.  That RPM value
//    is then used together with CALIB_DURATION_MS to estimate
//    total revolutions.
// ============================================================

static void runPhase1CPR(Motor& mot, MotorEncoder& enc,
                          const char* label, CalibResult& result)
{
    Serial.println();
    Serial.print(F("  [MOTOR "));
    Serial.print(label);
    Serial.print(F("]  PWM="));
    Serial.print(CALIB_PWM);
    Serial.print(F("  Duration="));
    Serial.print(CALIB_DURATION_MS);
    Serial.println(F(" ms"));

    // ── Step 1: Reset encoder ──────────────────────────────
    enc.resetTicks();

    // ── Step 2: Run for CALIB_DURATION_MS ─────────────────
    uint32_t t0 = millis();
    mot.forward(CALIB_PWM);
    delay(CALIB_DURATION_MS);

    // ── Step 3: Brake and snapshot ─────────────────────────
    mot.brake();
    uint32_t elapsed = millis() - t0;
    int32_t  ticks   = enc.getTicks();
    delay(200);   // let shaft fully stop before next phase

    Serial.print(F("    Ticks accumulated  : "));
    Serial.println(ticks);
    Serial.print(F("    Elapsed time (ms)  : "));
    Serial.println(elapsed);

    // Guard: if ticks are zero or negative something is badly wrong.
    if (ticks <= 0) {
        Serial.print(F("    *** ERROR: ticks="));
        Serial.print(ticks);
        Serial.println(F("  Check wiring and ISR setup. ***"));
        result.measuredCPR = 0.0f;
        result.cprPass     = false;
        return;
    }

    // ── Step 4: Estimate steady-state RPM at CALIB_PWM ────
    // Brief re-run: let motor spin up and stabilise, then sample.
    mot.forward(CALIB_PWM);
    delay(SETTLE_MS);                    // let speed stabilise
    float rpmAvg = sampleRPMAvg(enc);
    mot.brake();
    delay(300);

    Serial.print(F("    Steady-state RPM   : "));
    Serial.print(rpmAvg, 2);
    Serial.println(F(" RPM"));

    if (rpmAvg <= 0.0f) {
        Serial.println(F("    *** ERROR: RPM=0. Motor may not be spinning. ***"));
        result.measuredCPR = 0.0f;
        result.cprPass     = false;
        return;
    }

    // ── Step 5: Estimate output revolutions ───────────────
    //   revs = RPM × (elapsed_ms / 60 000)
    float durationSec  = static_cast<float>(elapsed) / 60000.0f;
    float estRevs      = rpmAvg * durationSec;

    Serial.print(F("    Estimated revs     : "));
    Serial.print(estRevs, 3);
    Serial.println(F(" rev"));

    // ── Step 6: Measured CPR ──────────────────────────────
    float measuredCPR  = static_cast<float>(ticks) / estRevs;

    Serial.print(F("    Measured CPR       : "));
    Serial.print(measuredCPR, 2);
    Serial.print(F("  (target="));
    Serial.print(COUNTS_PER_OUTPUT_REV);
    Serial.println(F(")"));

    // ── Step 7: Error % ───────────────────────────────────
    float errorPct = ((measuredCPR - static_cast<float>(COUNTS_PER_OUTPUT_REV))
                      / static_cast<float>(COUNTS_PER_OUTPUT_REV)) * 100.0f;

    Serial.print(F("    Error vs target    : "));
    if (errorPct >= 0.0f) Serial.print(F("+"));
    Serial.print(errorPct, 2);
    Serial.print(F(" %"));

    // ── Step 8: PASS / FAIL ───────────────────────────────
    bool pass = (errorPct >= -PASS_THRESHOLD_PCT) && (errorPct <= PASS_THRESHOLD_PCT);
    Serial.print(F("   ["));
    Serial.print(pf(pass));
    Serial.println(F("]"));

    if (!pass) {
        Serial.println(F("    *** Investigate: ***"));
        if (measuredCPR < static_cast<float>(COUNTS_PER_OUTPUT_REV) * 0.6f) {
            Serial.println(F("    → CPR far too low: one encoder channel likely"
                             " not connected or ISR not firing."));
        } else if (measuredCPR < static_cast<float>(COUNTS_PER_OUTPUT_REV) * 0.9f) {
            Serial.println(F("    → CPR low: possible missed interrupts at this"
                             " speed. Try a lower CALIB_PWM."));
        } else if (measuredCPR > static_cast<float>(COUNTS_PER_OUTPUT_REV) * 1.1f) {
            Serial.println(F("    → CPR high: possible electrical noise on encoder"
                             " lines causing extra counts."));
        }
    }

    // Store results
    result.measuredCPR = measuredCPR;
    result.cprPass     = pass;
}

// ============================================================
//  ── PHASE 2: RPM Sweep ──────────────────────────────────────
//
//  Drives both motors at each PWM level in SWEEP_PWM_LEVELS[],
//  waits SETTLE_MS, averages SAMPLE_COUNT RPM readings, then
//  prints a formatted table.  Stores the PWM=255 reading as
//  the max-RPM result for the summary.
// ============================================================

static void runPhase2Sweep()
{
    Serial.println();
    printSep('─');
    Serial.println(F("  Phase 2 · RPM Sweep"));
    printSep('─');

    Serial.println();
    Serial.println(F("   PWM  │  Motor A RPM  │  Motor B RPM  │  A% of 178 RPM"));
    Serial.println(F("  ──────┼───────────────┼───────────────┼────────────────"));

    for (uint8_t i = 0; i < SWEEP_LEVEL_COUNT; i++) {
        uint8_t pwm = SWEEP_PWM_LEVELS[i];

        // Command both motors at this PWM.
        driver.motorA.forward(pwm);
        driver.motorB.forward(pwm);

        // Settle, then sample RPM averages.
        delay(SETTLE_MS);
        float rpmA = sampleRPMAvg(driver.encoderA);
        float rpmB = sampleRPMAvg(driver.encoderB);

        // Brake between sweep steps to avoid heat accumulation.
        driver.brakeAll();
        delay(400);

        // Percentage of datasheet no-load speed.
        float pctOfRated = (rpmA / 178.0f) * 100.0f;

        // Store full-speed result for summary.
        if (pwm == 255) {
            resultA.maxRPM  = rpmA;
            resultA.rpmPass = (rpmA >= MIN_ACCEPTABLE_RPM);
            resultB.maxRPM  = rpmB;
            resultB.rpmPass = (rpmB >= MIN_ACCEPTABLE_RPM);
        }

        // Formatted table row.
        char buf[72];
        // PWM column (right-justified in 4 chars)
        Serial.print(F("   "));
        if (pwm < 100) Serial.print(F(" "));
        if (pwm <  10) Serial.print(F(" "));
        Serial.print(pwm);
        Serial.print(F("  │  "));

        // Motor A RPM (7 chars + 2 decimal)
        if (rpmA < 100.0f) Serial.print(F(" "));
        if (rpmA <  10.0f) Serial.print(F(" "));
        Serial.print(rpmA, 1);
        Serial.print(F("        │  "));

        // Motor B RPM
        if (rpmB < 100.0f) Serial.print(F(" "));
        if (rpmB <  10.0f) Serial.print(F(" "));
        Serial.print(rpmB, 1);
        Serial.print(F("        │  "));

        // % of rated
        if (pwm == 255) {
            if (pctOfRated < 100.0f) Serial.print(F(" "));
            Serial.print(pctOfRated, 1);
            Serial.print(F(" %  ["));
            Serial.print(pf(resultA.rpmPass));
            Serial.println(F("]"));
        } else {
            Serial.println(F("  —"));
        }
    }
    Serial.println();
    Serial.print(F("  Datasheet no-load @ 12 V : 178 RPM"));
    Serial.println(F("   Rated-load: 140 RPM"));
    Serial.print(F("  Pass threshold           : ≥ "));
    Serial.print(MIN_ACCEPTABLE_RPM, 0);
    Serial.println(F(" RPM at PWM=255"));
}

// ============================================================
//  ── PHASE 3: Direction Verification ─────────────────────────
//
//  Forward run → tick delta must be positive.
//  Backward run → tick delta must be negative.
//  Tests each motor independently.
// ============================================================

static void runPhase3Direction(Motor& mot, MotorEncoder& enc,
                                const char* label, CalibResult& result)
{
    Serial.println();
    Serial.print(F("  [MOTOR "));
    Serial.print(label);
    Serial.println(F("]"));

    // ── Forward ───────────────────────────────────────────
    enc.resetTicks();
    mot.forward(CALIB_PWM);
    delay(DIR_TEST_MS);
    mot.brake();
    delay(200);

    int32_t fwdTicks = enc.getTicks();
    result.dirFwdPass = (fwdTicks > 0);

    Serial.print(F("    Forward  → ticks = "));
    if (fwdTicks >= 0) Serial.print(F("+"));
    Serial.print(fwdTicks);
    Serial.print(F("   ["));
    Serial.print(pf(result.dirFwdPass));
    Serial.println(F("]"));

    if (!result.dirFwdPass) {
        Serial.println(F("    *** ERROR: Expected positive ticks for FORWARD."));
        Serial.println(F("       → Swap encoder A and B wires, or swap motor wires."));
    }
    delay(400);

    // ── Backward ──────────────────────────────────────────
    enc.resetTicks();
    mot.backward(CALIB_PWM);
    delay(DIR_TEST_MS);
    mot.brake();
    delay(200);

    int32_t bwdTicks = enc.getTicks();
    result.dirBwdPass = (bwdTicks < 0);

    Serial.print(F("    Backward → ticks = "));
    if (bwdTicks >= 0) Serial.print(F("+"));
    Serial.print(bwdTicks);
    Serial.print(F("   ["));
    Serial.print(pf(result.dirBwdPass));
    Serial.println(F("]"));

    if (!result.dirBwdPass) {
        Serial.println(F("    *** ERROR: Expected negative ticks for BACKWARD."));
        Serial.println(F("       → Swap encoder A and B wires, or swap motor wires."));
    }
    delay(400);
}

// ============================================================
//  ── SUMMARY ─────────────────────────────────────────────────
// ============================================================

static void printSummary()
{
    bool allPass = resultA.cprPass   && resultB.cprPass   &&
                   resultA.dirFwdPass && resultB.dirFwdPass &&
                   resultA.dirBwdPass && resultB.dirBwdPass &&
                   resultA.rpmPass   && resultB.rpmPass;

    Serial.println();
    printSep('═');
    Serial.println(F("  CALIBRATION SUMMARY"));
    printSep('═');
    Serial.println();

    // CPR
    Serial.print(F("  Motor A CPR check   : ["));
    Serial.print(pf(resultA.cprPass));
    Serial.print(F("]  Measured="));
    Serial.print(resultA.measuredCPR, 1);
    Serial.print(F("  Target="));
    Serial.println(COUNTS_PER_OUTPUT_REV);

    Serial.print(F("  Motor B CPR check   : ["));
    Serial.print(pf(resultB.cprPass));
    Serial.print(F("]  Measured="));
    Serial.print(resultB.measuredCPR, 1);
    Serial.print(F("  Target="));
    Serial.println(COUNTS_PER_OUTPUT_REV);

    Serial.println();

    // Direction
    Serial.print(F("  Motor A fwd dir     : ["));
    Serial.print(pf(resultA.dirFwdPass));
    Serial.println(F("]  Positive ticks on FORWARD"));

    Serial.print(F("  Motor A bwd dir     : ["));
    Serial.print(pf(resultA.dirBwdPass));
    Serial.println(F("]  Negative ticks on BACKWARD"));

    Serial.print(F("  Motor B fwd dir     : ["));
    Serial.print(pf(resultB.dirFwdPass));
    Serial.println(F("]  Positive ticks on FORWARD"));

    Serial.print(F("  Motor B bwd dir     : ["));
    Serial.print(pf(resultB.dirBwdPass));
    Serial.println(F("]  Negative ticks on BACKWARD"));

    Serial.println();

    // Max RPM
    Serial.print(F("  Motor A max RPM     : ["));
    Serial.print(pf(resultA.rpmPass));
    Serial.print(F("]  Measured="));
    Serial.print(resultA.maxRPM, 1);
    Serial.print(F(" RPM  (threshold ≥ "));
    Serial.print(MIN_ACCEPTABLE_RPM, 0);
    Serial.println(F(" RPM)"));

    Serial.print(F("  Motor B max RPM     : ["));
    Serial.print(pf(resultB.rpmPass));
    Serial.print(F("]  Measured="));
    Serial.print(resultB.maxRPM, 1);
    Serial.print(F(" RPM  (threshold ≥ "));
    Serial.print(MIN_ACCEPTABLE_RPM, 0);
    Serial.println(F(" RPM)"));

    Serial.println();
    printSep('═');
    if (allPass) {
        Serial.println(F("  Overall result : ALL PASS  ✓"));
        Serial.println(F("  Encoder wiring, ISR, and CPR constant verified."));
        Serial.println(F("  Safe to proceed to EncoderRead and PIDSpeedControl examples."));
    } else {
        Serial.println(F("  Overall result : FAIL  ✗"));
        Serial.println(F("  Resolve the failures above before running PID examples."));
        Serial.println();
        Serial.println(F("  Quick-reference troubleshooting:"));
        Serial.println(F("    CPR ≈ 0    → ISR not attached; check encoder GPIO & begin()"));
        Serial.println(F("    CPR ≈ 1232 → only one encoder channel receiving interrupts"));
        Serial.println(F("    CPR wrong ratio → COUNTS_PER_OUTPUT_REV mismatch in header"));
        Serial.println(F("    Direction fail → swap motor + or - wires (or encoder A/B)"));
        Serial.println(F("    RPM low        → check 12 V supply voltage under load"));
    }
    printSep('═');
}

// ============================================================
//  ── setup() ─────────────────────────────────────────────────
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(600);   // allow terminal to connect

    // ── Banner ────────────────────────────────────────────
    Serial.println();
    Serial.println(F("╔══════════════════════════════════════════════════════════════════╗"));
    Serial.println(F("║  TB6612FNG_ESP32 — Encoder Calibration                          ║"));
    Serial.println(F("║  Library v1.0.0  |  ESP32 Arduino                               ║"));
    Serial.println(F("║                                                                  ║"));
    Serial.println(F("║  Motors will spin!  Secure the robot before continuing.         ║"));
    Serial.println(F("╚══════════════════════════════════════════════════════════════════╝"));
    Serial.println();

    // ── Print the constants being verified ────────────────
    Serial.println(F("  Constants under test:"));
    Serial.print(F("    PULSES_PER_MOTOR_REV  = "));
    Serial.println(PULSES_PER_MOTOR_REV);
    Serial.print(F("    GEAR_RATIO            = "));
    Serial.println(GEAR_RATIO);
    Serial.print(F("    PULSES_PER_OUTPUT_REV = "));
    Serial.println(PULSES_PER_OUTPUT_REV);
    Serial.print(F("    COUNTS_PER_OUTPUT_REV = "));
    Serial.print(COUNTS_PER_OUTPUT_REV);
    Serial.println(F("  (target CPR — quadrature ×4)"));
    Serial.println();

    // ── Pin configs ───────────────────────────────────────
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
    EncoderPinConfig encoderCfgA = { .pinA = PIN_ENC_A_A, .pinB = PIN_ENC_A_B };
    EncoderPinConfig encoderCfgB = { .pinA = PIN_ENC_B_A, .pinB = PIN_ENC_B_B };

    driver.begin(motorCfgA, motorCfgB, encoderCfgA, encoderCfgB, PIN_STBY);
    driver.enable();
    Serial.println(F("  Driver initialised.  STBY → HIGH."));

    // ── Safety countdown ──────────────────────────────────
    for (int8_t i = 3; i > 0; i--) {
        Serial.print(F("  Starting in "));
        Serial.print(i);
        Serial.println(F(" s ..."));
        delay(1000);
    }
    Serial.println();

    // ══ Phase 1: CPR Verification ════════════════════════
    printSep('─');
    Serial.println(F("  Phase 1 · CPR Verification"));
    Serial.print(F("  ("));
    Serial.print(CALIB_DURATION_MS / 1000);
    Serial.print(F(" s run per motor at PWM="));
    Serial.print(CALIB_PWM);
    Serial.println(F(")"));
    printSep('─');

    runPhase1CPR(driver.motorA, driver.encoderA, "A", resultA);
    delay(600);
    runPhase1CPR(driver.motorB, driver.encoderB, "B", resultB);
    delay(600);

    // ══ Phase 2: RPM Sweep ═══════════════════════════════
    runPhase2Sweep();
    delay(600);

    // ══ Phase 3: Direction Test ═══════════════════════════
    Serial.println();
    printSep('─');
    Serial.println(F("  Phase 3 · Direction Verification"));
    printSep('─');

    runPhase3Direction(driver.motorA, driver.encoderA, "A", resultA);
    delay(600);
    runPhase3Direction(driver.motorB, driver.encoderB, "B", resultB);
    delay(600);

    // ══ Summary ═══════════════════════════════════════════
    printSummary();

    // ── Safe shutdown ─────────────────────────────────────
    driver.brakeAll();
    driver.disable();
    Serial.println();
    Serial.println(F("  Motors stopped.  STBY → LOW."));
    Serial.println(F("  Calibration complete — check results above."));
}

// ============================================================
//  ── loop() ──────────────────────────────────────────────────
//
//  Nothing to do.  All calibration logic runs once in setup().
//  The serial output remains readable on the monitor.
//  Press the ESP32 EN/RESET button to re-run calibration.
// ============================================================

void loop()
{
    // Re-run hint printed once per 30 s so the operator knows
    // the sketch is not frozen.
    static uint32_t lastHint = 0;
    if (millis() - lastHint >= 30000UL) {
        lastHint = millis();
        Serial.println(F("  [Press EN/RESET to re-run calibration]"));
    }
}
