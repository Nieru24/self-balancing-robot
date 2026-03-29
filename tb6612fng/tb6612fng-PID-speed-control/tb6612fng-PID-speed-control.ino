/**
 * @file    PIDSpeedControl.ino
 * @brief   Closed-loop RPM speed control example for TB6612FNG_ESP32 library.
 *
 * ─────────────────────────────────────────────────────────────────────────
 * PURPOSE
 * ─────────────────────────────────────────────────────────────────────────
 * Demonstrates the full closed-loop PID speed-control workflow:
 *
 *   1. Set a target RPM via setSetpoint().
 *   2. Call updatePID() at a fixed 20 Hz interval from loop().
 *      updatePID() reads getRPM() from each encoder, passes the result
 *      to pidA/pidB.update(), and feeds the signed output to setSpeed().
 *   3. Print a fixed-width telemetry table every PRINT_INTERVAL_MS so
 *      both Serial Monitor and Serial Plotter can display the response.
 *   4. Cycle through a setpoint schedule automatically, then accept live
 *      Serial commands for interactive tuning.
 *
 * ─────────────────────────────────────────────────────────────────────────
 * SETPOINT SCHEDULE  (runs once, then repeats)
 * ─────────────────────────────────────────────────────────────────────────
 *
 *   Step  Setpoint   Duration  Purpose
 *   ────  ─────────  ────────  ─────────────────────────────────────────
 *    1     100 RPM    6 s      Moderate forward — observe rise time
 *    2      60 RPM    5 s      Step down — observe settling / undershoot
 *    3     140 RPM    6 s      Aggressive forward — near rated speed
 *    4       0 RPM    3 s      Full stop — observe integral reset / brake
 *    5    -100 RPM    6 s      Reverse — confirm signed setpoint works
 *    6     100 RPM    6 s      Re-acquire forward — observe transient
 *
 * ─────────────────────────────────────────────────────────────────────────
 * SERIAL TELEMETRY FORMAT  (115200 baud)
 * ─────────────────────────────────────────────────────────────────────────
 *
 * Header (printed once at startup):
 *   t_ms,spA,rpmA,errA,pwmA,spB,rpmB,errB,pwmB
 *
 * Data rows (every PRINT_INTERVAL_MS):
 *   1050, 100.0, 97.3, 2.7,  162,  100.0,  96.1,  3.9,  164
 *   1100, 100.0, 99.8, 0.2,  155,  100.0,  99.2,  0.8,  157
 *   ...
 *
 * Fields:
 *   t_ms  — millis() timestamp
 *   spA   — Motor A setpoint (RPM)
 *   rpmA  — Motor A measured RPM
 *   errA  — Motor A PID error  (setpoint − measured)
 *   pwmA  — Motor A PWM output (signed, −255..+255)
 *   spB   — Motor B setpoint (RPM)
 *   rpmB  — Motor B measured RPM
 *   errB  — Motor B PID error
 *   pwmB  — Motor B PWM output
 *
 * Paste the CSV output into a spreadsheet or Python/matplotlib script to
 * plot step-response curves and tune Kp/Ki/Kd.
 *
 * ─────────────────────────────────────────────────────────────────────────
 * SERIAL PLOTTER COMPATIBILITY
 * ─────────────────────────────────────────────────────────────────────────
 * Set PLOTTER_MODE = true to emit bare space-separated values with Arduino
 * Serial Plotter labels.  The plotter cannot parse CSV headers, so in
 * plotter mode the CSV header is suppressed and labels are embedded in the
 * value stream instead.
 *
 * ─────────────────────────────────────────────────────────────────────────
 * LIVE SERIAL COMMANDS  (type in Serial Monitor, send with newline)
 * ─────────────────────────────────────────────────────────────────────────
 *   s<value>      Set BOTH motors to <value> RPM.  e.g. "s120" → 120 RPM
 *   a<value>      Set Motor A only.               e.g. "a80"
 *   b<value>      Set Motor B only.               e.g. "b80"
 *   p<kp>,<ki>,<kd>  Retune BOTH PIDs live.       e.g. "p2.0,1.0,0.02"
 *   r             Reset both PIDs and encoders.
 *   x             Emergency stop (setpoint 0, brakeAll).
 *   ?             Print current gains and setpoints.
 *
 * Commands pause the automatic setpoint schedule.  Send "auto" to resume.
 *
 * ─────────────────────────────────────────────────────────────────────────
 * GAIN TUNING GUIDE
 * ─────────────────────────────────────────────────────────────────────────
 * Starting point for JGB37-520 @ 12 V, no load:
 *   Kp = 1.5   Ki = 0.8   Kd = 0.05   (library defaults)
 *
 * Symptom                  → Adjustment
 * ───────────────────────────────────────────────────────────────────────
 * Slow to reach setpoint   → Increase Kp
 * Oscillation at setpoint  → Decrease Kp, or increase Kd slightly
 * Steady-state offset      → Increase Ki
 * Integral windup / kick   → Decrease Ki; check PID_INTEGRAL_LIMIT
 * Noisy derivative         → Decrease Kd or increase PID_INTERVAL_MS
 * Stalls below ~40 RPM     → Motor dead-band; add a feed-forward offset
 *
 * ─────────────────────────────────────────────────────────────────────────
 * PIN ASSIGNMENT  (edit the #defines to match your wiring)
 * ─────────────────────────────────────────────────────────────────────────
 *  TB6612FNG  │ ESP32 GPIO   Notes
 *  ───────────┼──────────── ─────────────────────────────────
 *  PWMA       │ 18          Motor A PWM   (LEDC ch 0)
 *  AIN1       │ 16          Motor A IN1
 *  AIN2       │ 17          Motor A IN2
 *  PWMB       │ 19          Motor B PWM   (LEDC ch 1)
 *  BIN1       │ 21          Motor B IN1
 *  BIN2       │ 22          Motor B IN2
 *  STBY       │  4          Standby (active HIGH)
 *  Enc A — A  │ 34          Yellow wire (C1)
 *  Enc A — B  │ 35          Green  wire (C2)
 *  Enc B — A  │ 32          Yellow wire (C1)
 *  Enc B — B  │ 33          Green  wire (C2)
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
//  ── PID TIMING ───────────────────────────────────────────────
//
//  PID_INTERVAL_MS controls how often the control law executes.
//  20 Hz (50 ms) is a good starting point for this motor class:
//    - Fast enough to catch transients from load disturbances.
//    - Slow enough that each getRPM() window contains enough ticks
//      to compute a meaningful speed (at 100 RPM, 50 ms ≈ 205 counts).
//
//  Halving this to 25 ms (40 Hz) works, but requires re-tuning Ki/Kd
//  because the integral and derivative terms scale with dt.
// ============================================================

static constexpr uint32_t PID_INTERVAL_MS   = 50;   ///< Control loop period (ms) — 20 Hz

// ============================================================
//  ── TELEMETRY TIMING ─────────────────────────────────────────
//
//  Print every PRINT_INTERVAL_MS.  Must be a multiple of
//  PID_INTERVAL_MS to avoid aliasing.  100 ms = 10 Hz output
//  gives smooth Serial Plotter traces without flooding the UART.
// ============================================================

static constexpr uint32_t PRINT_INTERVAL_MS = 100;  ///< Serial telemetry period (ms)

// ============================================================
//  ── OUTPUT MODE ──────────────────────────────────────────────
//
//  false → CSV with header (best for spreadsheet / logging)
//  true  → Space-separated with Arduino Serial Plotter labels
// ============================================================

static constexpr bool PLOTTER_MODE = false;

// ============================================================
//  ── INITIAL PID GAINS ────────────────────────────────────────
//
//  Override the library defaults here for this specific motor +
//  supply voltage combination.  Use the tuning guide in the
//  file header to adjust these values.
// ============================================================

static constexpr float INIT_KP = 1.5f;
static constexpr float INIT_KI = 0.8f;
static constexpr float INIT_KD = 0.05f;

// ============================================================
//  ── SETPOINT SCHEDULE ────────────────────────────────────────
// ============================================================

struct SetpointStep {
    float    rpmA;        ///< Motor A setpoint (RPM, signed)
    float    rpmB;        ///< Motor B setpoint (RPM, signed)
    uint32_t durationMs;  ///< How long to hold this setpoint
    const char* label;    ///< Human-readable step name (printed at transition)
};

static const SetpointStep SCHEDULE[] = {
    {  100.0f,  100.0f, 6000, "FWD  100 RPM  — observe rise time"        },
    {   60.0f,   60.0f, 5000, "FWD   60 RPM  — step down / settling"     },
    {  140.0f,  140.0f, 6000, "FWD  140 RPM  — near rated speed"         },
    {    0.0f,    0.0f, 3000, "STOP   0 RPM  — integral reset / brake"   },
    { -100.0f, -100.0f, 6000, "REV  100 RPM  — reverse direction"        },
    {  100.0f,  100.0f, 6000, "FWD  100 RPM  — re-acquire after reverse" },
};

static constexpr uint8_t SCHEDULE_LEN =
    sizeof(SCHEDULE) / sizeof(SCHEDULE[0]);

// ============================================================
//  ── DRIVER INSTANCE ─────────────────────────────────────────
// ============================================================

TB6612FNG_ESP32 driver;

// ============================================================
//  ── SKETCH STATE ─────────────────────────────────────────────
// ============================================================

static uint32_t lastPIDMs      = 0;   ///< Last PID tick timestamp
static uint32_t lastPrintMs    = 0;   ///< Last telemetry print timestamp
static uint32_t stepStartMs    = 0;   ///< When the current schedule step began
static uint8_t  scheduleIndex  = 0;   ///< Current index into SCHEDULE[]
static bool     autoMode       = true;///< true = run schedule; false = manual
static String   serialBuffer;         ///< Accumulates Serial input characters

// ============================================================
//  ── FORWARD DECLARATIONS ────────────────────────────────────
// ============================================================

static void applySetpoint(float rpmA, float rpmB);
static void printTelemetryHeader();
static void printTelemetry();
static void handleSerialCommand(const String& cmd);
static void advanceSchedule();

// ============================================================
//  ── applySetpoint() ──────────────────────────────────────────
//
//  Sets the PID target for both channels.  Calling setSetpoint()
//  triggers an automatic integrator reset inside the library when
//  the new value differs from the current one by more than 10 RPM,
//  which prevents windup carry-over between steps.
// ============================================================

static void applySetpoint(float rpmA, float rpmB)
{
    driver.pidA.setSetpoint(rpmA);
    driver.pidB.setSetpoint(rpmB);
}

// ============================================================
//  ── TELEMETRY ────────────────────────────────────────────────
// ============================================================

static void printTelemetryHeader()
{
    if (PLOTTER_MODE) {
        // Arduino Serial Plotter reads the first data row for labels
        // if they are prefixed with '>'.  Not all IDE versions support
        // this, so we emit a comment-style header instead.
        Serial.println(F("# setA:rpm  measuredA:rpm  errorA:rpm  pwmA  setB:rpm  measuredB:rpm  errorB:rpm  pwmB"));
    } else {
        // CSV header — importable directly into Excel / LibreOffice Calc
        // or pandas (pd.read_csv with parse_dates=False).
        Serial.println(F("t_ms,spA,rpmA,errA,pwmA,spB,rpmB,errB,pwmB"));
    }
}

static void printTelemetry()
{
    // Snapshot all values atomically-enough for display purposes.
    // RPM is re-read here (not from the last PID tick) so the print
    // interval and PID interval can differ independently.
    float spA   = driver.pidA.getSetpoint();
    float rpmA  = driver.encoderA.getRPM();
    float errA  = driver.pidA.getLastError();
    float outA  = driver.pidA.getLastOutput();

    float spB   = driver.pidB.getSetpoint();
    float rpmB  = driver.encoderB.getRPM();
    float errB  = driver.pidB.getLastError();
    float outB  = driver.pidB.getLastOutput();

    // Clamp raw PID output to display range for the PWM column.
    int16_t pwmA = (int16_t)constrain(outA, -255.0f, 255.0f);
    int16_t pwmB = (int16_t)constrain(outB, -255.0f, 255.0f);

    if (PLOTTER_MODE) {
        // Space-separated, no labels — Arduino Serial Plotter format.
        // Each token becomes a separate trace.
        Serial.print(spA,  1);  Serial.print(' ');
        Serial.print(rpmA, 1);  Serial.print(' ');
        Serial.print(spB,  1);  Serial.print(' ');
        Serial.println(rpmB, 1);
    } else {
        // Full CSV row.
        Serial.print(millis());   Serial.print(',');
        Serial.print(spA,  1);   Serial.print(',');
        Serial.print(rpmA, 1);   Serial.print(',');
        // Error: add leading '+' for positive values so columns stay aligned.
        if (errA >= 0.0f) Serial.print('+');
        Serial.print(errA, 1);   Serial.print(',');
        Serial.print(pwmA);      Serial.print(',');
        Serial.print(spB,  1);   Serial.print(',');
        Serial.print(rpmB, 1);   Serial.print(',');
        if (errB >= 0.0f) Serial.print('+');
        Serial.print(errB, 1);   Serial.print(',');
        Serial.println(pwmB);
    }
}

// ============================================================
//  ── SCHEDULE ADVANCE ─────────────────────────────────────────
// ============================================================

static void advanceSchedule()
{
    if (!autoMode) return;

    uint32_t now = millis();
    if (now - stepStartMs < SCHEDULE[scheduleIndex].durationMs) return;

    // Move to the next step (wrap at end).
    scheduleIndex = (scheduleIndex + 1) % SCHEDULE_LEN;
    stepStartMs   = now;

    const SetpointStep& step = SCHEDULE[scheduleIndex];
    applySetpoint(step.rpmA, step.rpmB);

    // Announce the transition on Serial (clearly marked, not CSV data).
    Serial.print(F("# ["));
    Serial.print(now);
    Serial.print(F(" ms]  Step "));
    Serial.print(scheduleIndex + 1);
    Serial.print(F("/"));
    Serial.print(SCHEDULE_LEN);
    Serial.print(F(":  "));
    Serial.println(step.label);
}

// ============================================================
//  ── SERIAL COMMAND PARSER ────────────────────────────────────
// ============================================================

static void handleSerialCommand(const String& cmd)
{
    if (cmd.length() == 0) return;

    char code = cmd.charAt(0);

    // ── s<rpm>  Set both motors ──────────────────────────────
    if (code == 's' && cmd.length() > 1) {
        autoMode = false;
        float rpm = cmd.substring(1).toFloat();
        applySetpoint(rpm, rpm);
        Serial.print(F("# Manual: both = "));
        Serial.print(rpm, 1);
        Serial.println(F(" RPM  (send 'auto' to resume schedule)"));
        return;
    }

    // ── a<rpm>  Set Motor A only ─────────────────────────────
    if (code == 'a' && cmd.length() > 1) {
        autoMode = false;
        float rpm = cmd.substring(1).toFloat();
        driver.pidA.setSetpoint(rpm);
        Serial.print(F("# Manual: Motor A = "));
        Serial.print(rpm, 1);
        Serial.println(F(" RPM"));
        return;
    }

    // ── b<rpm>  Set Motor B only ─────────────────────────────
    if (code == 'b' && cmd.length() > 1) {
        autoMode = false;
        float rpm = cmd.substring(1).toFloat();
        driver.pidB.setSetpoint(rpm);
        Serial.print(F("# Manual: Motor B = "));
        Serial.print(rpm, 1);
        Serial.println(F(" RPM"));
        return;
    }

    // ── p<kp>,<ki>,<kd>  Retune both PIDs live ───────────────
    if (code == 'p' && cmd.length() > 1) {
        // Parse three comma-separated floats after 'p'.
        String params = cmd.substring(1);
        int c1 = params.indexOf(',');
        int c2 = params.lastIndexOf(',');
        if (c1 > 0 && c2 > c1) {
            float kp = params.substring(0,      c1).toFloat();
            float ki = params.substring(c1 + 1, c2).toFloat();
            float kd = params.substring(c2 + 1).toFloat();
            driver.pidA.setGains(kp, ki, kd);
            driver.pidB.setGains(kp, ki, kd);
            // Reset integrators so the new gains take effect cleanly.
            driver.pidA.reset();
            driver.pidB.reset();
            Serial.print(F("# Gains updated: Kp="));
            Serial.print(kp, 3);
            Serial.print(F("  Ki="));
            Serial.print(ki, 3);
            Serial.print(F("  Kd="));
            Serial.println(kd, 4);
        } else {
            Serial.println(F("# Usage: p<Kp>,<Ki>,<Kd>  e.g. p2.0,1.0,0.02"));
        }
        return;
    }

    // ── r  Reset PIDs and encoders ───────────────────────────
    if (code == 'r') {
        driver.pidA.reset();
        driver.pidB.reset();
        driver.resetEncoders();
        Serial.println(F("# PIDs and encoders reset."));
        return;
    }

    // ── x  Emergency stop ────────────────────────────────────
    if (code == 'x') {
        autoMode = false;
        applySetpoint(0.0f, 0.0f);
        driver.brakeAll();
        driver.pidA.reset();
        driver.pidB.reset();
        Serial.println(F("# Emergency stop!  Motors braked.  Setpoints = 0."));
        return;
    }

    // ── ?  Print current state ───────────────────────────────
    if (code == '?') {
        Serial.println(F("# ── Current PID state ──────────────────────────"));
        Serial.print(F("#  Gains (A): Kp="));
        // We can't read gains back from the PID object (private), so we
        // echo the last values set and note that defaults are used if
        // no 'p' command has been issued.
        Serial.println(F("  [use 'p' command to set; default Kp=1.5 Ki=0.8 Kd=0.05]"));
        Serial.print(F("#  Setpoint A: "));
        Serial.print(driver.pidA.getSetpoint(), 1);
        Serial.print(F(" RPM    Setpoint B: "));
        Serial.print(driver.pidB.getSetpoint(), 1);
        Serial.println(F(" RPM"));
        Serial.print(F("#  Measured A: "));
        Serial.print(driver.encoderA.getRPM(), 1);
        Serial.print(F(" RPM    Measured B: "));
        Serial.print(driver.encoderB.getRPM(), 1);
        Serial.println(F(" RPM"));
        Serial.print(F("#  Error    A: "));
        Serial.print(driver.pidA.getLastError(), 2);
        Serial.print(F(" RPM    Error    B: "));
        Serial.print(driver.pidB.getLastError(), 2);
        Serial.println(F(" RPM"));
        Serial.print(F("#  PID out  A: "));
        Serial.print(driver.pidA.getLastOutput(), 1);
        Serial.print(F("       PID out  B: "));
        Serial.println(driver.pidB.getLastOutput(), 1);
        Serial.print(F("#  Ticks    A: "));
        Serial.print(driver.encoderA.getTicks());
        Serial.print(F("          Ticks    B: "));
        Serial.println(driver.encoderB.getTicks());
        Serial.print(F("#  Auto mode: "));
        Serial.println(autoMode ? F("ON") : F("OFF"));
        Serial.println(F("# ────────────────────────────────────────────────"));
        return;
    }

    // ── auto  Resume schedule ────────────────────────────────
    if (cmd.equalsIgnoreCase(F("auto"))) {
        autoMode      = true;
        stepStartMs   = millis();   // restart current step timer
        scheduleIndex = 0;
        const SetpointStep& step = SCHEDULE[0];
        applySetpoint(step.rpmA, step.rpmB);
        Serial.println(F("# Auto schedule resumed from step 1."));
        return;
    }

    Serial.print(F("# Unknown command: '"));
    Serial.print(cmd);
    Serial.println(F("'  (send '?' for help)"));
}

// ============================================================
//  ── setup() ─────────────────────────────────────────────────
// ============================================================

void setup()
{
    Serial.begin(115200);
    delay(500);

    // ── Banner ────────────────────────────────────────────────
    Serial.println();
    Serial.println(F("╔══════════════════════════════════════════════════════════════════╗"));
    Serial.println(F("║  TB6612FNG_ESP32 — Closed-Loop PID Speed Control                ║"));
    Serial.println(F("║  Library v1.0.0  |  ESP32 Arduino core v3.x                    ║"));
    Serial.println(F("╚══════════════════════════════════════════════════════════════════╝"));
    Serial.println();

    // ── Hardware constants ────────────────────────────────────
    Serial.print  (F("  COUNTS_PER_OUTPUT_REV : "));  Serial.println(COUNTS_PER_OUTPUT_REV);
    Serial.print  (F("  PID interval          : "));  Serial.print(PID_INTERVAL_MS);   Serial.println(F(" ms"));
    Serial.print  (F("  Telemetry interval    : "));  Serial.print(PRINT_INTERVAL_MS); Serial.println(F(" ms"));
    Serial.print  (F("  Initial gains         : Kp="));
    Serial.print  (INIT_KP, 3); Serial.print(F("  Ki="));
    Serial.print  (INIT_KI, 3); Serial.print(F("  Kd="));
    Serial.println(INIT_KD, 4);
    Serial.print  (F("  Output mode           : "));
    Serial.println(PLOTTER_MODE ? F("Serial Plotter") : F("CSV"));
    Serial.println();

    // ── Available commands ────────────────────────────────────
    Serial.println(F("  Live serial commands (send with newline):"));
    Serial.println(F("    s<rpm>          — set both motors  (e.g. s120)"));
    Serial.println(F("    a<rpm>          — set Motor A only (e.g. a80)"));
    Serial.println(F("    b<rpm>          — set Motor B only (e.g. b-50)"));
    Serial.println(F("    p<kp>,<ki>,<kd> — retune PIDs live (e.g. p2.0,1.0,0.02)"));
    Serial.println(F("    r               — reset PIDs and encoder counters"));
    Serial.println(F("    x               — emergency stop"));
    Serial.println(F("    auto            — resume automatic schedule"));
    Serial.println(F("    ?               — print current state"));
    Serial.println();

    // ── Pin configs ───────────────────────────────────────────
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

    // ── Driver init ───────────────────────────────────────────
    driver.begin(motorCfgA, motorCfgB, encoderCfgA, encoderCfgB, PIN_STBY);

    // ── Apply initial gains ───────────────────────────────────
    driver.pidA.setGains(INIT_KP, INIT_KI, INIT_KD);
    driver.pidB.setGains(INIT_KP, INIT_KI, INIT_KD);

    // ── Safety countdown ──────────────────────────────────────
    for (int8_t i = 3; i > 0; i--) {
        Serial.print(F("  Starting in "));
        Serial.print(i);
        Serial.println(F(" s ..."));
        delay(1000);
    }
    Serial.println();

    // ── Enable driver ─────────────────────────────────────────
    driver.enable();
    Serial.println(F("  STBY → HIGH (driver enabled)"));
    Serial.println();

    // ── Prime the RPM estimator ───────────────────────────────
    // getRPM() uses a delta-tick / delta-time method.  The very first
    // call after begin() would use a delta-time measured from before the
    // motors were configured, giving a spurious near-zero RPM.  Running
    // the PID for one warmup tick discards that stale window.
    driver.encoderA.getRPM();
    driver.encoderB.getRPM();

    // ── Load first schedule step ──────────────────────────────
    applySetpoint(SCHEDULE[0].rpmA, SCHEDULE[0].rpmB);
    stepStartMs  = millis();
    scheduleIndex = 0;

    Serial.print(F("# [0 ms]  Step 1/"));
    Serial.print(SCHEDULE_LEN);
    Serial.print(F(":  "));
    Serial.println(SCHEDULE[0].label);
    Serial.println();

    // ── Telemetry header ──────────────────────────────────────
    printTelemetryHeader();

    // ── Timestamp baselines ───────────────────────────────────
    lastPIDMs   = millis();
    lastPrintMs = millis();
    serialBuffer.reserve(32);
}

// ============================================================
//  ── loop() ───────────────────────────────────────────────────
//
//  Three independent timed tasks, all millis()-gated:
//
//    Task 1 (PID_INTERVAL_MS):   run control law on both channels
//    Task 2 (PRINT_INTERVAL_MS): emit one telemetry row
//    Task 3 (always):            drain Serial input buffer
//
//  The tasks are checked sequentially every iteration.  Because
//  none of them block (no delay()), all three fire on time as long
//  as the loop body completes well under PID_INTERVAL_MS.  On an
//  ESP32 @ 240 MHz, the worst-case Serial.print() chain is ~20 µs,
//  so there is no risk of timing drift at 20 Hz.
// ============================================================

void loop()
{
    uint32_t now = millis();

    // ── Task 1: PID control law ─────────────────────────────────
    if (now - lastPIDMs >= PID_INTERVAL_MS) {
        lastPIDMs = now;

        // updatePID() reads getRPM() from each encoder, calls
        // pidA/pidB.update(), and applies setSpeed() to each motor.
        // When setpoint is 0 the PID output is 0, which routes to
        // Motor::brake() — holding the wheels against gravity.
        driver.updatePID();

        // Advance the automatic schedule if in auto mode.
        advanceSchedule();
    }

    // ── Task 2: Serial telemetry ────────────────────────────────
    if (now - lastPrintMs >= PRINT_INTERVAL_MS) {
        lastPrintMs = now;
        printTelemetry();
    }

    // ── Task 3: Serial command input ────────────────────────────
    // Read all available bytes into a String buffer, execute when
    // a newline is received.  Using a String here is deliberate:
    // this is a low-frequency (human-typed) input path, not an ISR,
    // so the heap allocation is acceptable.
    while (Serial.available()) {
        char c = static_cast<char>(Serial.read());
        if (c == '\n' || c == '\r') {
            serialBuffer.trim();
            if (serialBuffer.length() > 0) {
                handleSerialCommand(serialBuffer);
            }
            serialBuffer = "";
        } else {
            serialBuffer += c;
        }
    }
}
