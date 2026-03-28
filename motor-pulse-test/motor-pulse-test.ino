// ============================================================
//  JGB37-520 Hall Encoder Test — ESP32
//  Prints pulse count and estimated RPM to Serial Monitor
// ============================================================

// ── Encoder pins ─────────────────────────────────────────────
// Connect encoder signal wires to any GPIO that supports interrupts.
// The JGB37-520 encoder has 4 wires:
//   Blue   → 3.3V (or 5V — check your module)
//   Black → GND
//   Green → Channel A
//   White → Channel B
#define ENC_A  34
#define ENC_B  35

// ── Encoder state ─────────────────────────────────────────────
volatile long pulseCount = 0;  // volatile because modified inside ISR

// ── Interrupt Service Routine ─────────────────────────────────
// Called every time Channel A sees a rising edge.
// We check Channel B at that moment to determine direction:
//   B is LOW  → forward  → count up
//   B is HIGH → backward → count down
void IRAM_ATTR onEncoderA() {
  if (digitalRead(ENC_B) == LOW) {
    pulseCount++;
  } else {
    pulseCount--;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // Attach interrupt to Channel A — triggers on every rising edge
  attachInterrupt(digitalPinToInterrupt(ENC_A), onEncoderA, RISING);

  Serial.println("Encoder test started. Spin the motor!");
}

void loop() {
  // Snapshot the count safely (disable interrupts briefly)
  noInterrupts();
  long count = pulseCount;
  interrupts();

  // ── RPM calculation ────────────────────────────────────────
  // JGB37-520 has 11 pulses per revolution on the motor shaft.
  // After the gearbox, multiply by gear ratio (e.g. 30, 50, 100
  // — check your specific model's label).
  // Change PPR and GEAR_RATIO to match yours.
  const float PPR        = 11.0;   // pulses per motor revolution
  const float GEAR_RATIO = 30.0;   // check your motor's label

  static long lastCount = 0;
  static unsigned long lastTime = 0;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;  // seconds

  if (dt >= 0.5) {  // update every 500ms
    long delta = count - lastCount;
    lastCount  = count;
    lastTime   = now;

    // Pulses per second → output shaft RPM
    float pps = delta / dt;
    float rpm = (pps / (PPR * GEAR_RATIO)) * 60.0;

    Serial.print("Pulses: ");   Serial.print(count);
    Serial.print("  Delta: ");  Serial.print(delta);
    Serial.print("  RPM: ");    Serial.println(rpm, 1);
  }
}