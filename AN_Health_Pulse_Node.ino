// ─────────────────────────────────────────────────────────────────────────────
//  BPM + SpO2 Monitor  ·  MAX30100 + SSD1306 128×64  ·  NodeMCU / ESP8266
//  Astranova Intelligence
//
//  MAX30100 I2C peripherals bata 1.8V logic matra support garxa —
//  5V logic (Uno/Nano/Mega) sanga directly connect nagarne.
//  NodeMCU (3.3V logic) use garne: SCL → D1, SDA → D2
//
//  Buzzer: BUZZER_PIN ra GND ko beech ma connect garne.
//          - Active buzzer  → works directly (tone() drives it)
//          - Passive buzzer → tone() + correct frequency required
//
//  Waveform: Real ECG morphology (P → PR → QRS → ST → T → TP)
//            Scrolls left at a speed that matches the measured BPM.
//            Snaps to R-peak on each detected beat for tight A/V sync.
//
//  ── USB SERIAL DASHBOARD PROTOCOL (added) ───────────────────────────────────
//  115200 baud, plain ASCII lines terminated with '\n'. Pairs with the
//  AN-Health Pulse Dashboard (Web Serial) over USB.
//
//    READY:<name>:sr=<hz>          sent once, right after successful sensor init
//    ERR:<code>                    sent once if sensor init fails (halts after)
//    D:<wave>,<bpm>,<spo2>,<finger> streamed every frame (~50 Hz / every 20ms)
//                                    wave   = signed int, current waveform y-offset
//                                    bpm    = float, 1 decimal (0.0 if unavailable)
//                                    spo2   = float, 1 decimal (0.0 if unavailable)
//                                    finger = 0 or 1
//    BEAT                           sent once per detected heartbeat
// ─────────────────────────────────────────────────────────────────────────────

#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <U8g2lib.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  CONFIGURATION
// ═══════════════════════════════════════════════════════════════════════════════
#define REPORTING_PERIOD_MS   1000      // how often to refresh BPM / SpO2 values
#define FILTER_SIZE           5         // median filter window size

#define BUZZER_PIN            D5        // GPIO14 on NodeMCU — change if needed
#define BUZZER_FREQ_HZ        1200      // tone for passive buzzer (ignored by active)
#define BUZZER_DURATION_MS    45        // beep length in milliseconds

#define SERIAL_SAMPLE_RATE_HZ 50        // matches the 20ms frame delay below

// ═══════════════════════════════════════════════════════════════════════════════
//  DISPLAY  — SSD1306 128×64, I2C
//  (original code had SH1106; replace U8G2_SSD1306 with U8G2_SH1106 if needed)
// ═══════════════════════════════════════════════════════════════════════════════
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset= */ U8X8_PIN_NONE);

// ═══════════════════════════════════════════════════════════════════════════════
//  SENSOR STATE
// ═══════════════════════════════════════════════════════════════════════════════
PulseOximeter pox;
uint32_t tsLastReport = 0;

float hrBuffer[FILTER_SIZE]   = {};
float spo2Buffer[FILTER_SIZE] = {};
int   filterIdx               = 0;
float displayBPM              = 0.0f;
float displaySpO2             = 0.0f;
float bpmForSpeed             = 60.0f;  // drives waveform scroll speed

// ═══════════════════════════════════════════════════════════════════════════════
//  BUZZER STATE
// ═══════════════════════════════════════════════════════════════════════════════
bool     buzzerPending = false;   // set inside onBeatDetected()
bool     buzzerOn      = false;
uint32_t buzzerStart   = 0;

// ═══════════════════════════════════════════════════════════════════════════════
//  ECG WAVEFORM
//
//  Render area  : x = 0..127 (128 px wide),  y = 13..63 (51 px tall)
//  Baseline     : y = 52   (flat line, positive offsets go UP → lower y)
//  R-peak limit : y = 14   (values clamped to avoid overwriting header)
//
//  Template: 49 samples = one cardiac cycle
//  At 60 BPM + 20 ms/frame the scroll speed = 49 samples / 50 frames ≈ 0.98 ✓
// ═══════════════════════════════════════════════════════════════════════════════
#define WAVE_BASELINE      52     // pixel row for isoelectric line
#define WAVE_CEIL          14     // topmost pixel row (R-peak clamped here)
#define ECG_TEMPLATE_SIZE  49
#define ECG_R_PEAK_IDX     19     // index of the R-peak sample in the template

//  Each value is a signed y-offset from WAVE_BASELINE:
//    positive  → upward deflection (y decreases)
//    negative  → downward deflection (y increases)
//
//  Anatomy:
//    idx  0–4  : isoelectric lead-in (5)
//    idx  5–12 : P wave — small rounded hump (8)
//    idx 13–15 : PR segment (3)
//    idx 16–17 : Q dip (2)
//    idx 18–21 : R spike — tall narrow peak (4)   ← R-peak at idx 19
//    idx 22–24 : S dip (3)
//    idx 25–29 : ST segment (5)
//    idx 30–38 : T wave — rounded hump (9)
//    idx 39–48 : TP baseline before next beat (10)
static const int8_t ECG_TEMPLATE[ECG_TEMPLATE_SIZE] PROGMEM = {
  // ── Isoelectric lead-in ──────────────────────────────────────
   0,  0,  0,  0,  0,
  // ── P wave ───────────────────────────────────────────────────
   2,  5,  8,  8,  7,  5,  2,  0,
  // ── PR segment ───────────────────────────────────────────────
   0,  0,  0,
  // ── Q dip ────────────────────────────────────────────────────
  -4, -8,
  // ── R spike ──────────────────────────────────────────────────
  12, 38, 38, 14,
  // ── S dip ────────────────────────────────────────────────────
  -8, -4,  0,
  // ── ST segment ───────────────────────────────────────────────
   0,  0,  0,  0,  0,
  // ── T wave ───────────────────────────────────────────────────
   2,  6, 11, 14, 14, 11,  6,  2,  0,
  // ── TP baseline (wait for next beat) ─────────────────────────
   0,  0,  0,  0,  0,  0,  0,  0,  0,  0
};

// Scrolling display buffer: one y-offset per column (128 columns)
int8_t  waveBuffer[128] = {};
float   templatePos     = 0.0f;   // fractional position inside the template
bool    prevFingerOn    = false;   // tracks finger state across frames

// ═══════════════════════════════════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════════════════════════════════
float medianFilter(float* arr, int n) {
  float tmp[FILTER_SIZE];
  memcpy(tmp, arr, n * sizeof(float));
  for (int i = 0; i < n - 1; i++)
    for (int j = i + 1; j < n; j++)
      if (tmp[i] > tmp[j]) { float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
  return tmp[n / 2];
}

float adaptiveFilter(float newVal, float oldVal, float alpha) {
  return alpha * newVal + (1.0f - alpha) * oldVal;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  BEAT CALLBACK  (called synchronously from pox.update())
// ═══════════════════════════════════════════════════════════════════════════════
void onBeatDetected() {
  Serial.println(F("BEAT"));
  buzzerPending = true;
  // Snap the template to the R-peak index so the on-screen QRS spike
  // and the buzzer beep are always in perfect sync.
  templatePos = (float)ECG_R_PEAK_IDX;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Splash screen
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(10, 28);
  u8g2.print("HealthSense v2.0");
  u8g2.setCursor(22, 44);
  u8g2.print("Initializing...");
  u8g2.sendBuffer();

  Serial.print("Initializing MAX30100...");
  if (!pox.begin()) {
    Serial.println("FAILED");
    Serial.println(F("ERR:sensor_init_failed"));
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(12, 30);
    u8g2.print("Sensor Error!");
    u8g2.setCursor(4, 46);
    u8g2.print("Check wiring/power");
    u8g2.sendBuffer();
    for (;;);
  }
  Serial.println("OK");

  pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);  // adjust if readings are weak
  pox.setOnBeatDetectedCallback(onBeatDetected);

  memset(waveBuffer, 0, sizeof(waveBuffer));

  // Tell the dashboard we're up and streaming.
  Serial.print(F("READY:AN-Health Pulse Node:sr="));
  Serial.println(SERIAL_SAMPLE_RATE_HZ);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
  pox.update();

  float heartRate = pox.getHeartRate();
  float spo2      = pox.getSpO2();
  bool  fingerOn  = (heartRate > 1.0f);

  // ── Update filtered display values every REPORTING_PERIOD_MS ────────────────
  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
    if (fingerOn) {
      hrBuffer[filterIdx]   = heartRate;
      spo2Buffer[filterIdx] = spo2;
      filterIdx = (filterIdx + 1) % FILTER_SIZE;

      displayBPM  = medianFilter(hrBuffer,   FILTER_SIZE);
      displaySpO2 = adaptiveFilter(
                      medianFilter(spo2Buffer, FILTER_SIZE),
                      displaySpO2, 0.3f);

      if (displayBPM > 30.0f && displayBPM < 220.0f)
        bpmForSpeed = displayBPM;
    }
    tsLastReport = millis();
  }

  // ── Advance the ECG waveform ─────────────────────────────────────────────────
  //  speed  = BPM × templateSize / (60 BPM × 50 frames/beat)
  //         = BPM × 49 / 3000
  //  e.g.  60 BPM → 0.98 samples/frame
  //        75 BPM → 1.22 samples/frame
  //       120 BPM → 1.96 samples/frame
  float advance = constrain(bpmForSpeed * ECG_TEMPLATE_SIZE / 3000.0f, 0.4f, 3.0f);

  if (fingerOn) {
    // Scroll buffer left and push next ECG template sample onto the right edge.
    // The waveform builds up from the right as the finger is held — it only
    // starts forming after detection, not before.
    memmove(waveBuffer, waveBuffer + 1, 127);
    int tIdx        = (int)templatePos % ECG_TEMPLATE_SIZE;
    waveBuffer[127] = (int8_t)pgm_read_byte(&ECG_TEMPLATE[tIdx]);
    templatePos     = fmodf(templatePos + advance, (float)ECG_TEMPLATE_SIZE);
  } else if (prevFingerOn) {
    // Finger just lifted — clear buffer and reset template so the next
    // detection starts a fresh waveform from a clean baseline.
    memset(waveBuffer, 0, sizeof(waveBuffer));
    templatePos  = 0.0f;
    displayBPM   = 0.0f;
    displaySpO2  = 0.0f;
    bpmForSpeed  = 60.0f;
  }
  // (if finger was already off and stays off — buffer untouched, stays flat)

  prevFingerOn = fingerOn;

  // ── Buzzer ───────────────────────────────────────────────────────────────────
  if (buzzerPending) {
    tone(BUZZER_PIN, BUZZER_FREQ_HZ);
    buzzerStart   = millis();
    buzzerOn      = true;
    buzzerPending = false;
  }
  if (buzzerOn && (millis() - buzzerStart >= BUZZER_DURATION_MS)) {
    noTone(BUZZER_PIN);
    digitalWrite(BUZZER_PIN, LOW);   // ensure pin goes low (active buzzer safety)
    buzzerOn = false;
  }

  // ── Render frame ─────────────────────────────────────────────────────────────
  u8g2.clearBuffer();

  // Header bar: BPM (left) | SpO2 (right)
  u8g2.setFont(u8g2_font_6x10_tf);

  u8g2.setCursor(0, 10);
  u8g2.print("BPM:");
  u8g2.setCursor(26, 10);
  if (displayBPM > 0) u8g2.print((int)displayBPM);
  else                u8g2.print("--");

  u8g2.setCursor(66, 10);
  u8g2.print("SpO2:");
  u8g2.setCursor(101, 10);
  if (displaySpO2 > 0) { u8g2.print((int)displaySpO2); u8g2.print('%'); }
  else                   u8g2.print("--");

  // Divider between header and waveform area
  u8g2.drawHLine(0, 12, 128);

  // "No finger" message
  if (!fingerOn) {
    u8g2.setCursor(14, 42);
    u8g2.print("Place finger...");
  }

  // Dotted baseline guide (every 4 px)
  for (int x = 0; x < 128; x += 4)
    u8g2.drawPixel(x, WAVE_BASELINE);

  // ECG waveform: connect each adjacent column pair with a line
  for (int x = 0; x < 127; x++) {
    int y1 = constrain(WAVE_BASELINE - (int)waveBuffer[x],     WAVE_CEIL, 63);
    int y2 = constrain(WAVE_BASELINE - (int)waveBuffer[x + 1], WAVE_CEIL, 63);
    u8g2.drawLine(x, y1, x + 1, y2);
  }

  u8g2.sendBuffer();

  // ── Stream this frame to the USB dashboard ──────────────────────────────────
  // Uses the same freshly-pushed waveform sample the OLED just drew, so the
  // browser trace and the on-device screen always show the same beat.
  Serial.print(F("D:"));
  Serial.print((int)waveBuffer[127]);
  Serial.print(',');
  Serial.print(displayBPM, 1);
  Serial.print(',');
  Serial.print(displaySpO2, 1);
  Serial.print(',');
  Serial.println(fingerOn ? 1 : 0);

  delay(20);   // 50 fps — matches the 50-frames/beat timing at 60 BPM, and the
               // dashboard's expected 50Hz serial rate
}
