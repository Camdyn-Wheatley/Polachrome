// ─────────────────────────────────────────────────────────────────────────────
// PPM Trainer for FlySky i6X — Direct Channel Protocol
//
// Protocol: line-based commands over serial (9600 baud).
//   CH:ch1,ch2,ch3,ch4,ch5,ch6\n
//   Each value is a PWM microsecond value clamped to [1000, 2000].
//
// Safety: If no valid command is received for 500ms, steering and drive
//         snap to NEUTRAL and weapon is turned off.
// ─────────────────────────────────────────────────────────────────────────────

#define PPM_PIN 2
#define NUM_CHANNELS 6
#define FRAME_LENGTH 20000
#define PULSE_WIDTH 400

#define NEUTRAL 1486
#define SERIAL_TIMEOUT_MS 500

// CH1=steer  CH2=drive  CH3=weapon  CH4=unused  CH5=arm  CH6=enable
int channel[NUM_CHANNELS] = {NEUTRAL, NEUTRAL, 1000, NEUTRAL, 1800, 2000};

unsigned long lastSerialTime = 0;

void setup() {
  pinMode(PPM_PIN, OUTPUT);
  digitalWrite(PPM_PIN, HIGH);
  Serial.begin(9600);
  Serial.println("FlySky Trainer Mode - PPM Generator Ready (Direct Channel)");
  lastSerialTime = millis();
}

void loop() {
  // ── Serial command processing ─────────────────────────────────────────────
  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.startsWith("CH:")) {
      // Parse comma-separated channel values after "CH:"
      String data = line.substring(3);
      int idx = 0;

      for (int i = 0; i < NUM_CHANNELS && data.length() > 0; i++) {
        int comma = data.indexOf(',');
        String token;

        if (comma == -1) {
          token = data;
          data = "";
        } else {
          token = data.substring(0, comma);
          data = data.substring(comma + 1);
        }

        int val = token.toInt();
        if (val >= 1000 && val <= 2000) {
          channel[i] = val;
        }
        idx++;
      }

      lastSerialTime = millis();
    }
  }

  // ── Safety timeout — snap to safe values if comms lost ────────────────────
  if (millis() - lastSerialTime > SERIAL_TIMEOUT_MS) {
    channel[0] = NEUTRAL;   // steer → center
    channel[1] = NEUTRAL;   // drive → stop
    channel[2] = 1000;      // weapon → off
    // CH4-6 (unused, arm, enable) left unchanged
  }

  // ── PPM generation — identical to known-working reference ─────────────────
  unsigned long elapsed_time = 0;

  for (int i = 0; i < NUM_CHANNELS; i++) {
    digitalWrite(PPM_PIN, LOW);
    delayMicroseconds(PULSE_WIDTH);

    digitalWrite(PPM_PIN, HIGH);
    delayMicroseconds(channel[i] - PULSE_WIDTH);
    elapsed_time += channel[i];
  }

  unsigned long sync_gap = FRAME_LENGTH - elapsed_time;
  digitalWrite(PPM_PIN, LOW);
  delayMicroseconds(PULSE_WIDTH);
  digitalWrite(PPM_PIN, HIGH);
  delayMicroseconds(sync_gap - PULSE_WIDTH);
}
