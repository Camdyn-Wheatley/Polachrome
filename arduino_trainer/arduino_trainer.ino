// ─────────────────────────────────────────────────────────────────────────────
// PPM Trainer for FlySky i6X — based on known-working reference code.
//
// Protocol: single-character commands over serial.
//   w/s = drive forward/back    a/d = steer left/right
//   x/z = weapon on/off         SPACE = emergency stop
//   c   = center steer+drive
// ─────────────────────────────────────────────────────────────────────────────

#define PPM_PIN 2
#define NUM_CHANNELS 6
#define FRAME_LENGTH 20000
#define PULSE_WIDTH 400

#define NEUTRAL 1486
#define STEP 5

// CH1=steer  CH2=drive  CH3=weapon  CH4=unused  CH5=arm  CH6=enable
int channel[NUM_CHANNELS] = {NEUTRAL, NEUTRAL, 1000, NEUTRAL, 1800, 2000};

void setup() {
  pinMode(PPM_PIN, OUTPUT);
  digitalWrite(PPM_PIN, HIGH);
  Serial.begin(9600);
  Serial.println("FlySky Trainer Mode - PPM Generator Ready");
}

void loop() {
  // ── Serial command processing ─────────────────────────────────────────────
  while (Serial.available() > 0) {
    char val = Serial.read();

    switch (val) {
      case 'w':
        if (channel[1] < 2000) { channel[1] += STEP; }
        if (channel[1] > 2000)   channel[1] = 2000;
        break;
      case 's':
        if (channel[1] > 1000) { channel[1] -= STEP; }
        if (channel[1] < 1000)   channel[1] = 1000;
        break;
      case 'd':
        if (channel[0] < 2000) { channel[0] += STEP; }
        if (channel[0] > 2000)   channel[0] = 2000;
        break;
      case 'a':
        if (channel[0] > 1000) { channel[0] -= STEP; }
        if (channel[0] < 1000)   channel[0] = 1000;
        break;
      case 'x':
        channel[2] = 2000;
        break;
      case 'z':
        channel[2] = 1000;
        break;
      case 'c':
        channel[0] = NEUTRAL;
        channel[1] = NEUTRAL;
        break;
      case ' ':
        channel[0] = NEUTRAL;
        channel[1] = NEUTRAL;
        channel[2] = 1000;
        break;
    }
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
