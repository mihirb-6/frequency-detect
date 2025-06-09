#include <arduinoFFT.h>  // Include the ArduinoFFT library for performing Fast Fourier Transforms (FFT)

// === CONFIGURATION ===
// SAMPLES: number of time-domain samples per FFT (must be a power of two for the algorithm)
#define SAMPLES 128
// SAMPLING_FREQUENCY: rate at which we sample the analog input (in Hz)
#define SAMPLING_FREQUENCY 4000   // 4 kHz sample rate gives Nyquist at 2 kHz
// ANALOG_PIN: the analog input pin connected to your filtered/amplified audio signal
#define ANALOG_PIN A0

// === LED PINS ===
// Each LED has a 220 Ω series resistor in hardware, and is connected to one digital pin
const uint8_t RED_LED_PIN   = 6;   // LED for 3rd octave (C3–B3) 8
const uint8_t GREEN_LED_PIN = 5;   // LED for 4th octave (C4–B4) 9
const uint8_t BLUE_LED_PIN  = 3;  // LED for 5th octave (C5–B5) 10

// === THRESHOLDS AND TIMING ===
// Magnitude threshold: ignore any FFT bins with magnitude below this
const double AMPLITUDE_THRESHOLD = 100;
// HOLD_MS: duration (in milliseconds) to keep an LED lit after detecting a peak
const unsigned long HOLD_MS = 200;

// Frequency boundaries for each musical octave band (in Hz)
const double F3_LOW  = 130.81, F3_HIGH = 246.94;  // C3 (130.81 Hz) to B3 (246.94 Hz)
const double F4_LOW  = 261.63, F4_HIGH = 493.88;  // C4 (261.63 Hz) to B4 (493.88 Hz)
const double F5_LOW  = 523.25, F5_HIGH = 987.77;  // C5 (523.25 Hz) to B5 (987.77 Hz)

// === FFT SETUP ===
// Allocate arrays for the real and imaginary parts of the FFT input/output
double vReal[SAMPLES];
double vImag[SAMPLES];
// Construct the FFT object, passing in the arrays, sample count, and sampling frequency
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

// Variables to help pace our sampling loop
unsigned int sampling_period_us;  // microseconds between each analog read
unsigned long microseconds;       // timestamp for current sample

// Variables to implement non-blocking LED "hold" logic
unsigned long lastRedHit   = 0;  // timestamp when red octave was last detected
unsigned long lastGreenHit = 0;  // likewise for green
unsigned long lastBlueHit  = 0;  // likewise for blue

// Simple struct to track the FFT bin index and magnitude of a detected peak
struct Peak {
  uint16_t bin;  // index into the FFT result array
  double   mag;  // magnitude at that bin
};

void setup() {
  // Initialize Serial for debugging output (9600 baud rate)
  Serial.begin(115200);
  // Compute how many microseconds we should wait between samples
  sampling_period_us = round(1000000.0 / SAMPLING_FREQUENCY);

  

  // Configure each LED pin as an output and ensure they're off
  pinMode(RED_LED_PIN,   OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN,  OUTPUT);
  digitalWrite(RED_LED_PIN,   LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN,  LOW);
}

void loop() {

  int sensorValue = analogRead(A0);
  Serial.println(sensorValue);

  // === 1) SAMPLE ACQUISITION ===
  // Fill vReal[] with SAMPLES readings from the analog pin, spaced by sampling_period_us
  for (int i = 0; i < SAMPLES; i++) {
    microseconds = micros();               // record time at start of this sample
    vReal[i] = analogRead(ANALOG_PIN);     // read audio amplitude (0–1023)
    vImag[i] = 0;                          // imaginary part = 0 for real-valued input
    // Busy-wait until it's time for the next sample
    while (micros() - microseconds < sampling_period_us);
  }

  // === 2) FFT PROCESSING ===
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // apply Hamming window to reduce spectral leakage
  FFT.compute(FFT_FORWARD);                        // perform the FFT on vReal & vImag arrays
  FFT.complexToMagnitude();                        // convert complex output to real magnitudes in vReal[]

  // === 3) IDENTIFY TOP 3 PEAKS ===
  // Initialize an array of three Peak structs, sorted by descending magnitude
  Peak peaks[3] = {{0,0},{0,0},{0,0}};
  for (uint16_t bin = 1; bin < SAMPLES/2; bin++) {
    double mag = vReal[bin];  // magnitude of this frequency bin
    // If this magnitude belongs in the top 3, insert it in sorted order
    if (mag > peaks[0].mag) {
      // New highest magnitude: shift old [0] to [1], [1] to [2]
      peaks[2] = peaks[1];
      peaks[1] = peaks[0];
      peaks[0] = {bin, mag};
    } else if (mag > peaks[1].mag) {
      // New second place: shift old [1] to [2]
      peaks[2] = peaks[1];
      peaks[1] = {bin, mag};
    } else if (mag > peaks[2].mag) {
      // New third place
      peaks[2] = {bin, mag};
    }
  }

  // === 4) OCTAVE-BAND DETECTION ===
  bool redDetect   = false;   // flag if any peak lands in 3rd octave
  bool greenDetect = false;   // flag for 4th octave
  bool blueDetect  = false;   // flag for 5th octave
  double bin_width = (double)SAMPLING_FREQUENCY / SAMPLES;  // Hz per FFT bin

  // Check each of the three peaks against our thresholds
  for (int i = 0; i < 3; i++) {
    double freq = peaks[i].bin * bin_width;  // convert bin index to frequency
    // Skip minor peaks below the amplitude threshold
    if (peaks[i].mag < AMPLITUDE_THRESHOLD) continue;

    // Determine which octave band this frequency belongs to
    if (freq >= F3_LOW && freq <= F3_HIGH)   redDetect   = true;
    if (freq >= F4_LOW && freq <= F4_HIGH)   greenDetect = true;
    if (freq >= F5_LOW && freq <= F5_HIGH)   blueDetect  = true;

    // Optional debug: print each valid peak's frequency and magnitude
    Serial.print("Peak: "); Serial.print(freq,1);
    Serial.print(" Hz (mag="); Serial.print(peaks[i].mag,1);
    Serial.println(")");
  }

  // === 5) UPDATE LED HOLD TIMERS ===
  unsigned long now = millis();  // current time in ms
  if (redDetect)   lastRedHit   = now;  // mark time of last red octave detection
  if (greenDetect) lastGreenHit = now;
  if (blueDetect)  lastBlueHit  = now;

  // === 6) DRIVE LEDS WITH HOLD LOGIC ===
  // If the current time minus last hit is less than HOLD_MS, keep LED on
  bool redOn   = (now - lastRedHit   < HOLD_MS);
  bool greenOn = (now - lastGreenHit < HOLD_MS);
  bool blueOn  = (now - lastBlueHit  < HOLD_MS);

  //digitalWrite(RED_LED_PIN,   redOn   ? HIGH : LOW);
  //digitalWrite(GREEN_LED_PIN, greenOn ? HIGH : LOW);
  //digitalWrite(BLUE_LED_PIN,  blueOn  ? HIGH : LOW);

  digitalWrite(RED_LED_PIN,   redOn);
  digitalWrite(GREEN_LED_PIN, greenOn);
  digitalWrite(BLUE_LED_PIN,  blueOn);


  // Short delay to avoid flooding the loop (and Serial) too rapidly
  delay(200);
}
