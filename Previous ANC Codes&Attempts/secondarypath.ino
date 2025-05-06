#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioInputI2SQuad i2s_quadIn;     // Quad I2S input for microphones
AudioSynthWaveform noiseGen;     // Noise generator for speaker input
AudioAmplifier speakerAmp;       // Amplifier for speaker
AudioRecordQueue queueLB;        // Record queue for left error mic
AudioOutputI2S i2sOut;           // I2S output for speaker
AudioConnection patchCord1(noiseGen, 0, speakerAmp, 0);
AudioConnection patchCord2(speakerAmp, 0, i2sOut, 0);
AudioConnection patchCord3(i2s_quadIn, 1, queueLB, 0); // Route channel 1 (LB) to queueLB
// GUItool: end automatically generated code

AudioControlSGTL5000 sgtl5000; // Control for Teensy audio shield

const int taps = 300;           // Tap length of secondary path coefficient
float32_t w[taps] = {0.0};      // Secondary path coefficients
float32_t x[taps] = {0.0};      // Input noise buffer
float32_t mu = 0.0000005;       // Step size
float32_t e, d, y;              // Error, mic input, and estimated mic output
float32_t muEX[taps];           // Helper buffer for coefficient update
File coefFile;                  // File for storing coefficients on SD card

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // Wait for serial monitor to initialize
  }

  // Initialize audio system
  AudioMemory(128);
  sgtl5000.enable();
  sgtl5000.volume(0.8);

  // Initialize SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  SD.remove("SecondaryPath.txt");

  // Initialize noise generator
  noiseGen.begin(0.5, 1000, WAVEFORM_SINE); // Generate white noise for speaker

  // Start recording from the error mic
  queueLB.begin();

  // Clear coefficients
  memset(w, 0, sizeof(w));
  Serial.println("Starting Secondary Path Measurement...");
}

void loop() {
  // Generate random noise and feed to the speaker
  float noise = random(-30000, 30000) / 32768.0; // Generate normalized white noise
  noiseGen.amplitude(noise); // Output noise to the speaker

  // Capture error mic signal
  if (queueLB.available() > 0) {
    int16_t *buffer = queueLB.readBuffer(); // Get audio buffer
    for (int i = 0; i < 128; i++) {        // Process 128 samples
      d = buffer[i] / 32768.0;             // Normalize microphone input
      arm_dot_prod_f32(w, x, taps, &y);    // Estimate mic signal using coefficients
      e = d - y;                           // Calculate error signal

      // Update coefficients
      arm_scale_f32(x, mu * e, muEX, taps);
      arm_add_f32(w, muEX, w, taps);

      // Shift input noise buffer
      for (int j = taps - 1; j > 0; j--) {
        x[j] = x[j - 1];
      }
      x[0] = noise; // Use generated noise as input
    }
    queueLB.freeBuffer(); // Release memory from buffer
  }

  // Stop loop after 5 seconds, save coefficients, and mute speaker
  static unsigned long startTime = millis();
  if (millis() - startTime > 5000) { // Run for 5 seconds
    saveCoefficients();
    
    // Mute the system
    noiseGen.amplitude(0);    // Stop noise generation
    speakerAmp.gain(0);      // Set amplifier gain to 0
    sgtl5000.volume(0);      // Set master volume to 0
    
    Serial.println("Measurement complete. Speaker muted.");
    while (1) {
      // Halt after saving coefficients and muting
    }
  }

  // Print coefficients to serial monitor periodically
  static int counter = 0;
  counter++;
  if (counter >= 100) {
    Serial.print("Coefficients: ");
    for (int i = 0; i < 10; i++) { // Print the first 10 coefficients
      Serial.print(w[i], 6);
      Serial.print(", ");
    }
    Serial.println();
    counter = 0;
  }
}

void saveCoefficients() {
 File file = SD.open("coefficients.txt", FILE_WRITE);
if (file) {
    for (int i = 0; i < NUM_COEFFICIENTS; i++) {
        file.print(i);
        file.print(",");
        file.println(coefficients[i], 6); // Save coefficients with 6 decimal precision
    }
    file.close();
    Serial.println("Coefficients saved to SD card.");
} else {
    Serial.println("Failed to open file for writing.");
}
}