#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioSynthNoiseWhite     noise;           // White noise generator
AudioFilterBiquad        bandpass;        // Band-pass filter for white noise
AudioOutputI2S           i2sOut;          // I2S output for speakers
AudioInputI2SQuad        i2s_quadIn;      // I2S input for microphones
AudioAnalyzeFFT256       fftOutput;       // FFT for error mic (LB)
AudioConnection          patchCord1(noise, bandpass);
AudioConnection          patchCord2(bandpass, 0, i2sOut, 0);  // Bandpass to left speaker
AudioConnection          patchCord3(bandpass, 0, i2sOut, 1);  // Bandpass to right speaker
AudioConnection          patchCord4(i2s_quadIn, 1, fftOutput, 0); // Error mic (LB) to FFT
AudioControlSGTL5000     sgtl5000;        // Audio shield control
// GUItool: end automatically generated code

#define SDCARD_CS_PIN    BUILTIN_SDCARD

// Variables for FFT bins and transfer function
float averagedBins[256] = {0.0};
const int binStart = 31;
const int binEnd = 33;
const int numMeasurements = 5;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {} // Wait for serial connection
  
  AudioMemory(64);

  // Initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(SDCARD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialized.");

  // Configure SGTL5000 audio shield
  if (!sgtl5000.enable()) {
    Serial.println("SGTL5000 initialization failed!");
    while (1);
  }
  sgtl5000.volume(0.8);
  sgtl5000.lineOutLevel(29); // 1.29 Vpp
  sgtl5000.unmuteLineout();

  // Configure band-pass filter (200-800 Hz)
  bandpass.setHighpass(0, 200, 0.707); 
  bandpass.setLowpass(1, 800, 0.707);

  // System stabilization delay
  delay(1000);
}

void loop() {
  // Array to accumulate measurements
  float transferFunction[256] = {0.0};
  
  Serial.println("Starting secondary path measurement...");
  noise.amplitude(0.8); // Enable white noise
  delay(500);           // Ensure noise has started before measurement
  
  // Perform measurements
  for (int measurement = 0; measurement < numMeasurements; measurement++) {
    Serial.print("Measurement ");
    Serial.print(measurement + 1);
    Serial.println(" of 5...");
    
    unsigned long startTime = millis();
    while (millis() - startTime < 5000) { // Measure for 5 seconds
      if (fftOutput.available()) {
        for (int i = 0; i < 256; i++) {
          transferFunction[i] += fftOutput.read(i); // Accumulate FFT data
        }
      }
    }
    delay(500); // Pause between measurements
  }

  // Average the FFT data
  for (int i = 0; i < 256; i++) {
    transferFunction[i] /= numMeasurements; 
  }

  // Focus on bins 31-33 and print averaged values
  Serial.println("\nAveraged FFT Bins:");
  for (int i = binStart; i <= binEnd; i++) {
    Serial.print("Bin ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(transferFunction[i], 6);
  }

  // Save the transfer function to SD card
  Serial.println("\nSaving to SD card...");
  if (SD.exists("SecondaryPath.txt")) {
    SD.remove("SecondaryPath.txt"); // Remove old file if it exists
  }

  File dataFile = SD.open("SecondaryPath.txt", FILE_WRITE);
  if (dataFile) {
    for (int i = 0; i < 256; i++) {
      dataFile.println(transferFunction[i], 6); // Save with six decimal places
    }
    dataFile.close();
    Serial.println("Transfer function successfully saved.");
  } else {
    Serial.println("Error opening file for writing.");
  }

  noise.amplitude(0); // Disable noise
  Serial.println("Measurement complete. Entering idle state.");
  while (1); // Stop further measurements
}
