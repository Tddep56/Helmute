#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioSynthNoiseWhite     noise;          
AudioFilterBiquad        bandpass;        
AudioInputI2SQuad        i2s_quadIn;      
AudioAmplifier           ampNoise;        
AudioAmplifier           ampLB;           // LB is error mic
AudioOutputI2S           i2sOut;          
AudioAnalyzeFFT1024      fftOutput;      
AudioAnalyzeFFT1024      fftInput;       
AudioAnalyzeRMS          rmsInput;        
AudioAnalyzeRMS          rmsNoise;        
AudioConnection          patchCord1(noise, bandpass);
AudioConnection          patchCord2(bandpass, ampNoise);
AudioConnection          patchCord3(ampNoise, 0, i2sOut, 0);  // Left channel
AudioConnection          patchCord4(ampNoise, 0, i2sOut, 1);  // Right channel
AudioConnection          patchCord5(i2s_quadIn, 1, ampLB, 0); 
AudioConnection          patchCord6(ampNoise, fftInput);
AudioConnection          patchCord7(ampLB, fftOutput);
AudioConnection          patchCord8(i2s_quadIn, 1, rmsInput, 0);
AudioConnection          patchCord9(ampNoise, 0, rmsNoise, 0);
AudioControlSGTL5000     sgtl5000;

// Constants
#define SDCARD_CS_PIN BUILTIN_SDCARD

// Arrays to store frequency response
float inputSpectrum[256] = {0};
float outputSpectrum[256] = {0};
float avgTransferFunction[256] = {0};
float baselineLevel = 0.0;


const int numMeasurements = 5;
int currentMeasurement = 0;
bool measurementActive = false;
unsigned long startTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {} // Wait for serial connection
  
  AudioMemory(512);
  
  // Initialize SD 
  Serial.print("Initializing SD card...");
  if (!SD.begin(SDCARD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialization done.");

  // Configure audio shield
  if (!sgtl5000.enable()) {
    Serial.println("SGTL5000 initialization failed!");
    while (1);
  }
  Serial.println("SGTL5000 initialization successful");
  
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000.lineInLevel(15);  // Most sensitive setting 
  sgtl5000.volume(0.8);
  sgtl5000.lineOutLevel(29); // 1.29 Vpp
  sgtl5000.unmuteLineout();
  
  // Configure bandpass filter (200-800 Hz)
  bandpass.setHighpass(0, 200, 0.707);
  bandpass.setLowpass(1, 800, 0.707);
  
  // Initialize arrays
  for (int i = 0; i < 256; i++) {
    avgTransferFunction[i] = 0.0;
  }
  
  // System stabilization delay
  delay(1000);
  
  // Measure baseline noise level
  Serial.println("Measuring baseline noise level...");
  float totalBaseline = 0;
  int samples = 0;
  for (int i = 0; i < 10; i++) {
    if (rmsInput.available()) {
      totalBaseline += rmsInput.read();
      samples++;
    }
    delay(100);
  }
  baselineLevel = (samples > 0) ? totalBaseline / samples : 0;
  Serial.print("Baseline level: ");
  Serial.println(baselineLevel, 6);
  
  // Gradually ramp up noise to avoid sudden loud sounds
  Serial.println("Ramping up noise...");
  for (float amp = 0.0; amp <= 1.0; amp += 0.2) {
    noise.amplitude(amp);
    delay(100);
  }
  
  Serial.println("Starting system identification...");
  startMeasurement();
}

void startMeasurement() {
  measurementActive = true;
  startTime = millis();
  currentMeasurement++;
  
  Serial.print("Starting measurement ");
  Serial.print(currentMeasurement);
  Serial.println(" of 5");
  
  // Reset spectrum array
  for (int i = 0; i < 256; i++) {
    inputSpectrum[i] = 0;
    outputSpectrum[i] = 0;
  }
}

void loop() {
  if (!measurementActive) return;
  
  // Monitor noise levels
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    if (rmsInput.available() && rmsNoise.available()) {
      Serial.print("Input RMS: ");
      Serial.print(rmsInput.read(), 6);
      Serial.print(" Noise RMS: ");
      Serial.println(rmsNoise.read(), 6);
    }
    lastPrint = millis();
  }
  
  // Check FFTs have data
  if (fftInput.available() && fftOutput.available()) {
    // Accumulate frequency response
    for (int i = 0; i < 256; i++) {
      float input = fftInput.read(i);
      float output = fftOutput.read(i) - baselineLevel;  // Subtract baseline
      if (output < 0) output = 0;  // Prevent negative values
      
      // Accumulate spectrum
      inputSpectrum[i] += input;
      outputSpectrum[i] += output;
    }
  }
  
  // Check if current measurement is complete (1 second)
  if (millis() - startTime >= 1000) {
    processMeasurement();
    
    if (currentMeasurement < numMeasurements) {
      delay(500);  // Wait between measurements
      startMeasurement();
    } else {
      finalizeAndSave();
    }
  }
}

void processMeasurement() {
  // Average and compute transfer function for this measurement
  for (int i = 0; i < 256; i++) {
    if (inputSpectrum[i] > 0.0001) {  // Avoid division by zero
      avgTransferFunction[i] += outputSpectrum[i] / inputSpectrum[i];
    }
  }
}

void finalizeAndSave() {
  noise.amplitude(0);    // Turn off noise
  measurementActive = false;
  
  // Average the measurements
  for (int i = 0; i < 256; i++) {
    avgTransferFunction[i] /= numMeasurements;
  }
  
  // Save to SD card 
  if (SD.exists("SecondaryPath.txt")) {
    SD.remove("SecondaryPath.txt");
  }

  File dataFile = SD.open("SecondaryPath.txt", FILE_WRITE);
  if (dataFile) {
    for (int i = 0; i < 256; i++) {
      dataFile.println(avgTransferFunction[i], 6);
    }
    dataFile.close();
    Serial.println("Transfer function saved to SD card");
  } else {
    Serial.println("Error opening file for writing");
  }
  
  Serial.println("System identification complete");
}
