#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioSynthNoiseWhite     noise;           //xy=195,543
AudioFilterBiquad        bandpass;        //xy=402,563
AudioInputI2SQuad        i2s_quadIn;      //xy=172,460
AudioAmplifier           ampNoise;        //xy=629,554
AudioAmplifier           ampLB;           //xy=371,424 LB is error mic    
AudioOutputI2S           i2sOut;          //xy=894,456  
AudioAnalyzeFFT1024      fftOutput;      //xy=622,144
AudioAnalyzeFFT1024      fftInput;        //xy=884,657
AudioAnalyzeRMS          rmsInput;        //xy=365,494  monitoring input levels
AudioAnalyzeRMS          rmsNoise;        //xy=857,716  monitoring noise levels
AudioConnection          patchCord1(noise, bandpass);
AudioConnection          patchCord2(bandpass, ampNoise);
AudioConnection          patchCord3(ampNoise, 0, i2sOut, 0);  // Left channel
AudioConnection          patchCord4(ampNoise, 0, i2sOut, 1);  // Right channel
AudioConnection          patchCord5(i2s_quadIn, 1, ampLB, 0);  
AudioConnection          patchCord6(ampNoise, fftInput);
AudioConnection          patchCord7(ampLB, fftOutput);
AudioConnection          patchCord8(i2s_quadIn, 1, rmsInput, 0);
AudioConnection          patchCord9(ampNoise, 0, rmsNoise, 0);
AudioControlSGTL5000     sgtl5000;     //xy=322,150

#define SDCARD_CS_PIN    BUILTIN_SDCARD
#define SDCARD_MOSI_PIN  11
#define SDCARD_SCK_PIN   13

// Arrays to store freq response
float inputSpectrum[256];
float outputSpectrum[256];
float transferFunction[256];
float avgTransferFunction[256];

// To store baseline noise level
float baselineLevel = 0;  

const int numMeasurements = 5;
int currentMeasurement = 0;
bool measurementActive = false;
unsigned long startTime = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {} // Wait for serial connection
  
  AudioMemory(512);
  
  // Initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(SDCARD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialization done.");

  // Configure audio shield
  if (!sgtl5000.enable()) {
    Serial.println("SGTL5000 initialization failed!");
    while(1);
  }
  Serial.println("SGTL5000 initialization successful");
  
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000.lineInLevel(5);  // Changed from 15 to 5 - less sensitive
  Serial.println("Line input configured");
  
  sgtl5000.volume(0.8);
  sgtl5000.lineOutLevel(29); // 1.29 Vpp
  sgtl5000.unmuteLineout();
  
  // Bandpass filter (200-800 Hz)
  bandpass.setHighpass(0, 200, 0.707);
  bandpass.setLowpass(1, 800, 0.707);
  
  // Configure amplifiers with reduced gains
  ampNoise.gain(0.8);  
  ampLB.gain(5);      // Changed from 15 to 5
  
  // Initialize arrays
  for (int i = 0; i < 256; i++) {
    avgTransferFunction[i] = 0.0;
  }
  
  // System stabilization delay
  delay(1000);
  
  // Measure baseline noise 
  Serial.println("Measuring baseline noise...");
  float totalBaseline = 0;
  int samples = 0;
  for(int i = 0; i < 10; i++) {
    if(rmsInput.available()) {
      totalBaseline += rmsInput.read();
      samples++;
    }
    delay(100);
  }
  baselineLevel = (samples > 0) ? totalBaseline / samples : 0;
  Serial.print("Baseline level: ");
  Serial.println(baselineLevel, 6);
  
  // Gradually ramp up noise
  Serial.println("Ramping up noise...");
  for(float amp = 0.0; amp <= 1.0; amp += 0.2) {
    noise.amplitude(amp);
    delay(100);
    if(rmsNoise.available()) {
      Serial.print("Noise level: ");
      Serial.println(rmsNoise.read(), 6);
    }
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
  
  // Reset spectrum arrays
  for (int i = 0; i < 256; i++) {
    inputSpectrum[i] = 0;
    outputSpectrum[i] = 0;
  }
}

void loop() {
  if (!measurementActive) return;
  
  // Monitor levels 
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
  
  // Check both FFTs have data
  if (fftInput.available() && fftOutput.available()) {
    // Accumulate freq response
    for (int i = 0; i < 256; i++) {
      float input = fftInput.read(i);
      float output = fftOutput.read(i) - baselineLevel;  // Subtract baseline
      if (output < 0) output = 0;  // Prevent negative values
      
      // Accumulate spectrum
      inputSpectrum[i] += input;
      outputSpectrum[i] += output;
    }
  }
  
  // Track if current measurement is complete (1 second)
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
    if (inputSpectrum[i] > 0.0001) {  // Avoid divide by zero
      avgTransferFunction[i] += outputSpectrum[i] / inputSpectrum[i];
    }
  }
}

void finalizeAndSave() {
  noise.amplitude(0);        // Turn off noise
  measurementActive = false;

  // Average the measurements
  for (int i = 0; i < 256; i++) {
    avgTransferFunction[i] /= numMeasurements;
  }

  // Print final transfer function values before saving
  Serial.println("\nFinal Transfer Function:");
  for (int i = 0; i < 256; i += 32) {
    Serial.print("Index ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(avgTransferFunction[i], 6);
  }

  // Save to SD card
  Serial.println("Saving transfer function to SD card...");
  if (SD.exists("SecondaryPath.txt")) {
    SD.remove("SecondaryPath.txt"); // Remove old file if it exists
  }

  File dataFile = SD.open("SecondaryPath.txt", FILE_WRITE);
  if (dataFile) {
    for (int i = 0; i < 256; i++) {
      dataFile.println(avgTransferFunction[i], 6); // Save with six decimal places
    }
    dataFile.close();
    Serial.println("Transfer function successfully saved to SD card.");
  } else {
    Serial.println("Error opening file for writing.");
  }

  Serial.println("System identification complete.");
}