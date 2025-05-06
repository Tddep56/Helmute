#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// GUItool: begin automatically generated code
AudioInputI2S            i2sInput;        // I2S microphone input
AudioAnalyzeFFT256       fft256;          // FFT analyzer (256 bins)
AudioSynthWaveformSine   sineGen;         // Sine wave generator (for cancellation)
AudioOutputI2S           i2sOutput;       // Audio output to speaker
AudioConnection          patchCord1(i2sInput, 0, fft256, 0);
AudioConnection          patchCord2(sineGen, 0, i2sOutput, 0);
AudioConnection          patchCord3(sineGen, 0, i2sOutput, 1);
// GUItool: end automatically generated code

//Constants for determining bins
const float sampleRate = 44100.0;
const int fftSize = 256;
const float binWidth = sampleRate / fftSize;  // ~172.27 Hz per bin

void setup() {
  Serial.begin(9600);
  AudioMemory(12);
  
  // Pure tone generation settings
  sineGen.amplitude(0.5);  // Adjust amplitude as needed
  sineGen.phase(0);        // Start with 0° phase
  
  // Initialize SD card 
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    while (1) delay(1);
  }
  Serial.println("SD card is ready.");
  
  //Erase any previous results.
  SD.remove("phase_results.txt");
  
  Serial.println("Starting measurement.");
}

void loop() {
  // Sweep for bins 0 through 12.
  for (int bin = 1; bin <= 12; bin++) {
    
    // Set the Pure tone frequency to target the current bin
    float toneFreq;
    sineGen.frequency(toneFreq);
    
    Serial.print("Testing FFT bin ");
    Serial.print(bin);
    Serial.print(" with tone frequency: ");
    Serial.print(toneFreq);
    Serial.println(" Hz");
    
    // Variables to record the best phase for given freq
    float bestAmplitude = 1e9;
    float bestPhase = 0;
    
    // Sweep phase from 0 to 180 in 2 deg increments.
    for (float phaseDeg = 0; phaseDeg <= 180; phaseDeg += 2) {
      sineGen.phase(phaseDeg);          // Update phase offset.
      
      Serial.print("  Setting phase: ");
      Serial.print(phaseDeg);
      
      unsigned long startTime = millis();
      int sampleCount = 0;
      float amplitudeSum = 0;
      
      // Sample the FFT for 2 seconds.
      while (millis() - startTime < 2000) {
        if (fft256.available()) {
          float amp = fft256.read(bin);
          amplitudeSum += amp;
          sampleCount++;
          
          Serial.print("    FFT bin ");
          Serial.print(bin);
          Serial.print(" amplitude: ");
          Serial.println(amp);
        }
        delay(10); 
      }
      
      // Compute the average amplitude over 2 seconds
      if (sampleCount > 0) {
        float avgAmp = amplitudeSum / sampleCount;
        Serial.print("    Average amplitude at phase ");
        Serial.print(phaseDeg);
        Serial.print(" deg: ");
        Serial.println(avgAmp);
        
        // Record lowest amplitude 
        if (avgAmp < bestAmplitude) {
          bestAmplitude = avgAmp;
          bestPhase = phaseDeg;
        }
      }
    }
    
    // Output the best phase
    Serial.print("Best phase for bin ");
    Serial.print(bin);
    Serial.print(" (tone ");
    Serial.print(toneFreq);
    Serial.print(" Hz): ");
    Serial.print(bestPhase);
    Serial.print(" degrees with average amplitude: ");
    Serial.println(bestAmplitude);
    
    // Write results to the SD
    File resultsFile = SD.open("phase_results.txt", FILE_WRITE);
    if (resultsFile) {
      resultsFile.print("Bin ");
      resultsFile.print(bin);
      resultsFile.print(", Tone Frequency: ");
      resultsFile.print(toneFreq);
      resultsFile.print(" Hz, Best Phase: ");
      resultsFile.print(bestPhase);
      resultsFile.print(" deg, Avg Amplitude: ");
      resultsFile.println(bestAmplitude);
      resultsFile.close();
      Serial.println("Result saved to SD card.");
    } else {
      Serial.println("Error opening phase_results.txt for writing.");
    }
    
    delay(100);
  }

  // End program when done all sweeps
  Serial.println("Phase sweep for all Frequencies complete.");
  
  while (1) {
    delay(1000);
  }
}
