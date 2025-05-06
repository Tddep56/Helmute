#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioSynthNoiseWhite     noise;          
AudioFilterBiquad        bandpass;        
AudioAmplifier           ampNoise;        
AudioInputI2SQuad        i2s_quadIn;      
AudioAmplifier           ampLB;           
AudioOutputI2S           i2sOut;          
AudioAnalyzeFFT256       fftInput;        
AudioAnalyzeFFT256       fftOutput;       
AudioConnection          patchCord1(noise, bandpass);
AudioConnection          patchCord2(bandpass, ampNoise);
AudioConnection          patchCord3(ampNoise, 0, i2sOut, 0);
AudioConnection          patchCord4(ampNoise, 0, i2sOut, 1);
AudioConnection          patchCord5(i2s_quadIn, 1, ampLB, 0);
AudioConnection          patchCord6(ampNoise, fftInput);
AudioConnection          patchCord7(ampLB, fftOutput);
AudioControlSGTL5000     sgtl5000;

#define SDCARD_CS_PIN    BUILTIN_SDCARD

// Measurement constants and variables
const int numMeasurements = 5;
const int numBins = 256;
int currentMeasurement = 0;
bool measurementActive = false;
unsigned long startTime = 0;
unsigned long lastPrintTime = 0;

// Arrays for transfer function calculation
float inputSpectrumMag[256];
float outputSpectrumMag[256];
float transferFunctionMag[256];
float transferFunctionPhase[256];
float transferFunctionPhaseUnwrapped[256];
float phaseMeasurements[5][256];  // Store individual measurements for statistics
float prevPhase[256];  // For phase unwrapping
float phaseStdDev[256];  // For phase statistics

// Function to unwrap phase
float unwrapPhase(float newPhase, float prevPhase) {
    float diff = newPhase - prevPhase;
    while (diff > PI) {
        diff -= 2 * PI;
    }
    while (diff < -PI) {
        diff += 2 * PI;
    }
    return prevPhase + diff;
}

// Function to calculate standard deviation
float calculateStdDev(float values[], int n, float mean) {
    if (n < 2) return 0.0;
    float variance = 0.0;
    for (int i = 0; i < n; i++) {
        float diff = values[i] - mean;
        variance += diff * diff;
    }
    return sqrt(variance / (n - 1));
}

void setup() {
    Serial.begin(115200);
    AudioMemory(512);
    
    // Initialize SD card
    if (!SD.begin(SDCARD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
        while (1);
    }
    Serial.println("SD card initialized successfully");
    
    // Configure audio shield
    sgtl5000.enable();
    sgtl5000.volume(0.8);
    sgtl5000.lineOutLevel(29);
    
    // Configure bandpass filter (200-800 Hz)
    bandpass.setHighpass(0, 200, 0.707);
    bandpass.setLowpass(1, 800, 0.707);
    
    // Configure amplifiers
    ampNoise.gain(0.5);
    ampLB.gain(10);
    
    // Set window function for FFTs
    fftInput.windowFunction(AudioWindowHanning256);
    fftOutput.windowFunction(AudioWindowHanning256);
    
    // Initialize arrays
    for (int i = 0; i < numBins; i++) {
        inputSpectrumMag[i] = 0.0;
        outputSpectrumMag[i] = 0.0;
        transferFunctionMag[i] = 0.0;
        transferFunctionPhase[i] = 0.0;
        transferFunctionPhaseUnwrapped[i] = 0.0;
        prevPhase[i] = 0.0;
        phaseStdDev[i] = 0.0;
        for (int j = 0; j < numMeasurements; j++) {
            phaseMeasurements[j][i] = 0.0;
        }
    }
    
    delay(1000);  // System stabilization
    
    Serial.println("Starting noise source...");
    noise.amplitude(1.0);
    delay(500);
    startMeasurement();
}

void startMeasurement() {
    measurementActive = true;
    startTime = millis();
    currentMeasurement++;
    
    Serial.println("\nStarting measurement " + String(currentMeasurement) + " of " + String(numMeasurements));
    Serial.println("Bin\tFreq(Hz)\tMag\tPhase(rad)\tUnwrapped Phase(rad)");
    
    // Reset spectrum arrays for new measurement
    for (int i = 0; i < numBins; i++) {
        inputSpectrumMag[i] = 0;
        outputSpectrumMag[i] = 0;
    }
}

void loop() {
    if (!measurementActive) return;
    
    // Print real-time FFT data every 200ms
    if (millis() - lastPrintTime > 200) {
        if (fftInput.available() && fftOutput.available()) {
            // Process bins around 516 Hz (bins 2-4)
            for (int i = 2; i <= 4; i++) {
                float freqHz = i * 172.0;
                float inReal = fftInput.output[i*2];
                float inImag = fftInput.output[i*2 + 1];
                float outReal = fftOutput.output[i*2];
                float outImag = fftOutput.output[i*2 + 1];
                
                // Calculate magnitude and phase
                float inMag = sqrt(inReal*inReal + inImag*inImag);
                float outMag = sqrt(outReal*outReal + outImag*outImag);
                float magTF = (inMag > 0.0001) ? outMag/inMag : 0;
                
                float inPhase = atan2(inImag, inReal);
                float outPhase = atan2(outImag, outReal);
                float phaseDiff = outPhase - inPhase;
                float phaseUnwrapped = unwrapPhase(phaseDiff, prevPhase[i]);
                
                // Accumulate for averaging
                inputSpectrumMag[i] += inMag;
                outputSpectrumMag[i] += outMag;
                
                Serial.print(i);
                Serial.print("\t");
                Serial.print(freqHz, 0);
                Serial.print("\t\t");
                Serial.print(magTF, 6);
                Serial.print("\t");
                Serial.print(phaseDiff, 6);
                Serial.print("\t");
                Serial.println(phaseUnwrapped, 6);
            }
            Serial.println();
        }
        lastPrintTime = millis();
    }
    
    // Check if current measurement is complete (1 second duration)
    if (millis() - startTime >= 1000) {
        processMeasurement();
        
        if (currentMeasurement < numMeasurements) {
            delay(100);
            startMeasurement();
        } else {
            finalizeAndSave();
        }
    }
}

void processMeasurement() {
    Serial.println("\nMeasurement " + String(currentMeasurement) + " Summary:");
    
    // Process and store measurement data
    for (int i = 2; i <= 4; i++) {
        float freqHz = i * 172.0;
        if (inputSpectrumMag[i] > 0.0001) {
            float inReal = fftInput.output[i*2];
            float inImag = fftInput.output[i*2 + 1];
            float outReal = fftOutput.output[i*2];
            float outImag = fftOutput.output[i*2 + 1];
            
            float magTF = outputSpectrumMag[i] / inputSpectrumMag[i];
            float inPhase = atan2(inImag, inReal);
            float outPhase = atan2(outImag, outReal);
            float phaseDiff = outPhase - inPhase;
            
            // Phase unwrapping
            if (currentMeasurement == 1) {
                prevPhase[i] = phaseDiff;
                transferFunctionPhaseUnwrapped[i] = phaseDiff;
            } else {
                transferFunctionPhaseUnwrapped[i] = unwrapPhase(phaseDiff, prevPhase[i]);
                prevPhase[i] = transferFunctionPhaseUnwrapped[i];
            }
            
            transferFunctionMag[i] += magTF;
            phaseMeasurements[currentMeasurement-1][i] = transferFunctionPhaseUnwrapped[i];
            
            Serial.print("Freq: ");
            Serial.print(freqHz, 0);
            Serial.print(" Hz, Mag: ");
            Serial.print(magTF, 6);
            Serial.print(", Phase (wrapped): ");
            Serial.print(phaseDiff, 6);
            Serial.print(", Phase (unwrapped): ");
            Serial.println(transferFunctionPhaseUnwrapped[i], 6);
        }
    }
}

void finalizeAndSave() {
    noise.amplitude(0);
    measurementActive = false;

    Serial.println("\nFinal Transfer Function Summary:");
    Serial.println("Freq(Hz)\tMagnitude\tPhase(rad)\tUnwrapped Phase(rad)\tPhase StdDev");
    
    // Calculate final averages and statistics
    for (int i = 0; i < numBins; i++) {
        if (i >= 2 && i <= 4) {
            float phaseSum = 0;
            float phases[numMeasurements];
            
            transferFunctionMag[i] /= numMeasurements;
            
            // Calculate phase statistics
            for (int j = 0; j < numMeasurements; j++) {
                phases[j] = phaseMeasurements[j][i];
                phaseSum += phases[j];
            }
            
            float phaseMean = phaseSum / numMeasurements;
            phaseStdDev[i] = calculateStdDev(phases, numMeasurements, phaseMean);
            
            // Store final phase values
            transferFunctionPhase[i] = fmod(phaseMean, 2*PI);  // Wrapped
            transferFunctionPhaseUnwrapped[i] = phaseMean;     // Unwrapped
            
            float freqHz = i * 172.0;
            Serial.print(freqHz, 0);
            Serial.print("\t");
            Serial.print(transferFunctionMag[i], 6);
            Serial.print("\t");
            Serial.print(transferFunctionPhase[i], 6);
            Serial.print("\t");
            Serial.print(transferFunctionPhaseUnwrapped[i], 6);
            Serial.print("\t");
            Serial.println(phaseStdDev[i], 6);
        }
    }

    // Save to SD card
    if (SD.exists("SecondaryPath.txt")) {
        SD.remove("SecondaryPath.txt");
        Serial.println("\nRemoved existing SecondaryPath.txt");
    }

    File dataFile = SD.open("SecondaryPath.txt", FILE_WRITE);
    if (dataFile) {
        dataFile.println("Bin\tFreq(Hz)\tMagnitude\tPhase(rad)\tUnwrapped Phase(rad)\tPhase StdDev");
        int valuesSaved = 0;
        for (int i = 0; i < numBins; i++) {
            dataFile.print(i);
            dataFile.print("\t");
            dataFile.print(i * 172.0);
            dataFile.print("\t");
            dataFile.print(transferFunctionMag[i], 6);
            dataFile.print("\t");
            dataFile.print(transferFunctionPhase[i], 6);
            dataFile.print("\t");
            dataFile.print(transferFunctionPhaseUnwrapped[i], 6);
            dataFile.print("\t");
            dataFile.println(phaseStdDev[i], 6);
            valuesSaved++;
        }
        dataFile.close();
        Serial.println("Transfer function saved: " + String(valuesSaved) + " values written");
        
        // Verify key frequencies
        Serial.println("\nVerifying saved data for key frequencies:");
        dataFile = SD.open("SecondaryPath.txt");
        if (dataFile) {
            String header = dataFile.readStringUntil('\n');
            Serial.println(header);
            for (int i = 0; i < 5; i++) {
                String line = dataFile.readStringUntil('\n');
                Serial.println(line);
            }
            dataFile.close();
        }
    } else {
        Serial.println("Error opening file for writing!");
    }

    Serial.println("\nMeasurement complete.");
}