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
    float variance = 0.0;
    for (int i = 0; i < n; i++) {
        variance += pow(values[i] - mean, 2);
    }
    return sqrt(variance / (n - 1));  // n-1 for sample standard deviation
}

void processMeasurement() {
    Serial.println("\nMeasurement " + String(currentMeasurement) + " Summary:");
    
    // Arrays for phase statistics
    static float phaseValues[5];
    
    // Compute and display transfer function for key frequencies
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
            
            // Unwrap phase
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
    for (int i = 0; i < 256; i++) {
        if (i >= 2 && i <= 4) {  // Only process our frequency range of interest
            transferFunctionMag[i] /= numMeasurements;
            
            // Calculate phase statistics
            float phaseSum = 0;
            float phases[numMeasurements];
            for (int j = 0; j < numMeasurements; j++) {
                phases[j] = phaseMeasurements[j][i];
                phaseSum += phases[j];
            }
            float phaseMean = phaseSum / numMeasurements;
            phaseStdDev[i] = calculateStdDev(phases, numMeasurements, phaseMean);
            
            transferFunctionPhase[i] = fmod(phaseMean, 2*PI);  // Wrapped phase
            transferFunctionPhaseUnwrapped[i] = phaseMean;     // Unwrapped phase
            
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
        for (int i = 0; i < 256; i++) {
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
    }

    Serial.println("\nMeasurement complete.");
}