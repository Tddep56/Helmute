#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// Structure to hold/store transfer function data
struct TransferFunctionData {
    float frequency;
    float magnitude;
    float phase;            
    float unwrappedPhase;   
};

// Structure to store FFT components
struct FFTComponents {
    float real;
    float imag;
};

// GUItool: begin automatically generated code
AudioSynthNoiseWhite     noise;          
AudioFilterBiquad        bandpass;        
AudioAmplifier           ampNoise;        
AudioInputI2SQuad        i2s_quadIn;     
AudioAmplifier           ampLB;          
AudioAmplifier           ampRB;          
AudioOutputI2S           i2sOut;          
AudioAnalyzeFFT256       fftInput;        
AudioAnalyzeFFT256       fftOutput;      
AudioConnection          patchCord1(noise, bandpass);
AudioConnection          patchCord2(bandpass, ampNoise);
AudioConnection          patchCord3(ampNoise, 0, i2sOut, 0);
AudioConnection          patchCord4(ampNoise, 0, i2sOut, 1);
AudioConnection          patchCord5(i2s_quadIn, 1, ampLB, 0);
AudioConnection          patchCord6(i2s_quadIn, 3, ampRB, 0);
AudioConnection          patchCord7(ampNoise, fftInput);
AudioConnection          patchCord8(ampLB, fftOutput);
AudioControlSGTL5000     sgtl5000;       

#define SDCARD_CS_PIN    BUILTIN_SDCARD

// Global constants and variables
const int numMeasurements = 5;     
const int numBins = 256;           
int currentMeasurement = 0;        
bool measurementActive = false;
bool measuringLeftSide = true;    // New flag to track which side we're measuring
unsigned long startTime = 0;       

// Arrays to store data
float inputMag[256];
float outputMag[256];
float tfMag[256];
float tfPhase[256];
float tfPhaseUnwrapped[256];
float prevPhase[256];

FFTComponents inputComponents[256];
FFTComponents outputComponents[256];

// Function to unwrap phase
float unwrapPhase(float newPhase, float prevPhase) {
    float diff = newPhase - prevPhase;
    while (diff > PI) diff -= 2 * PI;
    while (diff < -PI) diff += 2 * PI;
    return prevPhase + diff;
}

void setup() {
    Serial.begin(115200);
    AudioMemory(512);
    
    if (!SD.begin(SDCARD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
        while (1);
    }
    Serial.println("SD card initialized successfully");
    
    sgtl5000.enable();
    sgtl5000.volume(0.7);
    sgtl5000.lineOutLevel(29);
    
    bandpass.setHighpass(0, 200, 0.707);
    bandpass.setLowpass(1, 2200, 0.707);
    
    ampNoise.gain(0.5);
    ampLB.gain(10);
    ampRB.gain(10);
    
    fftInput.windowFunction(AudioWindowHanning256);
    fftOutput.windowFunction(AudioWindowHanning256);
    
    for (int i = 0; i < numBins; i++) {
        inputMag[i] = 0.0;
        outputMag[i] = 0.0;
        tfMag[i] = 0.0;
        tfPhase[i] = 0.0;
        tfPhaseUnwrapped[i] = 0.0;
        prevPhase[i] = 0.0;
        inputComponents[i] = {0.0, 0.0};
        outputComponents[i] = {0.0, 0.0};
    }
    
    delay(500);
    
    Serial.println("Starting Left Side Measurements");
    noise.amplitude(1.0);
    delay(500);
    startMeasurement();
}

void startMeasurement() {
    measurementActive = true;
    startTime = millis();
    currentMeasurement++;
    
    Serial.println("\nStarting measurement " + String(currentMeasurement) + " of " + String(numMeasurements) + 
                  (measuringLeftSide ? " (Left Side)" : " (Right Side)"));
    
    for (int i = 0; i < numBins; i++) {
        inputMag[i] = 0;
        outputMag[i] = 0;
        inputComponents[i] = {0.0, 0.0};
        outputComponents[i] = {0.0, 0.0};
    }
}

void loop() {
    if (!measurementActive) return;
    
    if (fftInput.available() && fftOutput.available()) {
        for (int i = 1; i <= 11; i++) {
            inputComponents[i].real = fftInput.output[i*2];
            inputComponents[i].imag = fftInput.output[i*2 + 1];
            outputComponents[i].real = fftOutput.output[i*2];
            outputComponents[i].imag = fftOutput.output[i*2 + 1];
            
            float inMag = sqrt(inputComponents[i].real * inputComponents[i].real + 
                             inputComponents[i].imag * inputComponents[i].imag);
            float outMag = sqrt(outputComponents[i].real * outputComponents[i].real + 
                              outputComponents[i].imag * outputComponents[i].imag);
            
            inputMag[i] += inMag;
            outputMag[i] += outMag;
        }
    }
    
    if (millis() - startTime >= 1000) {
        processMeasurement();
        
        if (currentMeasurement < numMeasurements) {
            delay(100);
            startMeasurement();
        } else {
            finalizeAndSave();
            
            if (measuringLeftSide) {
                // Switch to right side
                measuringLeftSide = false;
                currentMeasurement = 0;
                patchCord8.disconnect();
                patchCord8.connect(ampRB, fftOutput);
                Serial.println("\nStarting Right Side Measurements");
                delay(500);
                startMeasurement();
            } else {
                noise.amplitude(0);
                measurementActive = false;
                Serial.println("\nAll measurements complete!");
            }
        }
    }
}

void processMeasurement() {
    Serial.println("\nMeasurement " + String(currentMeasurement) + 
                  (measuringLeftSide ? " (Left Side)" : " (Right Side)") + " Summary:");
    
    for (int i = 1; i <= 11; i++) {
        float freqHz = i * 172.0;
        if (inputMag[i] > 0.0001) {
            float magTF = outputMag[i] / inputMag[i];
            float inPhase = atan2(inputComponents[i].imag, inputComponents[i].real);
            float outPhase = atan2(outputComponents[i].imag, outputComponents[i].real);
            float phaseDiff = outPhase - inPhase;
            
            if (currentMeasurement == 1) {
                prevPhase[i] = phaseDiff;
                tfPhaseUnwrapped[i] = phaseDiff;
            } else {
                tfPhaseUnwrapped[i] = unwrapPhase(phaseDiff, prevPhase[i]);
                prevPhase[i] = tfPhaseUnwrapped[i];
            }
            
            tfMag[i] += magTF;
            
            Serial.print("Freq: ");
            Serial.print(freqHz, 0);
            Serial.print(" Hz, Mag: ");
            Serial.print(magTF, 6);
            Serial.print(", Phase: ");
            Serial.println(tfPhaseUnwrapped[i], 6);
        }
    }
}

void finalizeAndSave() {
    const char* filename = measuringLeftSide ? "SecondaryPathL.txt" : "SecondaryPathR.txt";
    
    Serial.println("\nFinal Transfer Function Summary for ");
    Serial.println(measuringLeftSide ? "Left Side:" : "Right Side:");
    Serial.println("Freq(Hz)\tMagnitude\tPhase(rad)\tUnwrapped Phase(rad)");
    
    for (int i = 0; i < numBins; i++) {
        if (i >= 1 && i <= 11) {
            tfMag[i] /= numMeasurements;
            tfPhase[i] = fmod(tfPhaseUnwrapped[i], 2*PI);
            
            float freqHz = i * 172.0;
            Serial.print(freqHz, 0);
            Serial.print("\t");
            Serial.print(tfMag[i], 6);
            Serial.print("\t");
            Serial.print(tfPhase[i], 6);
            Serial.print("\t");
            Serial.println(tfPhaseUnwrapped[i], 6);
        }
    }

    if (SD.exists(filename)) {
        SD.remove(filename);
    }

    File dataFile = SD.open(filename, FILE_WRITE);
    if (dataFile) {
        dataFile.println("Bin\tFreq\tMagnitude\tPhase\tUnwrapped Phase");
        
        int valuesSaved = 0;
        for (int i = 1; i <= 11; i++) {
            dataFile.print(i);
            dataFile.print("\t");
            dataFile.print(i * 172.0);
            dataFile.print("\t");
            dataFile.print(tfMag[i], 6);
            dataFile.print("\t");
            dataFile.print(tfPhase[i], 6);
            dataFile.print("\t");
            dataFile.println(tfPhaseUnwrapped[i], 6);
            valuesSaved++;
        }
        dataFile.close();
        
        dataFile = SD.open(filename);
        if (dataFile) {
            Serial.println("\nVerifying saved data:");
            while (dataFile.available()) {
                String line = dataFile.readStringUntil('\n');
                Serial.println(line);
            }
            dataFile.close();
        }
        Serial.print("\nTransfer function saved to ");
        Serial.print(filename);
        Serial.print(": ");
        Serial.print(valuesSaved);
        Serial.println(" values written");
    } else {
        Serial.println("Error opening file for writing!");
    }
}
