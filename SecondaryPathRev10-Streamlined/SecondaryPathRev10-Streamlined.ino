#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// Structure to hold/store transfer function data, 
struct TransferFunctionData {
    float frequency;
    float magnitude;
    float phase;            //wrapped phase (constrains phase angle between 0 and 2pi, susectible to phase jump, which causes dicontinuities at multiples to 2pi, i.e. 4pi becomes 0)
    float unwrappedPhase;   // unwrapped phase (often used in developing transfer functions [why we are using it here], avoids discontinuities described above)
};

// New structure to store FFT components
struct FFTComponents {
    float real;
    float imag;
};

// GUItool: begin automatically generated code
AudioSynthNoiseWhite     noise;          //xy=139.19998931884766,396.1999702453613
AudioFilterBiquad        bandpass;       //xy=315.1999702453613,404.19997215270996 
AudioAmplifier           ampNoise;       //xy=507.2000045776367,403.2000036239624 
AudioInputI2SQuad        i2s_quadIn;     //xy=154.19998931884766,248.1999855041504
AudioAmplifier           ampLB;          //xy=588.2000045776367,163.1999855041504
AudioOutputI2S           i2sOut;         //xy=838.2000045776367,399.1999702453613 
AudioAnalyzeFFT256       fftInput;       //xy=774.2000045776367,264.1999702453613 
AudioAnalyzeFFT256       fftOutput;      //xy=817.2000045776367,170.1999855041504 
AudioConnection          patchCord1(noise, bandpass);
AudioConnection          patchCord2(bandpass, ampNoise);
AudioConnection          patchCord3(ampNoise, 0, i2sOut, 0);
AudioConnection          patchCord4(ampNoise, 0, i2sOut, 1);
AudioConnection          patchCord5(i2s_quadIn, 1, ampLB, 0);
AudioConnection          patchCord6(ampNoise, fftInput);
AudioConnection          patchCord7(ampLB, fftOutput);
AudioControlSGTL5000     sgtl5000;        //Teensy audio shield

#define SDCARD_CS_PIN    BUILTIN_SDCARD

// Global constants and variables
const int numMeasurements = 5;      // take 5 seperate measurements, to average 
const int numBins = 256;            // 256 bins since we are using FFT256 Teensy Audio Library Function
int currentMeasurement = 0;         //Keep track of how many measurements we have taken
bool measurementActive = false;
unsigned long startTime = 0;        //Tracks time so that eack measurement can be 1 second

// Arrays to store stuff for transfer function calculation again using 256 because we are doing FFT 256
float inputMag[256];
float outputMag[256];
float tfMag[256];
float tfPhase[256];
float tfPhaseUnwrapped[256];
float prevPhase[256];

// New arrays to store FFT components
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
    
    // Initialize SD card
    if (!SD.begin(SDCARD_CS_PIN)) {
        Serial.println("SD card initialization failed!");
        while (1);
    }
    Serial.println("SD card initialized successfully");
    
    // Configure audio shield
    sgtl5000.enable();
    sgtl5000.volume(0.7);
    sgtl5000.lineOutLevel(29);
    
    // Configure bandpass filter (200-1000 Hz)
    bandpass.setHighpass(0, 200, 0.707);
    bandpass.setLowpass(1, 1200, 0.707);
    
    // Configure amplifiers
    ampNoise.gain(0.5);
    ampLB.gain(10);
    
   // Set window function for FFTs prevents spectral leakage, where energy from one frequency spreads into neighboring frequencies
    fftInput.windowFunction(AudioWindowHanning256);
    fftOutput.windowFunction(AudioWindowHanning256);
    
    // Initialize arrays for transfer function calculation
    for (int i = 0; i < numBins; i++) {
        inputMag[i] = 0.0;
        outputMag[i] = 0.0;
        tfMag[i] = 0.0;
        tfPhase[i] = 0.0;
        tfPhaseUnwrapped[i] = 0.0;
        prevPhase[i] = 0.0;
        inputComponents[i] = {0.0, 0.0};  // Initialize FFT components
        outputComponents[i] = {0.0, 0.0};
    }
    
    delay(500);  // For system stabilization
    
    noise.amplitude(1.0);   //Turn on noise
    delay(500);
    startMeasurement();
}

void startMeasurement() {
    measurementActive = true;
    startTime = millis();
    currentMeasurement++;
    
    Serial.println("\nStarting measurement " + String(currentMeasurement) + " of " + String(numMeasurements));
    
    // Reset arrays for new measurement
    for (int i = 0; i < numBins; i++) {
        inputMag[i] = 0;
        outputMag[i] = 0;
        inputComponents[i] = {0.0, 0.0};  // Reset FFT components
        outputComponents[i] = {0.0, 0.0};
    }
}

void loop() {
    if (!measurementActive) return;
    
    if (fftInput.available() && fftOutput.available()) {
        // Process bins around 516 Hz (bins 2-7)
        for (int i = 2; i <= 10; i++) {
            // Store real and imaginary components
            inputComponents[i].real = fftInput.output[i*2];
            inputComponents[i].imag = fftInput.output[i*2 + 1];
            outputComponents[i].real = fftOutput.output[i*2];
            outputComponents[i].imag = fftOutput.output[i*2 + 1];
            
            // Calculate magnitude
            float inMag = sqrt(inputComponents[i].real * inputComponents[i].real + 
                             inputComponents[i].imag * inputComponents[i].imag);
            float outMag = sqrt(outputComponents[i].real * outputComponents[i].real + 
                              outputComponents[i].imag * outputComponents[i].imag);
            
            // Sum for averaging
            inputMag[i] += inMag;
            outputMag[i] += outMag;
        }
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
    
    for (int i = 2; i <= 10; i++) {
        float freqHz = i * 172.0;
        if (inputMag[i] > 0.0001) {
            // Use stored real and imaginary components
            float magTF = outputMag[i] / inputMag[i];
            float inPhase = atan2(inputComponents[i].imag, inputComponents[i].real);
            float outPhase = atan2(outputComponents[i].imag, outputComponents[i].real);
            float phaseDiff = outPhase - inPhase;
            
            // Phase unwrapping
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
    noise.amplitude(0);         //turn off noise for sanity
    measurementActive = false;

    Serial.println("\nFinal Transfer Function Summary:");
    Serial.println("Freq(Hz)\tMagnitude\tPhase(rad)\tUnwrapped Phase(rad)");
    
    // Calculate final averages
    for (int i = 0; i < numBins; i++) {
        if (i >= 2 && i <= 10) {
            tfMag[i] /= numMeasurements;
            tfPhase[i] = fmod(tfPhaseUnwrapped[i], 2*PI);  // Wrapped
            
            //Print to serial for verification
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

    // Save to SD card
    if (SD.exists("SecondaryPath.txt")) {
        SD.remove("SecondaryPath.txt");
    }

    File dataFile = SD.open("SecondaryPath.txt", FILE_WRITE);
    if (dataFile) {
        // Write header
        dataFile.println("Bin\tFreq\tMagnitude\tPhase\tUnwrapped Phase");
        
        //save values iteratively
        int valuesSaved = 0;
        for (int i = 2; i <= 10; i++) {
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
        
        // Verify saved data
        dataFile = SD.open("SecondaryPath.txt");
        if (dataFile) {
            Serial.println("\nVerifying saved data:");
            while (dataFile.available()) {
                String line = dataFile.readStringUntil('\n');
                Serial.println(line);
            }
            dataFile.close();
        }
        Serial.println("\nTransfer function saved: " + String(valuesSaved) + " values written");
    } else {
        Serial.println("Error opening file for writing!");
    }
}