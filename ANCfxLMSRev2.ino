#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// Secondary path transfer function values at 516 Hz
const float S_MAGNITUDE = 2.475724;  
const float S_PHASE = -0.026595;     

// FxLMS parameters
const float MU = 0.005;              // Increased for faster convergence
const float LEAK_FACTOR = 0.9999;    // Reduced leakage
float w[2] = {0.0, 0.0};           

// Audio setup
AudioInputI2SQuad     i2s_quadIn;      
AudioSynthWaveform    controlSignal;    
AudioOutputI2S        i2s_out;          
AudioAmplifier        amp_ctrl;         
AudioAnalyzeFFT256    fft_ref;          
AudioAnalyzeFFT256    fft_error;        
AudioConnection       patchCord1(i2s_quadIn, 0, fft_ref, 0);  // Reference mic
AudioConnection       patchCord2(i2s_quadIn, 1, fft_error, 0); // Error mic
AudioConnection       patchCord3(controlSignal, amp_ctrl);
AudioConnection       patchCord4(amp_ctrl, 0, i2s_out, 0);
AudioConnection       patchCord5(amp_ctrl, 0, i2s_out, 1);
AudioControlSGTL5000  sgtl5000;

// Variables for signal processing
float filtered_x_I = 0.0;
float filtered_x_Q = 0.0;
float error_I = 0.0;    
float error_Q = 0.0;    
float ref_I = 0.0;     
float ref_Q = 0.0;     

void setup() {
    Serial.begin(115200);
    AudioMemory(512);
    
    // Configure audio shield
    sgtl5000.enable();
    sgtl5000.volume(0.8);
    sgtl5000.lineOutLevel(29);
    
    // Initialize control signal
    controlSignal.begin(0.0, 516, WAVEFORM_SINE);
    
    // Configure FFTs
    fft_ref.windowFunction(AudioWindowHanning256);
    fft_error.windowFunction(AudioWindowHanning256);
    
    // Set amplifier gain
    amp_ctrl.gain(1.0);  // Full gain
    
    delay(1000);
}

void loop() {
    if (fft_ref.available() && fft_error.available()) {
        const int targetBin = 3;  // 516 Hz bin
        
        // Extract I/Q components
        ref_I = fft_ref.output[targetBin * 2];
        ref_Q = fft_ref.output[targetBin * 2 + 1];
        error_I = fft_error.output[targetBin * 2];
        error_Q = fft_error.output[targetBin * 2 + 1];
        
        // Filter reference through secondary path model
        filtered_x_I = ref_I * S_MAGNITUDE * cos(S_PHASE) - 
                      ref_Q * S_MAGNITUDE * sin(S_PHASE);
        filtered_x_Q = ref_Q * S_MAGNITUDE * cos(S_PHASE) + 
                      ref_I * S_MAGNITUDE * sin(S_PHASE);
        
        // Update weights using FxLMS
        w[0] = LEAK_FACTOR * w[0] - MU * (filtered_x_I * error_I + filtered_x_Q * error_Q);
        w[1] = LEAK_FACTOR * w[1] - MU * (filtered_x_Q * error_I - filtered_x_I * error_Q);
        
        // Calculate and update control signal
        float amplitude = sqrt(w[0]*w[0] + w[1]*w[1]);
        float phase = atan2(w[1], w[0]);
        controlSignal.amplitude(amplitude);
        controlSignal.phase(phase);
        
        // Print debug information every 100ms
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 100) {
            Serial.print("Weights: ");
            Serial.print(w[0], 6);
            Serial.print(", ");
            Serial.print(w[1], 6);
            Serial.print(" Error Mag: ");
            Serial.print(sqrt(error_I*error_I + error_Q*error_Q), 6);
            Serial.print(" Control Mag: ");
            Serial.println(amplitude, 6);
            lastPrint = millis();
        }
    }
}