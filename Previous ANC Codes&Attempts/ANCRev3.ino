#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <Adafruit_TPA2016.h>

// GUItool: begin automatically generated code
AudioInputI2SQuad i2s_quadIn;            // Input from 4 microphones (2 on each side)
AudioAmplifier ampLOut;                  // Left speaker output amplifier
AudioAmplifier ampROut;                  // Right speaker output amplifier
AudioAmplifier ampLA;                    // Left reference mic amplifier
AudioAmplifier ampLB;                    // Left error mic amplifier
AudioAmplifier ampRA;                    // Right reference mic amplifier
AudioAmplifier ampRB;                    // Right error mic amplifier
AudioFilterBiquad bandpassRefL;          // Bandpass filter for left reference mic (200-2000 Hz)
AudioFilterBiquad bandpassErrorL;        // Bandpass filter for left error mic (200-2000 Hz)
AudioFilterBiquad bandpassRefR;          // Bandpass filter for right reference mic (200-2000 Hz)
AudioFilterBiquad bandpassErrorR;        // Bandpass filter for right error mic (200-2000 Hz)
AudioOutputI2S i2sOut;                   // I2S output to speakers
AudioAnalyzeRMS rmsRefL;                 // RMS analyzer for left reference mic
AudioAnalyzeRMS rmsErrorL;               // RMS analyzer for left error mic
AudioConnection patchCord1(i2s_quadIn, 0, ampLA, 0);
AudioConnection patchCord2(i2s_quadIn, 1, ampLB, 0);
AudioConnection patchCord3(i2s_quadIn, 2, ampRA, 0);
AudioConnection patchCord4(i2s_quadIn, 3, ampRB, 0);
AudioConnection patchCord5(ampLOut, 0, i2sOut, 0);
AudioConnection patchCord6(ampROut, 0, i2sOut, 1);
AudioConnection patchCord7(ampLA, bandpassRefL); 
AudioConnection patchCord8(ampLB, bandpassErrorL);
AudioConnection patchCord9(bandpassRefL, rmsRefL);
AudioConnection patchCord10(bandpassErrorL, rmsErrorL);
AudioControlSGTL5000 sgtl5000;
// GUItool: end automatically generated code

Adafruit_TPA2016 speakerAmp = Adafruit_TPA2016();




void setup() {
    Serial.begin(115200);
    AudioMemory(128);
    sgtl5000.enable();
    sgtl5000.volume(0.8);
    sgtl5000.lineOutLevel(29);       // Set line out to 1.29 Vpp
    sgtl5000.unmuteLineout();
    
    // Speaker and mic gain settings
    ampLOut.gain(1.0);
    ampROut.gain(1.0);
    ampLA.gain(10);
    ampLB.gain(10);
    ampRA.gain(10);
    ampRB.gain(10);
    
    // Configure speaker amplifier
    speakerAmp.begin();
    speakerAmp.enableChannel(true, true);
    speakerAmp.setGain(10);
    speakerAmp.setLimitLevelOff();

    // Configure bandpass filters for 200-2000 Hz
    bandpassRefL.setBandpass(0, 1000.0, 1.0);    // Reference left
    bandpassErrorL.setBandpass(0, 1000.0, 1.0);  // Error left
}

void loop() {
    // Check if RMS values are available after bandpass filtering
    if (rmsRefL.available() && rmsErrorL.available()) {
        float reference_signal = rmsRefL.read();
        float error_signal = rmsErrorL.read();

        Serial.print(" LA (500Hz): ");
        Serial.print(reference_signal, 3);
        Serial.print("\t LB (500Hz): ");
        Serial.println(error_signal, 3);

        // Update the adaptive filter 
        float mu = 0.01;                // Step size for LMS
        float correction = mu * reference_signal * error_signal;
        
        leftOutputGain -= correction;
        rightOutputGain -= correction;
        
        // Update gains 
        leftOutputGain = constrain(leftOutputGain, 0.0, 1.0);
        rightOutputGain = constrain(rightOutputGain, 0.0, 1.0);
        ampLOut.gain(leftOutputGain);
        ampROut.gain(rightOutputGain);
    }
}
