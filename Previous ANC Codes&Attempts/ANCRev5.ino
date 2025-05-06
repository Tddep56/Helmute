#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// Structure for complex numbers
struct Complex {
    float real;
    float imag;
    
    Complex(float r = 0.0f, float i = 0.0f) : real(r), imag(i) {}
    
    Complex operator*(const Complex& other) const {
        return Complex(real * other.real - imag * other.imag,
                      real * other.imag + imag * other.real);
    }
    
    Complex conj() const {
        return Complex(real, -imag);
    }
};

// Structure to hold/store transfer function data
struct TransferFunctionData {
    float frequency;
    float magnitude;
    float phase;            
    float unwrappedPhase;   
};

// GUItool: begin automatically generated code
AudioInputI2SQuad        i2s_quadIn;     
AudioFilterBiquad        filterRefL;      // High-pass filter for left reference mic
AudioFilterBiquad        filterErrL;      // High-pass filter for left error mic
AudioAmplifier           ampRefL;         // Left reference mic amplifier
AudioAmplifier           ampErrL;         // Left error mic amplifier
AudioSynthNoiseWhite     noise;           // Noise source for anti-noise
AudioFilterBiquad        bandpass;        // Bandpass filter for noise
AudioAmplifier           ampNoise;        // Noise amplitude control
AudioMixer4              mixer;           // Mixer for combining signals
AudioAmplifier           ampOut;          // Final output amplifier
AudioOutputI2S           i2sOut;         
AudioAnalyzeFFT256       fftRef;          // FFT for reference signal
AudioAnalyzeFFT256       fftErr;          // FFT for error signal
AudioConnection          patchCord1(i2s_quadIn, 0, filterRefL, 0);  // Left reference mic to filter
AudioConnection          patchCord2(i2s_quadIn, 1, filterErrL, 0);  // Left error mic to filter
AudioConnection          patchCord3(filterRefL, ampRefL);           // Filter to amp
AudioConnection          patchCord4(filterErrL, ampErrL);           // Filter to amp
AudioConnection          patchCord5(ampRefL, fftRef);               // Amp to FFT
AudioConnection          patchCord6(ampErrL, fftErr);               // Amp to FFT
AudioConnection          patchCord7(noise, bandpass);               // Noise to bandpass
AudioConnection          patchCord8(bandpass, ampNoise);            // Bandpass to noise amp
AudioConnection          patchCord9(ampNoise, 0, mixer, 0);         // Noise amp to mixer
AudioConnection          patchCord10(mixer, ampOut);                // Mixer to output amp
AudioConnection          patchCord11(ampOut, 0, i2sOut, 0);         // Output amp to left speaker
AudioControlSGTL5000     sgtl5000;     


#define SDCARD_CS_PIN    BUILTIN_SDCARD
#define NUM_BINS 256
#define MAX_SECONDARY_PATH_POINTS 10
#define TARGET_FREQ 516    
#define TARGET_BIN 3       
#define NOISE_AMP 0.5      
#define MU 0.0001f        
#define MAX_WEIGHT 5.0f   
#define REF_THRESHOLD 4.0f  // Ignore signals below this
#define MAX_GAIN 0.8f      // Maximum gain limit
#define CONTROL_SCALE 0.1f // Scaling factor for control signal

// Function to limit weight values
void limitWeight(Complex& w) {
    // Limit real component
    if (w.real > MAX_WEIGHT) w.real = MAX_WEIGHT;
    else if (w.real < -MAX_WEIGHT) w.real = -MAX_WEIGHT;
    
    // Limit imaginary component
    if (w.imag > MAX_WEIGHT) w.imag = MAX_WEIGHT;
    else if (w.imag < -MAX_WEIGHT) w.imag = -MAX_WEIGHT;
}

// Global variables
TransferFunctionData secondaryPath[MAX_SECONDARY_PATH_POINTS];
int numSecondaryPathPoints = 0;

// FxLMS variables
Complex w(0.0f, 0.0f);    // Filter weight for target frequency
Complex spf;              // Secondary path for target frequency

// Function to load secondary path data from SD card
bool loadSecondaryPath() {
    if (!SD.exists("SecondaryPath.txt")) {
        Serial.println("Secondary path file not found!");
        return false;
    }

    File dataFile = SD.open("SecondaryPath.txt");
    if (!dataFile) {
        Serial.println("Error opening secondary path file!");
        return false;
    }

    // Skip header line
    dataFile.readStringUntil('\n');

    // Read data points
    numSecondaryPathPoints = 0;
    while (dataFile.available() && numSecondaryPathPoints < MAX_SECONDARY_PATH_POINTS) {
        String line = dataFile.readStringUntil('\n');
        
        // Parse tab-separated values
        int tabIndex = line.indexOf('\t');
        line = line.substring(tabIndex + 1);
        
        secondaryPath[numSecondaryPathPoints].frequency = line.toFloat();
        tabIndex = line.indexOf('\t');
        line = line.substring(tabIndex + 1);
        
        secondaryPath[numSecondaryPathPoints].magnitude = line.toFloat();
        tabIndex = line.indexOf('\t');
        line = line.substring(tabIndex + 1);
        
        secondaryPath[numSecondaryPathPoints].phase = line.toFloat();
        tabIndex = line.indexOf('\t');
        line = line.substring(tabIndex + 1);
        
        secondaryPath[numSecondaryPathPoints].unwrappedPhase = line.toFloat();
        
        numSecondaryPathPoints++;
    }
    
    dataFile.close();
    
    Serial.print("Loaded ");
    Serial.print(numSecondaryPathPoints);
    Serial.println(" secondary path points");
    
    return true;
}

// Convert secondary path magnitude and phase to complex form
void initializeSecondaryPath() {
    // Find and convert secondary path data for target bin
    for (int i = 0; i < numSecondaryPathPoints; i++) {
        if (abs(secondaryPath[i].frequency - TARGET_FREQ) < 1.0f) {
            float mag = secondaryPath[i].magnitude;
            float phase = secondaryPath[i].phase;
            spf.real = mag * cos(phase);
            spf.imag = mag * sin(phase);
            Serial.println("Secondary path ready");
            Serial.print("Magnitude: "); Serial.println(mag, 6);
            Serial.print("Phase: "); Serial.println(phase, 6);
            return;
        }
    }
    Serial.println("Warning: Target frequency not found in secondary path data!");
}

// Get complex form of FFT bin
Complex getComplexFFT(AudioAnalyzeFFT256& fft, int bin) {
    return Complex(fft.output[bin*2], fft.output[bin*2 + 1]);
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
    
    // Configure high-pass filters (100 Hz, Q=0.707)
    filterRefL.setHighpass(0, 100, 0.707);
    filterErrL.setHighpass(0, 100, 0.707);
    
    // Configure bandpass filter (516 Hz, Q=10)
    bandpass.setBandpass(0, TARGET_FREQ, 10);
    
    // Configure amplifiers and mixer
    ampRefL.gain(1);    // Set appropriate mic gain
    ampErrL.gain(1);
    ampNoise.gain(0);    // Start with noise off
    ampOut.gain(1.0);
    mixer.gain(0, 1.0);  // Noise input channel
    
    // Initialize noise source
    noise.amplitude(NOISE_AMP);
    
    // Set window function for FFTs
    fftRef.windowFunction(AudioWindowHanning256);
    fftErr.windowFunction(AudioWindowHanning256);
    
    // Load secondary path data
    if (!loadSecondaryPath()) {
        Serial.println("Failed to load secondary path data!");
        while(1);
    }
    
    initializeSecondaryPath();
    delay(1000);  // System stabilization
}

void loop() {
    if (fftRef.available() && fftErr.available()) {
        // Get complex FFT data
        Complex x = getComplexFFT(fftRef, TARGET_BIN);
        Complex e = getComplexFFT(fftErr, TARGET_BIN);
        
        // Calculate signal magnitudes
        float refLevel = sqrt(x.real * x.real + x.imag * x.imag);
        float errLevel = sqrt(e.real * e.real + e.imag * e.imag);
        
        // Declare control variable outside the if statement
        float limitedControl = 0;  // Initialize to 0

        // Only process if reference is above threshold
        if (refLevel > REF_THRESHOLD) {
            // Calculate filtered reference signal
            Complex xf = x * spf;
            
            // Calculate control signal
            Complex y = x * w;
            
            // Calculate control magnitude and scale it
            float controlMag = sqrt(y.real * y.real + y.imag * y.imag);
            float scaledControl = controlMag * CONTROL_SCALE;
            
            // Apply limiter
            limitedControl = min(scaledControl, MAX_GAIN);
            
            // Update noise output
            ampNoise.gain(limitedControl);
            
            // Update weights
            Complex xf_conj = xf.conj();
            Complex update = e * xf_conj;
            w.real -= MU * update.real;
            w.imag -= MU * update.imag;
            limitWeight(w);
        } else {
            // Below threshold - set noise to zero
            ampNoise.gain(0);
        }
        
        // Print diagnostic information
        Serial.print("Ref:"); Serial.print(refLevel, 6);
        Serial.print("\tErr:"); Serial.print(errLevel, 6);
        Serial.print("\tW_real:"); Serial.print(w.real, 6);
        Serial.print("\tW_imag:"); Serial.print(w.imag, 6);

        Serial.println();
    }
}

