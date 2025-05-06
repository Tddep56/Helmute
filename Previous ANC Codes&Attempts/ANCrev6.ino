#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>


#define FILTER_LENGTH 64           // Define the length (number of taps)
#define AUDIO_BLOCK_SAMPLES 128    // Define the block size (128 samples per block is standard for Teensy)




// Create a custom audio processing block: AudioLMS
class AudioLMS : public AudioStream {
public:
   AudioLMS() : AudioStream(2, inputQueueArray) {      //2 inputs (reference and error)
    // Initialize filter coefficients set buffer to zero
    for (int i = 0; i < FILTER_LENGTH; i++) {
      coeff[i] = 0.0;
      refBuffer[i] = 0.0;
    }
      mu = 0.0001;  //STEP SIZE
  }
  
  // The update() function is called automatically to process each block of samples
  virtual void update(void);
  
private:
  audio_block_t *inputQueueArray[2]; // Array to hold blocks for 2 channels
  float coeff[FILTER_LENGTH];        // array for filter coefficients
  float refBuffer[FILTER_LENGTH];    // Circular buffer to store reference samples
  float mu;                          // Step size for the LMS(adaptation rate)
};



// AudioLMS::update(): This function is called for each block of samples 
// It receives the reference and error signals, updates the adaptive filter, and produces the anti-noise output.

void AudioLMS::update(void) {
  // Retrieve a block from input 0 (reference microphone signal)
  audio_block_t *refBlock = receiveReadOnly(0);
  // Retrieve a block from input 1 (error microphone signal)
  audio_block_t *errBlock = receiveReadOnly(1);

  // If either block is missing, release block and throw error
  if (!refBlock || !errBlock) {
    if (refBlock) release(refBlock);
    if (errBlock) release(errBlock);
    Serial.println("Error: Missing block!");
    return;
  }

  // Allocate an output block for anti-noise 
  audio_block_t *outBlock = allocate();
  if (!outBlock) {
  Serial.println("Error: Could not create output block!");
    return;
  }

  // Process each sample in the current audio block.
  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
    // Convert the 16-bit reference and error sample to float for processing
     float x = (float) refBlock->data[i];
    float e = (float) errBlock->data[i];

    //  Update the Reference Buffer 
       for (int j = 0; j < FILTER_LENGTH - 1; j++) {      // Shift the buffer one position to the left to make room for the new incoming sample.
      refBuffer[j] = refBuffer[j + 1];
    }
      refBuffer[FILTER_LENGTH - 1] = x;       // Insert the new reference sample at the end of the buffer.

    
    // Compute the dot product of the adaptive filter coefficients and the reference buffer. This gives the estimated noise we want to cancel
    float y = 0.0;
    for (int j = 0; j < FILTER_LENGTH; j++) {
      y += coeff[j] * refBuffer[j];
    }

   
    // Update each filter coefficient using the LMS:
    //   coeff[j] = coeff[j] + mu * e * refBuffer[j]
    for (int j = 0; j < FILTER_LENGTH; j++) {
      coeff[j] += mu * e * refBuffer[j];
    }

 
    float antiNoise = -y;     //  invert filter output to produce a signal that is 180° out-of-phase.


    // Clamp float values to 16 bit signed int range to avoid errors
    if (antiNoise > 32767.0f) {
      antiNoise = 32767.0f;  
      }
    if (antiNoise < -32768.0f) {
        antiNoise = -32768.0f; 
      }
    int16_t outSample = (int16_t) antiNoise; // Convert float back to 16 bit int

    outBlock->data[i] = (int16_t) outSample; // Store the computed sample in the output block.
  }

  transmit(outBlock);   // Transmit the output block to the next stage (audio output)

  // Release the blocks after processing is complete.
  release(refBlock);
  release(errBlock);
  release(outBlock);
}


//   Input 0: Reference microphone signal 
//   Input 1: Error microphone signal 

AudioInputI2S            i2s1;           
AudioAmplifier           amp1;           
AudioAmplifier           amp2;           
AudioLMS                 lms;           
AudioOutputI2S           i2s2;           
AudioConnection          patchCord1(i2s1, 0, amp1, 0);
AudioConnection          patchCord2(amp1, 0, lms, 0);
AudioConnection          patchCord3(i2s1, 1, amp2, 0);
AudioConnection          patchCord4(amp2, 0, lms, 1);
AudioConnection          patchCord5(lms, 0, i2s2, 0);
AudioConnection          patchCord6(lms, 0, i2s2, 1);
AudioControlSGTL5000     sgtl5000_1;     


void setup() {
  Serial.begin(115200);
  AudioMemory(50);  // memory for audio processing (in blocks).

  amp1.gain(10.0); //gain for the reference microphone
  amp2.gain(10.0); //gain for the error microphone

  // Initialize and configure audio shield.
  sgtl5000_1.enable();           
  sgtl5000_1.volume(0.6);         
  sgtl5000_1.unmuteLineout();       
  sgtl5000_1.lineOutLevel(10);      
}

void loop() {

}
