#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <arm_math.h>

arm_rfft_fast_instance_f32 rfftInst;

AudioSynthNoiseWhite     whiteNoise;
AudioFilterBiquad        noiseBP;
AudioInputI2SQuad        i2sIn;
AudioRecordQueue         recRefL, recErrL, recRefR, recErrR;
AudioOutputI2S           audioOut;
AudioConnection          patchCord1(whiteNoise, noiseBP);
AudioConnection          patchCord2(noiseBP, 0, audioOut, 0);
AudioConnection          patchCord3(noiseBP, 0, audioOut, 1);
AudioConnection          patchCord4(i2sIn, 0, recRefL, 0);
AudioConnection          patchCord5(i2sIn, 1, recErrL, 0);
AudioConnection          patchCord6(i2sIn, 2, recRefR, 0);
AudioConnection          patchCord7(i2sIn, 3, recErrR, 0);


//Initialize varialbes
const int FFT_SIZE = 1024;
const int    NUM_BINS = 25;
float resolution;                   //frequency resolution in Hz per FFT bin (sampleRate/FFT_SIZE)
int   binIndex[NUM_BINS];
const int NUM_RUNS = 5;     //Number of runs, end result is average of these runs


// frequencies for table print to serial
const float  freqs[NUM_BINS] = {
  43, 86, 129, 172, 215, 258, 301, 344,
  387, 430, 473, 516, 559, 602, 645, 688,
  731, 744, 817, 860, 903, 946, 989, 1032, 1075
};


// data buffers
float bufRefL[FFT_SIZE], bufErrL[FFT_SIZE];
float bufRefR[FFT_SIZE], bufErrR[FFT_SIZE];
float fftRefL[FFT_SIZE], fftErrL[FFT_SIZE];
float fftRefR[FFT_SIZE], fftErrR[FFT_SIZE];

// accumulators (for real and complex)
float sumReL[NUM_BINS], sumImL[NUM_BINS];
float sumReR[NUM_BINS], sumImR[NUM_BINS];

void setup() {
  Serial.begin(115200);
  while (!Serial) ;
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD init failed!");
    while (1) ;
  }

  AudioMemory(20);
  float f1 = 43.0f, f2 = 1075.0f;           //limit white noise 43–1075 Hz:
  float f0 = sqrtf(f1 * f2);
  float Q  = f0 / (f2 - f1);
  noiseBP.setBandpass(f0, Q);
  whiteNoise.amplitude(0.5f);

  // start recording
  recRefL.begin();
  recErrL.begin();
  recRefR.begin();
  recErrR.begin();

  // initialize RFFT
  arm_rfft_fast_init_f32(&rfftInst, FFT_SIZE);

  // compute bin indices
  resolution = (float)AUDIO_SAMPLE_RATE_EXACT / FFT_SIZE;
  for (int i = 0; i < NUM_BINS; i++) {
    binIndex[i] = (int)(freqs[i] / resolution + 0.5f);
  }

  // set all values to zero to start
  for (int i = 0; i < NUM_BINS; i++) {
    sumReL[i] = sumImL[i] = 0.0f;
    sumReR[i] = sumImR[i] = 0.0f;
  }
}

void loop() {
  // run measurements
  for (int run = 0; run < NUM_RUNS; run++) {
    fillBuffer(recRefL, bufRefL);
    fillBuffer(recErrL, bufErrL);
    fillBuffer(recRefR, bufRefR);
    fillBuffer(recErrR, bufErrR);

    // compute FFTs
    arm_rfft_fast_f32(&rfftInst, bufRefL,  fftRefL,  0);
    arm_rfft_fast_f32(&rfftInst, bufErrL,  fftErrL,  0);
    arm_rfft_fast_f32(&rfftInst, bufRefR,  fftRefR,  0);
    arm_rfft_fast_f32(&rfftInst, bufErrR,  fftErrR,  0);

    // for each bin, compute H = Y/X
    for (int i = 0; i < NUM_BINS; i++) {
      int k = binIndex[i];
      float xr = fftRefL[2*k],   xi = fftRefL[2*k+1];
      float yr = fftErrL[2*k],   yi = fftErrL[2*k+1];
      float denom = xr*xr + xi*xi + 1e-12f;
      float Hr = (yr*xr + yi*xi) / denom;
      float Hi = (yi*xr - yr*xi) / denom;
      sumReL[i] += Hr;
      sumImL[i] += Hi;

      xr = fftRefR[2*k]; xi = fftRefR[2*k+1];
      yr = fftErrR[2*k]; yi = fftErrR[2*k+1];
      denom = xr*xr + xi*xi + 1e-12f;
      Hr = (yr*xr + yi*xi) / denom;
      Hi = (yi*xr - yr*xi) / denom;
      sumReR[i] += Hr;
      sumImR[i] += Hi;
    }
  }

  // save to SD & print results
  SD.remove("SecondaryPath.txt");
  File f = SD.open("SecondaryPath.txt", FILE_WRITE);
  printResults(Serial, f, "Left Side", sumReL, sumImL);
  printResults(Serial, f, "Right Side", sumReR, sumImR);
  f.close();

  // done once
  while (1) delay(1000);
}



void fillBuffer(AudioRecordQueue &q, float *buf) {
  // wait for FFT_SIZE samples = 8 blocks × 128 samples
  const int BLOCKS = FFT_SIZE / 128;
  int idx = 0;
  for (int b = 0; b < BLOCKS; b++) {
    while (!q.available()) ;
    int16_t *block = (int16_t*)q.readBuffer();
    for (int i = 0; i < 128; i++) {
      buf[idx++] = (float)block[i] / 32768.0f;
    }
    q.freeBuffer();
  }
}

void printResults(Stream &out1, File &out2,
                  const char *side,
                  float sumRe[], float sumIm[]) {
  out1.printf("\nFinal Transfer Function Summary for\n%s:\n", side);
  out2.printf("\nFinal Transfer Function Summary for\n%s:\n", side);

  out1.println("Freq(Hz) Magnitude  Phase(rad)  Unwrapped Phase(rad)");
  out2.println("Freq(Hz) Magnitude  Phase(rad)  Unwrapped Phase(rad)");

  float prevPhase = 0.0f;
  for (int i = 0; i < NUM_BINS; i++) {
    float Hr = sumRe[i] / NUM_RUNS;
    float Hi = sumIm[i] / NUM_RUNS;
    float mag   = sqrtf(Hr*Hr + Hi*Hi);
    float phase = atan2f(Hi, Hr);
    //unwrap phase
    float d = phase - prevPhase;
    if (d >  3.1415926f) phase -= 2*3.1415926f;
    if (d < -3.1415926f) phase += 2*3.1415926f;
    prevPhase = phase;

    out1.printf("%4d   %8.6f   %8.6f   %8.6f\n",
                (int)freqs[i], mag, phase, phase);
    out2.printf("%4d   %8.6f   %8.6f   %8.6f\n",
                (int)freqs[i], mag, phase, phase);
  }
}
