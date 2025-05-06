#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <arm_math.h>

arm_rfft_fast_instance_f32 rfftInst;

AudioInputI2SQuad        i2sIn;          
AudioEffectFxLMS         fxlms;
AudioOutputI2S           audioOut;       
AudioConnection          patchCord1(i2sIn, 0, fxlms, 0);
AudioConnection          patchCord2(i2sIn, 1, fxlms, 1);
AudioConnection          patchCord3(i2sIn, 2, fxlms, 2);
AudioConnection          patchCord4(i2sIn, 3, fxlms, 3);
AudioConnection          patchCord5(fxlms, 0, audioOut, 0);
AudioConnection          patchCord6(fxlms, 1, audioOut, 1);


#define FFT_SIZE      1024
#define TAP           16
#define BIN_COUNT     25
#define MU            0.01f


int   binIndex[BIN_COUNT];
float resolution;

//Bin labels (frequencies)
const float freqs[BIN_COUNT] = {
  43, 86, 129, 172, 215, 258, 301, 344,
  387, 430, 473, 516, 559, 602, 645, 688,
  731, 744, 817, 860, 903, 946, 989, 1032, 1075
};

// storage for secondary path
float magL[BIN_COUNT], phL[BIN_COUNT];
float magR[BIN_COUNT], phR[BIN_COUNT];
float secL[TAP], secR[TAP];


//custom FxLMS Object
class AudioEffectFxLMS : public AudioStream {
public:
  AudioEffectFxLMS(): AudioStream(4, inputQueueArray) {
    // set buffers & weights to zero
    for (int i=0; i<TAP; i++) {
      wL[i] = wR[i] = 0.0f;
      xbufL[i]=xbufR[i]=0.0f;
      xpBufL[i]=xpBufR[i]=0.0f;
    }
    bufPos = 0;
  }
  // load  secondary path
  void setSecondaryPath(const float *hL, const float *hR) {
    memcpy(secL, hL, sizeof(secL));
    memcpy(secR, hR, sizeof(secR));
  }

  // process one audio_block at a time
  virtual void update(void) {
    audio_block_t *bRefL = receiveReadOnly(0);
    audio_block_t *bErrL = receiveReadOnly(1);
    audio_block_t *bRefR = receiveReadOnly(2);
    audio_block_t *bErrR = receiveReadOnly(3);
    audio_block_t *bOutL = allocate();
    audio_block_t *bOutR = allocate();
    }

    // normalize
    for (int i=0; i < AUDIO_BLOCK_SAMPLES; i++) {
       float xL = bRefL->data[i] * (1.0f / 32768.0f);
      float eL = bErrL->data[i] * (1.0f / 32768.0f);
      float xR = bRefR->data[i] * (1.0f / 32768.0f);
      float eR = bErrR->data[i] * (1.0f / 32768.0f);

      // circular buffer
      bufPos = (bufPos + 1) & (TAP-1);
      xbufL[bufPos] = xL;
      xbufR[bufPos] = xR;

      // compute xp[n]
      float xpL = 0, xpR = 0;
      for (int k=0; k<TAP; k++) {
        int idx = (bufPos - k) & (TAP-1);
        xpL += secL[k] * xbufL[idx];
        xpR += secR[k] * xbufR[idx];
      }
      xpBufL[bufPos] = xpL;
      xpBufR[bufPos] = xpR;

      // compute  y[n]
      float yL = 0, yR = 0;
      for (int k=0; k<TAP; k++) {
        int idx = (bufPos - k) & (TAP-1);
        yL += wL[k] * xbufL[idx];
        yR += wR[k] * xbufR[idx];
      }

      // send adaptive output signal
      int16_t outSampL = (int16_t)constrain(-yL * 32767.0f, -32768, 32767);
      int16_t outSampR = (int16_t)constrain(-yR * 32767.0f, -32768, 32767);
      bOutL->data[i] = outSampL;
      bOutR->data[i] = outSampR;

      // update weight
      for (int k=0; k<TAP; k++) {
        int idx = (bufPos - k) & (TAP-1);
        wL[k] += MU * eL * xpBufL[idx];
        wR[k] += MU * eR * xpBufR[idx];
      }
    }

    // transmit and release audio blocks
    transmit(bOutL, 0);
    transmit(bOutR, 1);
    release(bRefL);
    release(bErrL);
    release(bRefR);
    release(bErrR);
    release(bOutL);
    release(bOutR);
  }

private:
  audio_block_t *inputQueueArray[4];
  float secL[TAP], secR[TAP];
  float wL[TAP], wR[TAP];
  float xbufL[TAP], xbufR[TAP];
  float xpBufL[TAP], xpBufR[TAP];
  int   bufPos;
};



//read secondary path
void readSection(File &f, float *mag, float *ph) {
  char line[64];              // skip header
  do {
    if (!f.fgets(line, sizeof(line))) return;
  } while (strncmp(line, "Freq", 4) != 0);
  for (int i=0; i<BIN_COUNT; i++) {
    if (!f.fgets(line, sizeof(line))) return;
    sscanf(line, "%*f %f %f %*f", &mag[i], &ph[i]);
  }
}

//— build complex spectrum secondary path
void buildImpulse(const float *mag, const float *ph, float *sec) {
  static float spec[FFT_SIZE];
  static float timeDom[FFT_SIZE];
  memset(spec, 0, sizeof(spec));
    for (int i=0; i<BIN_COUNT; i++) {
    int k = binIndex[i];
    float r = mag[i] * cosf(ph[i]);
    float im= mag[i] * sinf(ph[i]);
    spec[2*k]   = r;
    spec[2*k+1] = im;
  }
  // take fft to time domain
  arm_rfft_fast_f32(&rfftInst, spec, timeDom, 1);
  for (int i=0; i<TAP; i++) sec[i] = timeDom[i];
}

void setup() {
  Serial.begin(115200);
  while (!Serial) ;
  if (!SD.begin(BUILTIN_SDCARD)) {               //initialize SD
    Serial.println("SD init failed"); while(1);
  }
  arm_rfft_fast_init_f32(&rfftInst, FFT_SIZE);      //initialize CMSIS FFT

  // compute bin indices
  resolution = (float)AUDIO_SAMPLE_RATE_EXACT / FFT_SIZE;
  for (int i=0; i<BIN_COUNT; i++) {
    binIndex[i] = int(freqs[i] / resolution + 0.5f);
  }

  // open & seprerate SecondaryPath.txt
  File f = SD.open("SecondaryPath.txt");  
  while (f.available() && !f.find("Left Side")) { }         //Left side
  readSection(f, magL, phL);
  while (f.available() && !f.find("Right Side")) { }      //Right side
  readSection(f, magR, phR);
  f.close();

  // build impulse response
  buildImpulse(magL, phL, secL);
  buildImpulse(magR, phR, secR);

  // hand off to FxLMS object
  fxlms.setSecondaryPath(secL, secR);
}

void loop() {
}