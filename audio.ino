// Audio and transport functions for Looper
/*
#include <Arduino.h>
#include <Audio.h>
#include <SD.h>
#include "status.h"

extern int16_t loopBuffer[][128];
extern volatile int blocksRecorded;
extern int loopPlayIndex;
extern float waveformGain;
extern int16_t waveformPeak;
extern bool isRecording, isPlaying, armedRecord;
extern const float ARM_LEVEL_THRESHOLD;
extern AudioRecordQueue recQ;
extern AudioPlayQueue playQ;
extern AudioMixer4 mix;
extern AudioEffectFreeverb reverb;
extern Status current;

// SD card audio variables
File wavFile;
volatile uint32_t samplesWritten = 0;
bool isSDRecording = false;

File playFile;
const uint32_t BUF_SAMPLES = 4096;
DMAMEM int16_t sdBufA[BUF_SAMPLES];
DMAMEM int16_t sdBufB[BUF_SAMPLES];
volatile uint32_t rdIndexA = 0, validA = 0;
volatile uint32_t rdIndexB = 0, validB = 0;
volatile bool bufAReady = false, bufBReady = false;
bool useA = true;
bool sdPlaying = false;

// SD card audio functions
void writeWavHeader(File &f, uint32_t sampleRate, uint16_t bitsPerSample, uint16_t channels) {
  uint8_t hdr[44];
  uint32_t byteRate = sampleRate * channels * (bitsPerSample/8);
  uint16_t blockAlign = channels * (bitsPerSample/8);
  memcpy(hdr+0,  "RIFF", 4);
  *(uint32_t*)(hdr+4)  = 0;
  memcpy(hdr+8,  "WAVE", 4);
  memcpy(hdr+12, "fmt ", 4);
  *(uint32_t*)(hdr+16) = 16;
  *(uint16_t*)(hdr+20) = 1;
  *(uint16_t*)(hdr+22) = channels;
  *(uint32_t*)(hdr+24) = sampleRate;
  *(uint32_t*)(hdr+28) = byteRate;
  *(uint16_t*)(hdr+32) = blockAlign;
  *(uint16_t*)(hdr+34) = bitsPerSample;
  memcpy(hdr+36, "data", 4);
  *(uint32_t*)(hdr+40) = 0;
  f.write(hdr, 44);
}

void patchWavSizes(File &f, uint32_t dataBytes) {
  uint32_t riffSize = 36 + dataBytes;
  f.seek(4);  f.write((uint8_t*)&riffSize, 4);
  f.seek(40); f.write((uint8_t*)&dataBytes, 4);
}

void startSDRecord(const char* path = "/loop.wav") {
  wavFile = SD.open(path, FILE_WRITE);
  if (wavFile) {
    writeWavHeader(wavFile, 44100, 16, 1);
    samplesWritten = 0;
    isSDRecording = true;
  }
}

void stopSDRecord() {
  if (isSDRecording && wavFile) {
    patchWavSizes(wavFile, samplesWritten * 2);
    wavFile.close();
    isSDRecording = false;
  }
}

void startSDPlay(const char* path = "/loop.wav") {
  if (sdPlaying) return;
  playFile = SD.open(path, FILE_READ);
  if (!playFile) return;
  playFile.seek(44);
  rdIndexA = rdIndexB = 0;
  validA = validB = 0;
  bufAReady = bufBReady = false;
  useA = true;
  sdPlaying = true;
}

void stopSDPlay() {
  if (!sdPlaying) return;
  playFile.close();
  sdPlaying = false;
}

void refillIfNeeded() {
  if (!bufAReady) {
    int32_t n = playFile.read((uint8_t*)sdBufA, BUF_SAMPLES * 2);
    if (n > 0) {
      validA = (uint32_t)n / 2;
      rdIndexA = 0;
      bufAReady = true;
    }
  }
  if (!bufBReady) {
    int32_t n = playFile.read((uint8_t*)sdBufB, BUF_SAMPLES * 2);
    if (n > 0) {
      validB = (uint32_t)n / 2;
      rdIndexB = 0;
      bufBReady = true;
    }
  }
}

void applyFadeInOut();
void startArmedRecord();
void startRecordingNow();
void stopRecording();
void startPlayback();
void stopPlayback();
void deleteSample();

void applyFadeInOut() {
  if (blocksRecorded <= 0) return;
  const int fadeN = 64;
  for (int i=0;i<min(fadeN,128);i++){
    int32_t s = loopBuffer[0][i];
    s = (s * i) / fadeN;
    loopBuffer[0][i] = (int16_t)s;
  }
  int last = blocksRecorded - 1;
  for (int i = 0; i < min(fadeN, 128); i++) {
    int idx = 128 - 1 - i;
    int32_t s = loopBuffer[last][idx];
    float fade = (float)i / (float)fadeN;
    fade = fade * fade;
    s = (int32_t)(s * fade);
    loopBuffer[last][idx] = (int16_t)s;
  }
  loopBuffer[last][128 - 1] = 0;
}
void startArmedRecord(){
  blocksRecorded = 0;
  isPlaying = false;
  isRecording = false;
  armedRecord = true;
  recQ.clear();
  memset(loopBuffer, 0, sizeof(loopBuffer));
  loopPlayIndex = 0;
  mix.gain(0, 0.0f);
  recQ.begin();
  current = ST_ARMED; drawStatus(current);
}
void startRecordingNow(){
  isRecording = true;
  armedRecord = false;
  current = ST_RECORDING; drawStatus(current);
  // startSDRecord("/loop.wav"); // If you want SD recording, implement here
}
void stopRecording() {
  if (!(isRecording || armedRecord)) return;
  recQ.end();
  // stopSDRecord(); // If you want SD recording, implement here
  bool hadAudio = (blocksRecorded > 0);
  isRecording = false;
  armedRecord = false;
  if (hadAudio) {
    applyFadeInOut();
    int16_t m = 0;
    for (int b = 0; b < blocksRecorded; ++b) {
      for (int i = 0; i < 128; ++i) {
        int16_t s = abs(loopBuffer[b][i]);
        if (s > m) m = s;
      }
    }
    waveformPeak = max((int16_t)m, (int16_t)256);
    waveformGain = 20.0f / (waveformPeak / 32768.0f);
    current = ST_RECORDED;
  } else {
    current = ST_NOAUDIO;
  }
  drawStatus(current);
}
void startPlayback(){
  if (blocksRecorded <= 0) { current = ST_NOAUDIO; drawStatus(current); return; }
  loopPlayIndex = 0;
  isPlaying = true;
  current = ST_PLAYING; drawStatus(current);
  mix.gain(0, 0.0f);
}
void stopPlayback(){
  isPlaying = false;
  current = ST_STOPPED; drawStatus(current);
  if (!isRecording && !armedRecord) mix.gain(0, 1.0f);
}
void deleteSample() {
  isRecording = false;
  isPlaying = false;
  armedRecord = false;
  blocksRecorded = 0;
  loopPlayIndex = 0;
  memset(loopBuffer, 0, sizeof(loopBuffer));
  mix.gain(0, 1.0f);
  current = ST_READY;
  drawStatus(current);
  drawCounts(0);
  extern Adafruit_SSD1351 tft;
  tft.fillRect(0, 96, 128, 32, BLACK);
}
*/