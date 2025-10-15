#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <stdint.h>
#include "status.h"

// Forward declare Status enum for use in function prototypes
enum Status;
// UI
void drawHeader();
void drawStatus(Status s);
void drawFX(float depth, float vol);
void drawCounts(int n);
void drawMeter(float p);
int sampleToY(int16_t s);
void drawWaveform();

// Audio/Transport
void applyFadeInOut();
void startArmedRecord();
void startRecordingNow();
void stopRecording();
void startPlayback();
void stopPlayback();
void deleteSample();

// SD Card
void writeWavHeader(File &f, uint32_t sampleRate, uint16_t bitsPerSample, uint16_t channels);
void patchWavSizes(File &f, uint32_t dataBytes);
void startSDRecord(const char* path = "/loop.wav");
void stopSDRecord();
void startSDPlay(const char* path = "/loop.wav");
void stopSDPlay();
void refillIfNeeded();

#endif // FUNCTIONS_H
