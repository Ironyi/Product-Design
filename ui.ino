// UI and display functions for Looper
/*
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include "status.h"

extern Adafruit_SSD1351 tft;
extern const uint16_t BLACK, WHITE, RED, GREEN, BLUE, YELLOW, CYAN, GRAY;
extern int16_t waveformPeak;

void drawHeader();
void drawStatus(Status s);
void drawFX(float depth, float vol);
void drawCounts(int n);
void drawMeter(float p);
int sampleToY(int16_t s);
void drawWaveform();

// --- UI function implementations ---
void drawHeader(){
  tft.fillScreen(BLACK);
  tft.setTextColor(WHITE); tft.setTextSize(1);
  tft.setCursor(4,4); tft.println("USB Looper");
  tft.drawLine(0,16,127,16,GRAY);
}
void drawStatus(Status s){
  static Status lastDrawn = (Status)255;
  if (s == lastDrawn) return;
  lastDrawn = s;
  tft.fillRect(0,20,128,18,BLACK); tft.setCursor(4,22);
  switch(s){
    case ST_READY:     tft.setTextColor(WHITE);  tft.print("Status: Ready"); break;
    case ST_ARMED:     tft.setTextColor(YELLOW); tft.print("Status: Armed (waiting)"); break;
    case ST_RECORDING: tft.setTextColor(GREEN);  tft.print("Status: Recording..."); break;
    case ST_RECORDED:  tft.setTextColor(YELLOW); tft.print("Status: Recorded"); break;
    case ST_PLAYING:   tft.setTextColor(GREEN);  tft.print("Status: Playing..."); break;
    case ST_STOPPED:   tft.setTextColor(WHITE);  tft.print("Status: Stopped"); break;
    case ST_NOAUDIO:   tft.setTextColor(RED);    tft.print("Status: No audio"); break;
    case ST_TOOQUIET:  tft.setTextColor(RED);    tft.print("Status: Too quiet"); break;
  }
}
void drawFX(float depth, float vol){
  tft.fillRect(0,40,128,20,BLACK); tft.setCursor(4,42);
  tft.setTextColor(CYAN); tft.print("Reverb: "); tft.print(depth,2);
  tft.setCursor(4,54);
  tft.setTextColor(YELLOW); tft.print("Volume: "); tft.print(vol,2);
}
void drawCounts(int n){
  tft.fillRect(0,62,128,18,BLACK);
  tft.setCursor(4,64);
  tft.setTextColor(YELLOW);
  float seconds = (n * (float)128) / 44100.0f;
  tft.print("Blocks: "); tft.print(n);
  tft.print("  ("); tft.print(seconds, 2); tft.print("s)");
}
void drawMeter(float p){
  int w = (int)(constrain(p,0.0f,1.0f)*127.0f);
  tft.fillRect(0,84,w,6,GREEN);
  tft.fillRect(w,84,127-w,6,BLACK);
}
// Map int16 sample to screen Y (centered at y=104, height=40)
int sampleToY(int16_t s) {
  float norm = (float)s / (float)waveformPeak;
  int yCenter = 104;
  int y = yCenter - (int)(norm * 20.0f);
  if (y < 64) y = 64;
  if (y > 128) y = 128;
  return y;
}
void drawWaveform() {
  extern int blocksRecorded;
  extern int16_t loopBuffer[][128];
  tft.fillRect(0, 96, 128, 32, BLACK);
  if (blocksRecorded <= 0) return;
  const uint32_t totalSamples = (uint32_t)blocksRecorded * 128;
  for (int x = 0; x < 128; x++) {
    uint32_t idx = (uint32_t)x * totalSamples / 128;
    uint32_t blk = idx / 128;
    uint32_t off = idx % 128;
    if (blk >= (uint32_t)blocksRecorded) blk = blocksRecorded - 1;
    int16_t s = loopBuffer[blk][off];
    int y = sampleToY(s);
    int y0 = 104;
    int y1 = y;
    if (y1 < y0) { int tmp = y0; y0 = y1; y1 = tmp; }
    tft.drawFastVLine(x, y0, y1 - y0 + 1, CYAN);
  }
}
*/