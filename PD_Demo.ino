#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <cstdint>
#include <malloc.h>

#include "BetterButton.h"
#include "status.h"
#include "functions.h"

// Pins / display
constexpr int OLED_CS  = 26;
constexpr int OLED_DC  = 27;
constexpr int OLED_RST = 9;
Adafruit_SSD1351 tft(128, 128, &SPI, OLED_CS, OLED_DC, OLED_RST);

// Colors
constexpr uint16_t COL_BLACK  = 0x0000;
constexpr uint16_t COL_WHITE  = 0xFFFF;
constexpr uint16_t COL_RED    = 0xF800;
constexpr uint16_t COL_GREEN  = 0x07E0;
constexpr uint16_t COL_BLUE   = 0x001F;
constexpr uint16_t COL_YELLOW = 0xFFE0;
constexpr uint16_t COL_CYAN   = 0x07FF;
constexpr uint16_t COL_GRAY   = 0x8410;

// Modes / FX
enum class Fx { Reverb, Bitcrush, Lowpass };
enum class UiMode { Ram, SdRec, Menu, FxMenu };

Fx currentFx = Fx::Reverb;
UiMode uiMode = UiMode::Ram;

// Audio graph
AudioInputI2S        lineIn;
AudioMixer4          inputMix;
AudioFilterBiquad    hp;
AudioRecordQueue     recordQueue;
AudioPlayQueue       playQueue;
AudioPlaySdWav       sd1, sd2;
AudioMixer4          sdMix;      
AudioMixer4          mainMix;     
AudioEffectFreeverb  reverb;
AudioEffectBitcrusher bitcrush;
AudioFilterBiquad    lowpass;
AudioOutputI2S       i2sOut;
AudioAnalyzePeak     peakIn;
AudioAnalyzePeak     peakOut;
AudioControlSGTL5000 codec;

// Wiring
AudioConnection c0(lineIn, 0, inputMix, 1);
AudioConnection c1(inputMix, 0, hp, 0);
AudioConnection c2(hp, 0, recordQueue, 0);
AudioConnection c3(hp, 0, mainMix, 0);
AudioConnection c4(playQueue, 0, mainMix, 1);
AudioConnection c5(sd1, 0, sdMix, 0);
AudioConnection c6(sd2, 0, sdMix, 1);
AudioConnection c7(sdMix, 0, mainMix, 2);
AudioConnection c8(mainMix, 0, reverb, 0);
AudioConnection c9(reverb, 0, lowpass, 0);
AudioConnection c10(lowpass, 0, i2sOut, 0);
AudioConnection c11(lowpass, 0, i2sOut, 1);
AudioConnection c12(inputMix, 0, peakIn, 0);
AudioConnection c13(reverb, 0, peakOut, 0);

// SD
SdFs sdfs;
bool sdfsReady = false; 

struct WavInfo {
  uint32_t dataOffset = 0;
  uint32_t dataBytes  = 0;
  uint16_t channels   = 1;
  uint32_t sampleRate = 44100;
  uint16_t bits       = 16;
  bool ok             = false;
};

WavInfo parseWavHeader(FsFile &f) {
  WavInfo wi;
  if (!f) return wi;
  uint8_t hdr[44];
  if (f.read(hdr, 44) != 44) { f.close(); return wi; }
  if (memcmp(hdr+0,  "RIFF", 4) || memcmp(hdr+8, "WAVE", 4)) { f.close(); return wi; }
  wi.channels   = *(uint16_t*)(hdr+22);
  wi.sampleRate = *(uint32_t*)(hdr+24);
  wi.bits       = *(uint16_t*)(hdr+34);
  wi.dataBytes  = *(uint32_t*)(hdr+40);
  wi.dataOffset = 44;
  wi.ok = (wi.channels == 1 && wi.sampleRate == 44100 && wi.bits == 16 && wi.dataBytes > 0);
  if (!wi.ok) f.close();
  return wi;
}

void flushPlaybackQueue() {
  const int MAX_FLUSH_ITER = 4; 
  int iter = 0;
  while (iter < MAX_FLUSH_ITER && playbackQueue.available() > 0) {
    int16_t *buf = playbackQueue.getBuffer();
    if (!buf) break;
    memset(buf, 0, 128 * sizeof(int16_t));
    playbackQueue.playBuffer();
    ++iter;
    yield();
  }
}

//  Pins 
const int buttonPins[7] = {33, 34, 35, 36, 37, 32, 30}; 
const int POT_FX = A10;
const int POT_VOL = A14;
#ifndef INPUT_SWITCH_PIN
#define INPUT_SWITCH_PIN 31
#endif

BetterButton button1(buttonPins[0], 0, INPUT_PULLUP); // Record
BetterButton button2(buttonPins[1], 1, INPUT_PULLUP); // Play
BetterButton button3(buttonPins[2], 2, INPUT_PULLUP); // Stop
BetterButton button4(buttonPins[3], 3, INPUT_PULLUP); // Delete
BetterButton button5(buttonPins[4], 4, INPUT_PULLUP); // SD Mode
BetterButton menuBtnUp(buttonPins[5], 5, INPUT_PULLUP); // Menu Up
BetterButton menuBtnDown(buttonPins[6], 6, INPUT_PULLUP); // Menu Down

BetterButton* buttonArray[7] = { &button1, &button2, &button3, &button4, &button5, &menuBtnUp, &menuBtnDown };

//  Loop buffer 
const int BLOCK_SAMPLES = 128;        
const int MAX_BLOCKS    = 700;  
int sdRecMaxBlocks = 4000;  

DMAMEM __attribute__((aligned(32)))
int16_t loopBuffer[MAX_BLOCKS][BLOCK_SAMPLES];
volatile int blocksRecorded = 0;
// total number of recorded blocks in RAM or SD
volatile int& recordedBlockCount = blocksRecorded;
static int loopPlayIndex = 0;
// current playback block index
static int& playbackBlockIndex = loopPlayIndex;
int16_t waveformPeak = 512;     
float   waveformGain = 1.0f;   

bool isRecording   = false;
bool isPlaying     = false;
bool armedRecord   = false;    
bool& recordingState = isRecording;
bool& playingState = isPlaying;
bool& armedState = armedRecord;
const float ARM_LEVEL_THRESHOLD = 0.02f;

//  Looper state 
bool loopCaptured = false;     
bool overdubEnabled = false;   
float odInputGain = 0.8f;      
float odFeedback  = 0.7f;      
volatile bool countsDirty = false;

//  WAV Writer state 
FsFile wavFile;
volatile uint32_t samplesWritten = 0;    
bool isSDRecording = false; 

// Playback source: false = RAM, true = SD
bool playFromSD = false; 
uint32_t sdBlocksToPlay = 0; 

//  WAV Reader 
FsFile playFile; 
const uint32_t BUF_SAMPLES = 4096;       
DMAMEM int16_t sdBufA[BUF_SAMPLES];
DMAMEM int16_t sdBufB[BUF_SAMPLES];
volatile uint32_t rdIndexA = 0, validA = 0;
volatile uint32_t rdIndexB = 0, validB = 0; 
volatile bool bufAReady = false, bufBReady = false;
volatile uint32_t& sdReadIndexA = rdIndexA;
volatile uint32_t& sdValidSamplesA = validA;
volatile uint32_t& sdReadIndexB = rdIndexB;
volatile uint32_t& sdValidSamplesB = validB;
volatile bool& sdBufferAReady = bufAReady;
volatile bool& sdBufferBReady = bufBReady;
bool useA = true;                         
bool sdPlaying = false;
bool& sdPlaybackActive = sdPlaying;

//  Clip indicators 
const int CLIP_LED_PIN = 2;               
AudioAnalyzePeak peakOut;                 
bool clipLatched = false;                
AudioConnection pcOutMeter(reverb, 0, peakOut, 0);


//  UI 
Status current = ST_READY, lastDrawn = (Status)255;


void drawStatus(Status s);


//  SD Menu System 
#define MAX_MENU_ITEMS 32
char menuItems[MAX_MENU_ITEMS][32]; 
int menuLength = 0;
int menuIndex = 0;
String currentPath = "/";


//  Playback & Menu screen state 
FXType sdPlayFX = FX_REVERB;
bool sdPlayFXEnabled[FX_COUNT] = {true, true, true};
float sdPlayFXDepth[FX_COUNT] = {0.3f, 0.3f, 0.3f};
bool inEffectsMenu = false;
int effectsMenuIndex = 0;
const char* effectsMenuItems[FX_COUNT] = { "Reverb", "Bitcrush", "Lowpass" };
bool inPlaybackScreen = false;
String playbackFile = "";

//  Playback screen drawing 
void drawPlaybackScreen() {
  tft.fillRect(0, 20, 128, 108, BLACK);
  tft.setTextColor(GREEN); tft.setCursor(4, 22);
  tft.print("Playing: "); tft.print(playbackFile);
  tft.setTextColor(YELLOW); tft.setCursor(4, 110);
  tft.print("Menu Up/Down to exit");
}

void scanSDMenu(const String& path) {
  menuLength = 0;
  FsFile dir = sdfs.open(path.c_str());
  if (!dir) {
    strcpy(menuItems[0], "<SD Error>");
    menuLength = 1;
    return;
  }
  FsFile entry;
  while ((entry = dir.openNextFile())) {
    if (menuLength >= MAX_MENU_ITEMS) break;
    char nameBuf[64];
    entry.getName(nameBuf, sizeof(nameBuf));
    String name = String(nameBuf);
    if (entry.isDirectory()) name += "/";
    strncpy(menuItems[menuLength], name.c_str(), 31);
    menuItems[menuLength][31] = '\0';
    menuLength++;
    entry.close();
  }
  dir.close();
  if (menuLength == 0) {
    strcpy(menuItems[0], "<Empty>");
    menuLength = 1;
  }
  menuIndex = 0;
}

void drawMenu() {
  tft.fillRect(0, 20, 128, 108, BLACK); 
  const int visibleRows = 6; // Number of items visible at once
  int y = 24;
  // Infinite scrolling
  for (int i = 0; i < visibleRows; i++) {
    int menuIdx = menuIndex + i - visibleRows/2;
    if (menuIdx < 0 || menuIdx >= menuLength) continue;
    if (menuIdx == menuIndex) {
      tft.fillRect(0, y-2, 128, 14, BLUE); 
      tft.setTextColor(WHITE, BLUE);
    } else {
      tft.setTextColor(WHITE, BLACK);
    }
    tft.setCursor(4, y);
    tft.print(menuItems[menuIdx]);
    y += 16;
  }

  tft.fillRect(0, 96, 128, 32, BLACK);
  String sel = String(menuItems[menuIndex]);
  if (!sel.endsWith("/") && sel.endsWith(".WAV")) {
    String fullPath = currentPath + sel;
  FsFile f = sdfs.open(fullPath.c_str(), FILE_READ);
    if (f && f.size() > 44) {
      f.seek(44); 
      const int previewSamples = 1024;
      int16_t *buf = (int16_t*)extmem_malloc(previewSamples * sizeof(int16_t));
      if (!buf) {
        f.close();
      } else {
        int n = f.read((uint8_t*)buf, previewSamples * 2);
        if (n > 0) {
          n = n / 2; 
          for (int i = n; i < previewSamples; ++i) buf[i] = 0;
          int16_t peak = 256;
          for (int i = 0; i < n; ++i) {
            int16_t s = abs(buf[i]);
            if (s > peak) peak = s;
          }
          if (peak < 1) peak = 1;
          for (int x = 0; x < 128; ++x) {
            int sampleIndex = x * n / 128;
            if (sampleIndex >= n) sampleIndex = n - 1;
            int16_t s = buf[sampleIndex];
            float norm = (float)s / (float)peak;
            int y0 = 112;
            int y1 = y0 - (int)(norm * 15.0f);
            if (y1 < 96) y1 = 96;
            if (y1 > 127) y1 = 127;
            tft.drawFastVLine(x, min(y0, y1), abs(y1 - y0) + 1, CYAN);
          }
        }
        f.close();
        extmem_free(buf);
      }
    }
  }
}

void drawEffectsMenu() {
  tft.fillRect(0, 20, 128, 108, BLACK); 
  tft.setTextColor(YELLOW); tft.setCursor(4, 22);
  tft.print("Effects");
  int y = 40;
  for (int i = 0; i < FX_COUNT; ++i) {
    if (i == effectsMenuIndex) {
      tft.fillRect(0, y-2, 64, 14, BLUE); 
      tft.setTextColor(WHITE, BLUE);
    } else {
      tft.setTextColor(WHITE, BLACK);
    }
    tft.setCursor(4, y);
    tft.print(effectsMenuItems[i]);
    y += 16;
  }
  tft.setTextColor(sdPlayFXEnabled[effectsMenuIndex] ? GREEN : RED);
  tft.setCursor(4, 100);
  tft.print(sdPlayFXEnabled[effectsMenuIndex] ? "Enabled" : "Disabled");


  tft.fillRect(64, 24, 64, 72, BLACK); 
  float potVal = analogRead(POT_FX) / 1023.0f;
  switch (effectsMenuIndex) {
    case FX_REVERB: {
      int knobCenterX = 64 + 32, knobCenterY = 60, knobRadius = 20;
  tft.drawCircle(knobCenterX, knobCenterY, knobRadius, GRAY);
  float angle = 3.14159f * (0.75f + 1.5f * potVal); // 135° to 405°
  int knobHandleX = knobCenterX + (int)(knobRadius * cos(angle));
  int knobHandleY = knobCenterY + (int)(knobRadius * sin(angle));
  tft.drawLine(knobCenterX, knobCenterY, knobHandleX, knobHandleY, CYAN);
  tft.setTextColor(CYAN); tft.setCursor(knobCenterX-20, knobCenterY+36);
  tft.print("Depth: "); tft.print(potVal,2);
      break;
    }
    case FX_BITCRUSH: {
      int bars = 8 + (int)(potVal * 8.0f);
      int barX = 68, barY = 40;
      for (int barIdx = 0; barIdx < 16; ++barIdx) {
        int barHeight = 4 + barIdx;
        uint16_t col = (barIdx < bars) ? CYAN : GRAY;
        tft.fillRect(barX + barIdx*3, barY + 32 - barHeight, 2, barHeight, col);
      }
      tft.setTextColor(CYAN); tft.setCursor(barX, barY+48);
      tft.print("Bits: "); tft.print(bars);
      break;
    }
    case FX_LOWPASS: {
  int curveBaseY = 90, curveWidth = 56, curveHeight = 24;
      float cutoff = 12000.0f - potVal * (12000.0f - 500.0f);
      for (int curveX = 0; curveX < curveWidth; ++curveX) {
        float freq = 500.0f + (curveX/(float)curveWidth)*(12000.0f-500.0f);
        float curveY = curveHeight * exp(-freq/cutoff);
        tft.drawPixel(64 + curveX, curveBaseY - (int)curveY, CYAN);
      }
    tft.setTextColor(CYAN); tft.setCursor(66, curveBaseY+16);
      tft.print("Cutoff: "); tft.print((int)cutoff);
      break;
    }
    default: break;
  }
}

// Applies current selected FX parameters immediately
void applyEffectsNow() {
  FXType fx = sdPlayFX;
  float fxDepth = sdPlayFXDepth[fx];
  bool fxEnabled = sdPlayFXEnabled[fx];
  switch (fx) {
    case FX_REVERB:
      reverb.roomsize(fxEnabled ? (fxDepth > 0.01f ? fxDepth * 0.95f : 0.01f) : 0.01f);
      break;
    case FX_BITCRUSH:
      break;
    case FX_LOWPASS: {
      float minCutoff = 500.0f;
      float maxCutoff = 12000.0f;
      float cutoff = fxEnabled ? (maxCutoff - fxDepth * (maxCutoff - minCutoff)) : maxCutoff;
      if (fxDepth < 0.01f || !fxEnabled) cutoff = maxCutoff;
      lowpassFX.setLowpass(0, cutoff, 0.707f);
      break;
    }
    default: break;
  }

    if (sdPolyPlaying) {
    mix.gain(0, 0.0f); // dry
    mix.gain(1, 0.0f); // queue (RAM/SD mono)
    mix.gain(2, 1.0f); // poly voices
    } else {
    // mono queue is audible
    mix.gain(0, 0.0f);
    mix.gain(1, 1.0f);
    mix.gain(2, 0.0f);
    }
}

void drawHeader(){
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  tft.fillScreen(BLACK);
  tft.setTextColor(WHITE); tft.setTextSize(1);
  tft.setCursor(4,4); tft.println("USB Looper");
  tft.drawLine(0,16,127,16,GRAY);
  tft.setTextColor(YELLOW);
  tft.setCursor(80, 4);
  switch (uiMode) {
  case MODE_RAM:   tft.print("RAM Rec"); break;
  case MODE_SDREC: tft.print("SD Rec"); break;
  case MODE_MENU:  tft.print("MENU");   break;
  case MODE_EFFECTS_MENU: tft.print("FX Menu"); break;
  default: break;
}
  tft.setTextColor(playFromSD ? CYAN : YELLOW);
  tft.setCursor(80, 12); tft.print(playFromSD ? "SDPlay" : "RAMPlay");
  SPI.endTransaction();
}
void drawStatus(Status s){
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
  tft.fillRect(0,40,128,20,BLACK);
  tft.setCursor(4,42);
  tft.setTextColor(CYAN);
  float d = 0.0f;
  switch (currentFX) {
    case FX_REVERB:
      d = reverbDepth;
      tft.print("Reverb: "); tft.print(d,2);
      break;
    case FX_BITCRUSH:
      d = bitcrushDepth;
      tft.print("Bitcrush: "); tft.print(d,2);
      break;
    case FX_LOWPASS:
      d = lowpassDepth;
      tft.print("Lowpass: "); tft.print(d,2);
      break;
    default:
      break;
  }
  tft.setCursor(4,54);
  tft.setTextColor(YELLOW); tft.print("Volume: "); tft.print(vol,2);
}
void drawCounts(int n){
  tft.fillRect(0,62,128,18,BLACK);
  tft.setCursor(4,64);
  tft.setTextColor(YELLOW);
  float seconds = (n * (float)BLOCK_SAMPLES) / 44100.0f;
  tft.print("Blocks: "); tft.print(n);
  tft.print("  ("); tft.print(seconds, 2); tft.print("s)");
}
void drawMeter(float p){
  int w = (int)(constrain(p,0.0f,1.0f)*127.0f);
  tft.fillRect(0,84,w,6,GREEN);
  tft.fillRect(w,84,127-w,6,BLACK);
}


inline int sampleToY(int16_t sample) {
  float normalized = (float)sample / (float)waveformPeak;   
  int centerY = 104;
  int mappedY = centerY - (int)(normalized * 20.0f);   
  if (mappedY < 64) mappedY = 64;
  if (mappedY > 128) mappedY = 128;
  return mappedY;
}

void drawWaveform() {
  tft.fillRect(0, 96, 128, 32, BLACK);
  if (blocksRecorded <= 0) return;

  const uint32_t totalSamples = (uint32_t)blocksRecorded * BLOCK_SAMPLES;
  for (int x = 0; x < 128; x++) {
    uint32_t sampleIndex = (uint32_t)x * totalSamples / 128;
    uint32_t blockIndex = sampleIndex / BLOCK_SAMPLES;
    uint32_t offsetInBlock = sampleIndex % BLOCK_SAMPLES;
    if (blockIndex >= (uint32_t)blocksRecorded) blockIndex = blocksRecorded - 1;

    int16_t s = loopBuffer[blockIndex][offsetInBlock];
    int y = sampleToY(s);
  int lineTop = 104;
  int lineBottom = y;
  if (lineBottom < lineTop) { int tmpSwap = lineTop; lineTop = lineBottom; lineBottom = tmpSwap; }
  tft.drawFastVLine(x, lineTop, lineBottom - lineTop + 1, CYAN);
  }
}

//  Fade out Logic 
void applyFadeInOut() {
  if (blocksRecorded <= 0) return;
  const int fadeN = min(64, BLOCK_SAMPLES);
  if (fadeN <= 1) {
    loopBuffer[0][0] = 0;
    int lastBlockIndex_local = blocksRecorded - 1;
    loopBuffer[lastBlockIndex_local][BLOCK_SAMPLES - 1] = 0;
    return;
  }

  for (int fadePos = 0; fadePos < fadeN; ++fadePos) {
    float win = 0.5f * (1.0f - cosf(M_PI * (float)fadePos / (float)(fadeN - 1)));
    int32_t sampleVal = loopBuffer[0][fadePos];
    sampleVal = (int32_t)(sampleVal * win);
    loopBuffer[0][fadePos] = (int16_t)sampleVal;
  }

  int lastBlockIndex = blocksRecorded - 1;
  for (int fadePos = 0; fadePos < fadeN; ++fadePos) {
    float win = 0.5f * (1.0f - cosf(M_PI * (float)fadePos / (float)(fadeN - 1)));
    float gain = 1.0f - win;
    int fadeIndex = BLOCK_SAMPLES - fadeN + fadePos;
    if (fadeIndex < 0) fadeIndex = 0;
    int32_t sampleVal = loopBuffer[lastBlockIndex][fadeIndex];
    sampleVal = (int32_t)(sampleVal * gain);
    loopBuffer[lastBlockIndex][fadeIndex] = (int16_t)sampleVal;
  }
  int lastBlockFinal = blocksRecorded - 1;
  loopBuffer[lastBlockFinal][BLOCK_SAMPLES - 1] = 0;
}

void smoothLoopWrap() {
  if (blocksRecorded <= 1) return;
  const int CROSSFADE_SAMPLES = min(32, BLOCK_SAMPLES);
  const int firstBlockIndex = 0;
  const int lastBlockIndex = blocksRecorded - 1;
  const int fadeCount = CROSSFADE_SAMPLES;

  for (int sample = 0; sample < fadeCount; ++sample) {
    float mixT = (float)sample / (float)(fadeCount - 1);
    float gainFirst = mixT;          // ramps up
    float gainLast  = 1.0f - mixT;   // ramps down

    int lastSampleIndex = BLOCK_SAMPLES - fadeCount + sample;

    int32_t lastVal  = (int32_t)loopBuffer[lastBlockIndex][lastSampleIndex];
    int32_t firstVal = (int32_t)loopBuffer[firstBlockIndex][sample];

    int32_t mixed = (int32_t)(lastVal * gainLast + firstVal * gainFirst);

    // clamp to int16_t range
    if (mixed > 32767) mixed = 32767;
    else if (mixed < -32768) mixed = -32768;

    loopBuffer[lastBlockIndex][lastSampleIndex] = (int16_t)mixed;
  }
}

//  Recording/Playback Control 
void startArmedRecord(){
  isPlaying = false;
  isRecording = false;
  armedRecord = true;
  recordQueue.clear();
  if (!isSDRecording) {
    // In normal RAM record mode we clear the buffer. In overdub mode we keep
    // the existing loop so live input can layer on top while listening.
    if (!overdubEnabled) {
      blocksRecorded = 0;
      memset(loopBuffer, 0, sizeof(loopBuffer));
      playbackBlockIndex = 0;
    }
  }
  if (isSDRecording) {
  mainMixer.gain(0, 0.0f); 
  } else {
  mainMixer.gain(0, 1.0f); 
  }
  recordQueue.begin();      
  current = ST_ARMED; drawStatus(current);
}


// Find next available ###.WAV 
String nextRecFilename() {
  int recIndex = 1;
  char fnameBuf[16];
  while (recIndex < 1000) {
    snprintf(fnameBuf, sizeof(fnameBuf), "/REC%03d.WAV", recIndex);
  if (!sdfs.exists(fnameBuf)) return String(fnameBuf);
    recIndex++;
  }
  return String("/REC999.WAV");
}

void startSDRecord(const char* path) {
  if (!isSDRecording) return;
  if (!sdfsReady) {
    return;
  }
  // Try common open modes; SdFs may require O_CREAT | O_WRITE
  wavFile = sdfs.open(path, FILE_WRITE);
  if (!wavFile) {
    Serial.print("startSDRecord: FILE_WRITE open failed for "); Serial.println(path);
    wavFile = sdfs.open(path, O_CREAT | O_WRITE);
  }
  if (wavFile) {
    Serial.print("startSDRecord: opened "); Serial.println(path);
    writeWavHeader(wavFile, 44100, 16, 1);
    samplesWritten = 0;
  } else {
    Serial.print("startSDRecord: could not open "); Serial.println(path);
  }
}

void stopSDRecord() {
  if (!isSDRecording || !wavFile) return;
  Serial.print("stopSDRecord: samplesWritten="); Serial.println(samplesWritten);
  patchWavSizes(wavFile, samplesWritten * 2);
  wavFile.close();
  Serial.println("stopSDRecord: wavFile closed");
}

void startRecordingNow(){
  isRecording = true;
  armedRecord = false;
  current = ST_RECORDING; drawStatus(current);
  if (isSDRecording) {
    mix.gain(0, 0.0f); 
    static String lastRecFile = "";
    lastRecFile = nextRecFilename();
    startSDRecord(lastRecFile.c_str());
    // Display filename above SD Blocks
    tft.fillRect(0, 96, 128, 12, BLACK); 
    tft.setCursor(4, 98);
    tft.setTextColor(CYAN);
    tft.print("Rec: ");
    tft.print(lastRecFile);
  } else {
    // RAM mode: clear buffer and reset
    mix.gain(0, 1.0f);
    if (!overdubEnabled) {
      blocksRecorded = 0;
      loopPlayIndex = 0;
      memset(loopBuffer, 0, sizeof(loopBuffer));
    }
  }
}

void stopRecording() {
  if (!(isRecording || armedRecord)) return;
  recordQueue.end();
  stopSDRecord();
  isRecording = false;
  armedRecord = false;
  mix.gain(0, 1.0f);
  bool hadAudio = (blocksRecorded > 0);
  if (!isSDRecording && hadAudio) {
    applyFadeInOut();
    int16_t m = 0;
    for (int b = 0; b < blocksRecorded; ++b) {
      for (int i = 0; i < BLOCK_SAMPLES; ++i) {
        int16_t s = abs(loopBuffer[b][i]);
        if (s > m) m = s;
      }
    }
    waveformPeak = max((int16_t)m, (int16_t)256);     
    waveformGain = 20.0f / (waveformPeak / 32768.0f); 
    current = ST_RECORDED;
  } else if (isSDRecording && hadAudio) {
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
  stopSDPlay();
  sdPolyPlaying = false;
  current = ST_STOPPED; drawStatus(current);
  // Always restore dry/live when idle
  mix.gain(0, 1.0f); // dry/live restored
  mix.gain(1, 0.8f); // loop path back to default
  mix.gain(2, 0.0f); // SD poly mix muted
}

// SD playback loop logic 
void handleSDPlaybackLoop() {
  if (playFromSD && sdPlaying && playFile) {
    if (playFile.available() == 0) {
      // End of file
      playFile.seek(44);
    }
  }
}

//  Polyphonic SD playback 
void startSDPolyPlay(const char* path1, const char* path2) {
  if (!sdfsReady) { Serial.println("startSDPolyPlay: sdfs not ready"); return; }
  // Queued SD playback is stopped
  stopSDPlay();
  // Stop voices
  playSd1.stop();
  playSd2.stop();
  sdPolyPlaying = true;
  polyMix.gain(0, 1.0f); // voice 1
  polyMix.gain(1, 1.0f); // voice 2
  polyMix.gain(2, 0.0f); // future use
  polyMix.gain(3, 0.0f); // future use
  // Mute RAM loop; keep dry/live path if overdubbing
  mix.gain(0, overdubEnabled ? 1.0f : 0.0f);
  mix.gain(1, 0.0f); // RAM loop muted
  mix.gain(2, 1.0f); // SD poly mix to main mix
  delay(10);
  Serial.print("startSDPolyPlay: "); Serial.print(path1); Serial.print(" , "); Serial.println(path2);
  playSd1.play(path1);
  playSd2.play(path2);
  delay(20);
  Serial.print("playSd1.isPlaying="); Serial.println(playSd1.isPlaying());
  Serial.print("playSd2.isPlaying="); Serial.println(playSd2.isPlaying());
}

void stopSDPolyPlay() {
  if (!sdPolyPlaying) return;
  playSd1.stop();
  playSd2.stop();
  sdPolyPlaying = false;
  // restore mix routing
  mix.gain(0, 1.0f);
  mix.gain(1, 0.8f);
  mix.gain(2, 0.0f);
}

void startSDPlay(const char* path) {
  stopSDPlay();                 
  flushPlaybackQueue();       

  mix.gain(0, overdubEnabled ? 1.0f : 0.0f);
  mix.gain(1, 1.0f);           
  mix.gain(2, 0.0f);           

  playFile = sdfs.open(path, FILE_READ);
  if (!playFile) {
    Serial.println("SD: open failed");
    sdPlaying = false; playFromSD = false; return;
  }

  WavInfo wi = parseWavHeader(playFile);
  if (!wi.ok) {
    Serial.println("SD: unsupported WAV (need 44.1k mono 16-bit)");
    sdPlaying = false; playFromSD = false; return;
  }
  Serial.print("SD WAV: sr="); Serial.print(wi.sampleRate);
  Serial.print(" ch="); Serial.print(wi.channels);
  Serial.print(" bits="); Serial.print(wi.bits);
  Serial.print(" bytes="); Serial.println(wi.dataBytes);

  sdBlocksToPlay = wi.dataBytes / (BLOCK_SAMPLES * sizeof(int16_t));
  if (sdBlocksToPlay < 1) sdBlocksToPlay = 1;

  rdIndexA = rdIndexB = 0;
  validA = validB = 0;
  bufAReady = bufBReady = false;
  useA = true;

  flushPlaybackQueue();

  sdPlaying = true;
  playFromSD = true;
  Serial.print("SD: play "); Serial.print(path);
  Serial.print(" blocks="); Serial.println(sdBlocksToPlay);
}

void stopSDPlay() {
  if (playFile) playFile.close();
  sdPlaying = false;
  playFromSD = false;
  bufAReady = bufBReady = false;
  rdIndexA = rdIndexB = validA = validB = 0;
  flushPlaybackQueue();     
  // restore live and loop defaults
  mix.gain(0, 1.0f);
  mix.gain(1, 0.8f);
  mix.gain(2, 0.0f);
}

//  Delete sample 
void deleteSample() {
  isRecording = false;
  isPlaying = false;
  armedRecord = false;
  stopSDRecord();
  stopSDPlay();
  blocksRecorded = 0;
  loopPlayIndex = 0;
  memset(loopBuffer, 0, sizeof(loopBuffer));
  mix.gain(0, 1.0f); 
  current = ST_READY;
  drawStatus(current);
  drawCounts(0);
  // Clear waveform area
  tft.fillRect(0, 96, 128, 32, BLACK);
}


void writeWavHeader(FsFile &f, uint32_t sampleRate, uint16_t bitsPerSample, uint16_t channels) {
  uint8_t hdr[44];
  uint32_t byteRate = sampleRate * channels * (bitsPerSample/8);
  uint16_t blockAlign = channels * (bitsPerSample/8);
  // RIFF chunk
  memcpy(hdr+0,  "RIFF", 4);
  *(uint32_t*)(hdr+4)  = 0;             
  memcpy(hdr+8,  "WAVE", 4);
  // fmt chunk
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

void patchWavSizes(FsFile &f, uint32_t dataBytes) {

  uint32_t riffSize = 36 + dataBytes;
  f.seek(4);  f.write((uint8_t*)&riffSize, 4);
  f.seek(40); f.write((uint8_t*)&dataBytes, 4);
}

//  SD playback
void refillIfNeeded() {
  if (!sdPlaying || !playFile) return;

  if (!bufAReady) {
    int32_t bytesRead = playFile.read((uint8_t*)sdBufA, BUF_SAMPLES * sizeof(int16_t));
    if (bytesRead <= 0) {
      playFile.seek(44);
      bytesRead = playFile.read((uint8_t*)sdBufA, BUF_SAMPLES * sizeof(int16_t));
    }
    validA = (uint32_t)(bytesRead / sizeof(int16_t));
    rdIndexA = 0;
    bool oldA = bufAReady;
    bufAReady = (validA >= BLOCK_SAMPLES);
    if (bytesRead <= 0) Serial.println("refillIfNeeded: bytesReadA<=0");
    if (bufAReady && !oldA) {
      Serial.print("refillIfNeeded: bufAReady validA="); Serial.println(validA);
    }
  }

  if (!bufBReady) {
    int32_t bytesRead = playFile.read((uint8_t*)sdBufB, BUF_SAMPLES * sizeof(int16_t));
    if (bytesRead <= 0) {
      playFile.seek(44);
      bytesRead = playFile.read((uint8_t*)sdBufB, BUF_SAMPLES * sizeof(int16_t));
    }
    validB = (uint32_t)(bytesRead / sizeof(int16_t));
    rdIndexB = 0;
    bool oldB = bufBReady;
    bufBReady = (validB >= BLOCK_SAMPLES);
    if (bytesRead <= 0) Serial.println("refillIfNeeded: bytesReadB<=0");
    if (bufBReady && !oldB) {
      Serial.print("refillIfNeeded: bufBReady validB="); Serial.println(validB);
    }
  }
}


void queueFromBuffer() {
  static uint32_t sdPlayedBlockCount = 0;
  if (!sdPlaying) { sdPlayedBlockCount = 0; return; }

  int slots = playbackQueue.available();
  while (slots-- > 0) {
    int16_t *destBuf = playbackQueue.getBuffer();
    if (!destBuf) break;

    int16_t *src;
    uint32_t *idx, *valid;
    bool *ready;
    if (useA) { src = sdBufA; idx = (uint32_t*)&rdIndexA; valid = (uint32_t*)&validA; ready = (bool*)&bufAReady; }
    else      { src = sdBufB; idx = (uint32_t*)&rdIndexB; valid = (uint32_t*)&validB; ready = (bool*)&bufBReady; }

    if (!*ready || (*idx + BLOCK_SAMPLES) > *valid) {
      useA = !useA;
      if (useA) { src = sdBufA; idx = (uint32_t*)&rdIndexA; valid = (uint32_t*)&validA; ready = (bool*)&bufAReady; }
      else      { src = sdBufB; idx = (uint32_t*)&rdIndexB; valid = (uint32_t*)&validB; ready = (bool*)&bufBReady; }

      if (!*ready || (*idx + BLOCK_SAMPLES) > *valid) {
    memset(destBuf, 0, BLOCK_SAMPLES * sizeof(int16_t));
    playbackQueue.playBuffer();
    break;

      }
    }

    memcpy(destBuf, src + *idx, BLOCK_SAMPLES * sizeof(int16_t));
    *idx += BLOCK_SAMPLES;
    if (*idx >= *valid) *ready = false;

    playbackQueue.playBuffer();
    sdPlayedBlockCount++;

    if (sdPlayedBlockCount >= sdBlocksToPlay) {
      playFile.seek(44);
      sdPlayedBlockCount = 0;
      bufAReady = bufBReady = false;
      rdIndexA = rdIndexB = validA = validB = 0;
      useA = true;
      break;
    }
  }
}

//  setup  
void setup(){
    menuBtnDown.onHold([](int){
      if (uiMode == MODE_MENU) {
        uiMode = MODE_EFFECTS_MENU;
        inEffectsMenu = true;
        effectsMenuIndex = 0;
        drawEffectsMenu();
      } else if (uiMode == MODE_RAM) {
        overdubEnabled = !overdubEnabled;
        tft.fillRect(0, 96, 128, 12, BLACK);
        tft.setCursor(4, 98);
        tft.setTextColor(overdubEnabled ? CYAN : RED);
        tft.print("Overdub: "); tft.print(overdubEnabled ? "ON " : "OFF");
      }
    }, 800);
  menuBtnUp.onPress([](int){
    if (inPlaybackScreen) {
      inPlaybackScreen = false;
      drawMenu();
      return;
    }
    if (uiMode == MODE_EFFECTS_MENU) {
      effectsMenuIndex--;
      if (effectsMenuIndex < 0) effectsMenuIndex = FX_COUNT - 1; 
      drawEffectsMenu();
      return;
    }
    if (uiMode == MODE_SDREC) {
  sdRecMaxBlocks += 500;
  if (sdRecMaxBlocks > 4000) sdRecMaxBlocks = 4000;
      tft.fillRect(0, 108, 128, 12, BLACK);
      tft.setCursor(4, 108);
      tft.setTextColor(CYAN);
      tft.print("SD Blocks: ");
      tft.print(sdRecMaxBlocks);
    } else if (uiMode == MODE_MENU) {
      menuIndex--;
      if (menuIndex < 0) menuIndex = 0;
      drawMenu();
    }
  });
  menuBtnDown.onPress([](int){
    if (inPlaybackScreen) {
      inPlaybackScreen = false;
      drawMenu();
      return;
    }
    if (uiMode == MODE_EFFECTS_MENU) {
      effectsMenuIndex++;
      if (effectsMenuIndex >= FX_COUNT) effectsMenuIndex = 0; 
      drawEffectsMenu();
      return;
    }
    if (uiMode == MODE_SDREC) {
  sdRecMaxBlocks -= 500;
  if (sdRecMaxBlocks < 100) sdRecMaxBlocks = 100;
      tft.fillRect(0, 108, 128, 12, BLACK); 
      tft.setCursor(4, 108);
      tft.setTextColor(CYAN);
      tft.print("SD Blocks: ");
      tft.print(sdRecMaxBlocks);
    } else if (uiMode == MODE_MENU) {
      menuIndex++;
      if (menuIndex >= menuLength) menuIndex = 0; 
    drawMenu();
    } else if (uiMode == MODE_RAM) {
      currentFX = (FXType)((currentFX + 1) % FX_COUNT);
      tft.fillRect(0, 40, 128, 20, BLACK);
      float d = 0.0f;
      switch (currentFX) {
        case FX_REVERB: d = reverbDepth; break;
        case FX_BITCRUSH: d = bitcrushDepth; break;
        case FX_LOWPASS: d = lowpassDepth; break;
        default: break;
      }
      drawFX(d, analogRead(POT_VOL) / 1023.0f);
    }
  });
  pinMode(INPUT_SWITCH_PIN, INPUT_PULLUP); 

  // Button 1: Record/Arm in RAM or SDREC modes; toggle poly/mono in menu mode
  button1.onPress([](int){
    // toggle or record based on uiMode
    if (uiMode == MODE_SDREC) {
      // Start armed record or stop recording
      if (!isRecording && !armedRecord) {
        startArmedRecord();
      } else {
        stopRecording();
      }
      return;
    }
    if (uiMode == MODE_RAM) {
      // RAM mode: record/stop
      if (!isRecording && !armedRecord) {
        startArmedRecord();
      } else {
        stopRecording();
      }
      return;
    }
    if (uiMode == MODE_MENU) {
      sdPolyMode = !sdPolyMode;
      tft.fillRect(0, 120, 128, 8, BLACK);
      tft.setCursor(4, 120);
      tft.setTextColor(CYAN);
      tft.print(sdPolyMode ? "Poly SDPlay" : "Mono SDPlay");
    }
  });

  // Button 2: Select in menu , play in other modes
  button2.onPress([](int){
    if (uiMode == MODE_EFFECTS_MENU) {
      // Set selected effect for SD playback and return to menu
      sdPlayFX = (FXType)effectsMenuIndex;
      uiMode = MODE_MENU;
      inEffectsMenu = false;
      applyEffectsNow();
      drawMenu();
      return;
    }
    if (uiMode == MODE_MENU) {
      if (sdPolyMode) {
        int selIndex1 = menuIndex;
        int selIndex2 = (menuIndex + 1) % menuLength;
        String selName1 = String(menuItems[selIndex1]);
        String selName2 = String(menuItems[selIndex2]);
        if (!selName1.endsWith("/") && !selName2.endsWith("/")) {
          String fullPath1 = currentPath + selName1;
          String fullPath2 = currentPath + selName2;
          startSDPolyPlay(fullPath1.c_str(), fullPath2.c_str());
          current = ST_PLAYING;
          drawStatus(current);
          // Show playback screen for first file
          inPlaybackScreen = true;
          playbackFile = fullPath1;
          drawPlaybackScreen();
        }
    } else {
        // Monophonic: play only selected file
        int selIndex = menuIndex;
        String selName = String(menuItems[selIndex]);
        if (!selName.endsWith("/")) {
          String fullPath1 = currentPath + selName;
      // ensure no poly voices are active
  stopSDPolyPlay();
  stopSDPlay();
  flushPlaybackQueue();
      startSDPlay(fullPath1.c_str());
          current = ST_PLAYING;
          drawStatus(current);
          inPlaybackScreen = true;
          playbackFile = fullPath1;
          drawPlaybackScreen();
        }
      }
    } else {
      if (isRecording || armedRecord) stopRecording();
      if (isSDRecording) {
        playFromSD = true;
        startSDPlay("/loop.wav");
        current = ST_PLAYING; drawStatus(current);
      } else {
        playFromSD = false;
        startPlayback();
      }
    }
  });

 // Re-enable effect on hold in RAM mode
  static float lastReverbDepth = 0.3f;
  static float lastBitcrushDepth = 0.3f;
  static float lastLowpassDepth = 0.3f;
  button3.onPress([](int){
    if (uiMode == MODE_EFFECTS_MENU) {
      sdPlayFXEnabled[effectsMenuIndex] = !sdPlayFXEnabled[effectsMenuIndex];
      applyEffectsNow();
      drawEffectsMenu();
      return;
    }
    if (uiMode == MODE_RAM) {
      switch (currentFX) {
        case FX_REVERB:
          lastReverbDepth = reverbDepth > 0.01f ? reverbDepth : lastReverbDepth;
          reverbDepth = 0.0f;
          reverb.roomsize(0.01f);
          break;
        case FX_BITCRUSH:
          lastBitcrushDepth = bitcrushDepth > 0.01f ? bitcrushDepth : lastBitcrushDepth;
          bitcrushDepth = 0.0f;
          // bitcrushFX.bits(16);
          // bitcrushFX.sampleRate(1.0f);
          break;
        case FX_LOWPASS:
          lastLowpassDepth = lowpassDepth > 0.01f ? lowpassDepth : lastLowpassDepth;
          lowpassDepth = 0.0f;
          lowpassFX.setLowpass(0, 12000.0f, 0.707f);
          break;
        default:
          break;
      }
      tft.fillRect(0, 40, 128, 20, BLACK);
      drawFX(0.0f, analogRead(POT_VOL) / 1023.0f);
      tft.setCursor(4, 60);
      tft.setTextColor(RED);
      tft.print("Effect Disabled");
    } else if (uiMode != MODE_MENU) {
      stopRecording();
      stopPlayback();
      stopSDRecord();
      current = ST_STOPPED; drawStatus(current);
    }
    // else: do nothing in menu mode
  });

  // Re-enable effect on hold in RAM mode
  button3.onHold([](int){
    if (uiMode == MODE_RAM) {
      switch (currentFX) {
        case FX_REVERB:
          reverb.roomsize(reverbDepth > 0.01f ? (reverbDepth * 0.95f) : 0.01f);
          break;
        case FX_BITCRUSH:
          break;
        case FX_LOWPASS:
          {
            float minCutoff = 500.0f;
            float maxCutoff = 12000.0f;
            float cutoff = maxCutoff - lowpassDepth * (maxCutoff - minCutoff);
            if (lowpassDepth < 0.01f) cutoff = maxCutoff;
            lowpassFX.setLowpass(0, cutoff, 0.707f);
          }
          break;
        default:
          break;
      }
      tft.fillRect(0, 40, 128, 20, BLACK);
      drawFX((currentFX == FX_REVERB ? reverbDepth : currentFX == FX_BITCRUSH ? bitcrushDepth : lowpassDepth), analogRead(POT_VOL) / 1023.0f);
      tft.setCursor(4, 60);
      tft.setTextColor(GREEN);
      tft.print("Effect Enabled");
    }
    // else: do nothing in other modes
  });

  // Delete Button
  button4.onPress([](int){
    if (uiMode == MODE_MENU) {
      String sel = String(menuItems[menuIndex]);
      if (!sel.endsWith("/")) {
        // File: delete it
        String fullPath = currentPath + sel;
        if (sdfs.exists(fullPath.c_str())) {
          sdfs.remove(fullPath.c_str());
          tft.setCursor(4, 120);
          tft.setTextColor(RED);
          tft.print("Deleted: ");
          tft.print(sel);
          scanSDMenu(currentPath);
          drawMenu();
        }
      }
    } else {
      deleteSample();
    }
  });

  // Mode toggle button
  button5.onPress([](int){
    // Cycle mode
    uiMode = static_cast<UIMode>((uiMode + 1) % 3);
    // Reset all state and buffers when switching modes
    isRecording = false;
    isPlaying = false;
    armedRecord = false;
    stopSDRecord();
    stopSDPlay();
  stopPlayback();
  flushPlaybackQueue();
    blocksRecorded = 0;
    loopPlayIndex = 0;
    memset(loopBuffer, 0, sizeof(loopBuffer));
    // Set SD recording flag
    isSDRecording = (uiMode == MODE_SDREC);
    // set mixer gains for RAM mode when entering RAM mode
    if (uiMode == MODE_RAM) {
      mix.gain(0, 1.0f);   
      mix.gain(1, 0.8f);   
      mix.gain(2, 0.0f);   
    }
    drawHeader();
    drawStatus(ST_READY);
    drawCounts(0);
    tft.setCursor(4, 120);
    if (uiMode == MODE_SDREC) {
      tft.setTextColor(GREEN); tft.print("SD Rec: ON  ");
    } else if (uiMode == MODE_RAM) {
      tft.setTextColor(RED); tft.print("SD Rec: OFF ");
    } else if (uiMode == MODE_MENU) {
      tft.setTextColor(CYAN); tft.print("MENU MODE   ");
      scanSDMenu(currentPath);
      drawMenu(); // Draw menu once when entering menu mode
    }
  });
  Serial.begin(115200);
  while (!Serial && millis() < 2000) ; 
  tft.begin();
  drawHeader(); drawStatus(ST_READY); drawFX(0.30f, 0.60f); drawCounts(0);

  AudioMemory(320);
  sgtl5000.enable();
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN); 
  sgtl5000.lineOutLevel(6);               
  sgtl5000.lineInLevel(15);               
  sgtl5000.volume(0.60f);

  hp.setHighpass(0, 20.0f, 0.707f);
  // Always set mixer gains for RAM mode at startup
  mix.gain(0, 1.0f);   // dry/live ON
  mix.gain(1, 0.8f);   // loop path default
  mix.gain(2, 0.0f);   // SD poly mix muted
  reverb.roomsize(0.25f);
  reverb.damping(0.50f);

  // Set input mixer: only line-in active
  inputMix.gain(0, 0.0f); 
  inputMix.gain(1, 1.0f); 

  // Stop button: stop both RAM and SD playback and recording
  button3.onPress([](int){
    stopRecording();
    stopPlayback();
    stopSDRecord();
    current = ST_STOPPED; drawStatus(current);
  });

  pinMode(CLIP_LED_PIN, OUTPUT);
  digitalWrite(CLIP_LED_PIN, LOW);

if (!sdfs.begin(SdioConfig(FIFO_SDIO))) {
  tft.setCursor(4, 80); tft.setTextColor(RED); tft.println("SDIO init failed");
} else {
  sdfsReady = true;
}

  drawMenu();
}

//  loop 
void loop(){
  static elapsedMillis ramTimer = 0;
  if (ramTimer > 2000) {
    ramTimer = 0;
  }
  for (int i = 0; i < 7; ++i) buttonArray[i]->process();

  // Line-in as input
  inputMix.gain(0, 0.0f); 
  inputMix.gain(1, 1.0f); 
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN); 

  if (playFromSD && sdPlaying) {
    refillIfNeeded();
  int slots = playbackQueue.available();
    int toQueue = min(slots, 8); 
    while (toQueue-- > 0 && slots-- > 0) {
      queueFromBuffer();
    }
  }

  if (uiMode == MODE_MENU) {
    if (inPlaybackScreen) {
      // Throttle playback 
      static uint32_t lastPlayDraw = 0;
      static String lastPlayFile = "";
      if (playbackFile != lastPlayFile || millis() - lastPlayDraw > 1000) {
        drawPlaybackScreen();
        lastPlayDraw = millis();
        lastPlayFile = playbackFile;
      }
      menuBtnUp.process();
      menuBtnDown.process();
      return;
    } else {
      menuBtnUp.process();
      menuBtnDown.process();
      return;
    }
  }
  // Effects menu logic
  if (uiMode == MODE_EFFECTS_MENU) {
    static int lastEffectsMenuIndex = -1;
    static bool lastEffectsMenuScreen = false;
    static bool lastFXEnabled[FX_COUNT] = {false, false, false};
    static float lastFXDepth[FX_COUNT] = {0.0f, 0.0f, 0.0f};
    bool shouldRedraw = false;
    // Redraws only if index or effect state changes
    if (lastEffectsMenuIndex != effectsMenuIndex || !lastEffectsMenuScreen) {
      shouldRedraw = true;
    }
    for (int fxIdx = 0; fxIdx < FX_COUNT; ++fxIdx) {
      if (lastFXEnabled[fxIdx] != sdPlayFXEnabled[fxIdx] || lastFXDepth[fxIdx] != sdPlayFXDepth[fxIdx]) {
        shouldRedraw = true;
        break;
      }
    }
  float fxPotVal = analogRead(POT_FX) / 1023.0f;
    static float lastFxPotVal = -1.0f;
    if (fabs(fxPotVal - lastFxPotVal) > 0.01f) { 
      tft.fillRect(64, 24, 64, 104, BLACK);
      sdPlayFXDepth[effectsMenuIndex] = fxPotVal;
      applyEffectsNow();
      switch (effectsMenuIndex) {
        case FX_REVERB: {
          int knobCenterX = 64 + 32, knobCenterY = 60, knobRadius = 20;
          tft.drawCircle(knobCenterX, knobCenterY, knobRadius, GRAY);
          float angle = 3.14159f * (0.75f + 1.5f * fxPotVal);
          int knobHandleX = knobCenterX + (int)(knobRadius * cos(angle));
          int knobHandleY = knobCenterY + (int)(knobRadius * sin(angle));
          tft.drawLine(knobCenterX, knobCenterY, knobHandleX, knobHandleY, CYAN);
          tft.setTextColor(CYAN); tft.setCursor(knobCenterX-20, knobCenterY+36);
      tft.print("Depth: "); tft.print(fxPotVal,2);
          break;
        }
        case FX_BITCRUSH: {
          int bars = 8 + (int)(fxPotVal * 8.0f);
          int barX = 68, barY = 40;
          for (int barIdx = 0; barIdx < 16; ++barIdx) {
            int barHeight = 4 + barIdx;
            uint16_t col = (barIdx < bars) ? CYAN : GRAY;
            tft.fillRect(barX + barIdx*3, barY + 32 - barHeight, 2, barHeight, col);
          }
          tft.setTextColor(CYAN); tft.setCursor(barX, barY+48);
      tft.print("Bits: "); tft.print(bars);
          break;
        }
        case FX_LOWPASS: {
          int curveBaseY = 90, curveWidth = 56, curveHeight = 24;
          float cutoff = 12000.0f - fxPotVal * (12000.0f - 500.0f);
          for (int curveX = 0; curveX < curveWidth; ++curveX) {
            float freq = 500.0f + (curveX/(float)curveWidth)*(12000.0f-500.0f);
            float curveY = curveHeight * exp(-freq/cutoff);
            tft.drawPixel(64 + curveX, curveBaseY - (int)curveY, CYAN);
          }
          tft.setTextColor(CYAN); tft.setCursor(66, curveBaseY+16);
          tft.print("Cutoff: "); tft.print((int)cutoff);
          break;
        }
        default: break;
      }
      lastFxPotVal = fxPotVal;
    }
    if (shouldRedraw) {
      drawEffectsMenu();
      lastEffectsMenuIndex = effectsMenuIndex;
      lastEffectsMenuScreen = true;
      for (int fxIdx = 0; fxIdx < FX_COUNT; ++fxIdx) {
        lastFXEnabled[fxIdx] = sdPlayFXEnabled[fxIdx];
        lastFXDepth[fxIdx] = sdPlayFXDepth[fxIdx];
      }
      lastFxPotVal = fxPotVal; 
    }
    menuBtnUp.process();
    menuBtnDown.process();
    button2.process();
    button3.process();
    return;
  }

  // Update FX and volume from pots
  float fxDepth = analogRead(POT_FX) / 1023.0f;
  float masterVolume = analogRead(POT_VOL) / 1023.0f;
  // Live hearback logic for all modes
  if (uiMode == MODE_RAM) {
    if (sdPolyPlaying) {
      mix.gain(0, 0.0f);
      mix.gain(1, 0.0f);
      mix.gain(2, 1.0f);
    } else if (!isPlaying) {
      mix.gain(0, 1.0f);
      mix.gain(1, 0.8f);
      mix.gain(2, 0.0f);
    } else {
      mix.gain(0, 0.0f);
      mix.gain(1, 1.0f);
      mix.gain(2, 0.0f);
    }
    switch (currentFX) {
      case FX_REVERB:
        reverbDepth = fxDepth;
        reverb.roomsize(reverbDepth > 0.01f ? (reverbDepth * 0.95f) : 0.01f);
        break;
      case FX_BITCRUSH:
        // Bitcrushing bypassed for now
        // bitcrushDepth = depth;
        // int bits = 16 - (int)(bitcrushDepth * 8.0f);
        // if (bits < 8) bits = 8;
        // if (bits > 16) bits = 16;
        // float downsample = 1.0f + bitcrushDepth * 7.0f;
        // bitcrushFX.bits(bits);
        // bitcrushFX.sampleRate(downsample);
        break;
      case FX_LOWPASS:
        lowpassDepth = fxDepth;
        {
          float minCutoff = 500.0f;
          float maxCutoff = 12000.0f;
          float cutoff = maxCutoff - lowpassDepth * (maxCutoff - minCutoff);
          if (lowpassDepth < 0.01f) cutoff = maxCutoff;
          lowpassFX.setLowpass(0, cutoff, 0.707f);
        }
        break;
      default:
        break;
    }
  } else if (uiMode == MODE_SDREC) {
    if (!isRecording && !armedRecord && !isPlaying && !sdPolyPlaying) {
      mix.gain(0, 1.0f);
      mix.gain(1, 0.8f);
      mix.gain(2, 0.0f);
    }
  }
  // SD playback effect processing 
  if (playFromSD && sdPlaying) {
    FXType fx = sdPlayFX;
    float fxDepth = sdPlayFXDepth[fx];
    bool fxEnabled = sdPlayFXEnabled[fx];
    switch (fx) {
      case FX_REVERB:
        reverb.roomsize(fxEnabled ? (fxDepth > 0.01f ? fxDepth * 0.95f : 0.01f) : 0.01f);
        break;
      case FX_BITCRUSH:
        // bitcrushFX.bits(fxEnabled ? (16 - (int)(fxDepth * 8.0f)) : 16);
        // bitcrushFX.sampleRate(fxEnabled ? (1.0f + fxDepth * 7.0f) : 1.0f);
        break;
      case FX_LOWPASS:
        {
          float minCutoff = 500.0f;
          float maxCutoff = 12000.0f;
          float cutoff = fxEnabled ? (maxCutoff - fxDepth * (maxCutoff - minCutoff)) : maxCutoff;
          if (fxDepth < 0.01f || !fxEnabled) cutoff = maxCutoff;
          lowpassFX.setLowpass(0, cutoff, 0.707f);
        }
        break;
      default:
        break;
    }
  }
  sgtl5000.volume(constrain(masterVolume, 0.0f, 1.0f));  
  if (!isSDRecording) {
    // UI ~20 Hz
    static elapsedMillis uiTick = 0;
    if (uiTick > 50){
  drawFX(fxDepth, masterVolume);
      if (isPlaying && blocksRecorded > 0) {
        int peak = 0;
        if (loopPlayIndex < blocksRecorded) {
          for (int i = 0; i < BLOCK_SAMPLES; ++i) {
            int val = abs(loopBuffer[loopPlayIndex][i]);
            if (val > peak) peak = val;
          }
        }
        float normPeak = (float)peak / 32768.0f;
        drawMeter(normPeak);
      } else if (peakL.available()) {
        drawMeter(peakL.read());
      }
      if (current == ST_RECORDED || current == ST_PLAYING) {
        drawWaveform();
      }
      if (countsDirty) {
        drawCounts(blocksRecorded);
        countsDirty = false;
      }
      uiTick = 0;
    }
  }

  // handle recording/arming
  if (armedRecord || isRecording){
    while (recordQueue.available()){
      const int16_t* b = (const int16_t*)recordQueue.readBuffer(); // 128 mono samples
      if (armedRecord) {
        float lvl = peakL.available() ? peakL.read() : 0.0f;
        if (lvl >= ARM_LEVEL_THRESHOLD) {
          startRecordingNow(); // switch to recording state
          // store THIS triggering block as first block
          if (isSDRecording) {
            if (wavFile && blocksRecorded < MAX_BLOCKS) {
              wavFile.write((const uint8_t*)b, BLOCK_SAMPLES * sizeof(int16_t));
              samplesWritten += BLOCK_SAMPLES;
              blocksRecorded++;
              countsDirty = true;
              yield();
              if (blocksRecorded >= MAX_BLOCKS) { stopRecording(); break; }
            }
          } else {
            if (overdubEnabled) {
              // Mix into current playhead block
              int mixBlock = loopPlayIndex % max(1, blocksRecorded);
              if (blocksRecorded == 0) {
                memcpy(loopBuffer[blocksRecorded], b, BLOCK_SAMPLES * sizeof(int16_t));
                blocksRecorded++;
                countsDirty = true;
              } else {
                for (int i = 0; i < BLOCK_SAMPLES; ++i) {
                  int32_t existing = loopBuffer[mixBlock][i];
                  int32_t in = b[i];
                  int32_t mixed = (int32_t)(existing * odFeedback + in * odInputGain);
                  if (mixed > 32767) mixed = 32767;
                  else if (mixed < -32768) mixed = -32768;
                  loopBuffer[mixBlock][i] = (int16_t)mixed;
                }
                countsDirty = true;
              }
            } else if (blocksRecorded < MAX_BLOCKS) {
              memcpy(loopBuffer[blocksRecorded], b, BLOCK_SAMPLES * sizeof(int16_t));
              blocksRecorded++;
              countsDirty = true;
              if (blocksRecorded >= MAX_BLOCKS) { stopRecording(); break; }
            }
          }
        }
  recordQueue.freeBuffer();
        continue;
      }

      // Active recording
      if (isRecording) {
        if (isSDRecording) {
          // SD mode: write to SD only
          if (wavFile && blocksRecorded < sdRecMaxBlocks) {
              wavFile.write((const uint8_t*)b, BLOCK_SAMPLES * sizeof(int16_t));
            samplesWritten += BLOCK_SAMPLES;
            blocksRecorded++;
            countsDirty = true;
            yield(); // allow UI and other tasks to run
            if (blocksRecorded >= sdRecMaxBlocks) { stopRecording(); break; }
          }
        } else {
          // RAM mode: store in RAM only
          if (overdubEnabled) {
            // Mix input into current playhead block with feedback
            int targetBlock = blocksRecorded; // append if at end
            if (targetBlock >= MAX_BLOCKS) targetBlock = MAX_BLOCKS - 1;
            if (blocksRecorded < MAX_BLOCKS && (blocksRecorded == 0 || loopPlayIndex >= blocksRecorded)) {
              memcpy(loopBuffer[blocksRecorded], b, BLOCK_SAMPLES * sizeof(int16_t));
              blocksRecorded++;
              countsDirty = true;
              if (blocksRecorded >= MAX_BLOCKS) { stopRecording(); break; }
            } else {
              int mixBlock = loopPlayIndex % max(1, blocksRecorded);
              for (int i = 0; i < BLOCK_SAMPLES; ++i) {
                int32_t existing = loopBuffer[mixBlock][i];
                int32_t in = b[i];
                int32_t mixed = (int32_t)(existing * odFeedback + in * odInputGain);
                if (mixed > 32767) mixed = 32767;
                else if (mixed < -32768) mixed = -32768;
                loopBuffer[mixBlock][i] = (int16_t)mixed;
              }
              countsDirty = true;
            }
          } else {
            if (blocksRecorded < MAX_BLOCKS) {
              memcpy(loopBuffer[blocksRecorded], b, BLOCK_SAMPLES * sizeof(int16_t));
              blocksRecorded++;
              countsDirty = true;
              if (blocksRecorded >= MAX_BLOCKS) { stopRecording(); break; }
            }
          }
        }
  recordQueue.freeBuffer();
      }
    }
  }

  // RAM playback
  if (isPlaying && !playFromSD && blocksRecorded > 0) {
    int slots = playQ.available();
    int toQueue = min(slots, 6); 
    while (toQueue-- > 0 && slots-- > 0) {
      if (loopPlayIndex >= blocksRecorded) loopPlayIndex = 0;
  int16_t *outBuffer = playbackQueue.getBuffer();
      if (outBuffer) {
        memcpy(outBuffer, loopBuffer[loopPlayIndex], BLOCK_SAMPLES * sizeof(int16_t));
  playbackQueue.playBuffer();
        loopPlayIndex++;
      } else {
        break;
      }
    }
  }

  // SD playback 
  if (playFromSD && sdPlaying) {
    refillIfNeeded();
  int slots = playbackQueue.available();
    int toQueue = min(slots, 8); 
    while (toQueue-- > 0 && slots-- > 0) {
      queueFromBuffer();
    }
  }
  menuBtnUp.process();
  menuBtnDown.process();
}
