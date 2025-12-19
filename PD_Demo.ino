enum FXType { FX_TAPE_SPEED, FX_TREMOLO, FX_LOWPASS, FX_REVERB, FX_COUNT };
FXType currentFX = FX_REVERB;
float reverbDepth = 0.3f;
float lowpassDepth = 0.3f;
enum PotFxMode { POTFX_REVERB_LP, POTFX_TREMOLO, POTFX_TAPE_SPEED, POTFX_MODE_COUNT };
PotFxMode potFxMode = POTFX_REVERB_LP;
enum FxTarget { FX_TGT_BOTH, FX_TGT_A, FX_TGT_B, FX_TGT_COUNT };
FxTarget fxTarget = FX_TGT_BOTH;
float tremoloDepth = 0.0f;
float tremoloRate  = 5.0f;
float tremoloPhaseA = 0.0f;
float tremoloPhaseB = 0.0f;
float tapeSpeedRate = 1.0f;
enum UIMode { MODE_RAM, MODE_SDREC, MODE_MENU };
UIMode uiMode = MODE_RAM;

float usbGain = 1.0f;
float lineInGain = 0.0f;
enum ClipSlot { CLIP_NONE, CLIP_A, CLIP_B };

#include <cstdint>
#include <cstdint>
#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <malloc.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include "BetterButton.h"
SdFs sdfs;
bool sdfsReady = false;


#include "status.h"
#include "functions.h"

struct WavInfo {
  uint32_t dataOffset = 0;
  uint32_t dataBytes  = 0;
  uint16_t channels   = 1;
  uint32_t sampleRate = 44100;
  uint16_t bits       = 16;
  bool ok             = false;
};

inline float readPotInv(int pin) {
  return 1.0f - (analogRead(pin) / 1023.0f);
}


#define OLED_CS    4
#define OLED_DC    9
#define OLED_RST   5
Adafruit_SSD1351 tft(128, 128, &SPI, OLED_CS, OLED_DC, OLED_RST);
#define BLACK 0x0000
#define WHITE 0xFFFF
#define RED   0xF800
#define GREEN 0x07E0
#define BLUE  0x001F
#define YELLOW 0xFFE0
#define CYAN  0x07FF
#define GRAY  0x8410


AudioInputUSB      usbIn;
AudioInputI2S      lineIn;
AudioMixer4         inputMix;
AudioFilterBiquad   hp;
AudioRecordQueue    recQ;
AudioRecordQueue&   recordQueue = recQ;
AudioRecordQueue    bounceQ;
AudioPlayQueue      playQ;
AudioPlayQueue&     playbackQueue = playQ;
AudioPlaySdWav      playSd1;
AudioPlaySdWav&     sdPlayer1 = playSd1;
AudioPlaySdWav      playSd2;
AudioPlaySdWav&     sdPlayer2 = playSd2;
AudioMixer4         polyMix;
AudioMixer4&        sdPolyMixer = polyMix;
AudioMixer4         mix;
AudioMixer4&        mainMixer = mix;
AudioMixer4         outMix;
bool sdPolyPlaying = false;
bool sdPolyMode = true;
AudioEffectFreeverb reverb;
AudioFilterBiquad lowpassFX;
AudioOutputI2S      i2sOut;
AudioAnalyzePeak    peakL;
AudioControlSGTL5000 sgtl5000;

AudioConnection pc0(usbIn, 0, inputMix, 0);
AudioConnection pc0b(lineIn, 0, inputMix, 1);
AudioConnection pc1(inputMix, 0, hp, 0);
AudioConnection pc2(hp,    0, recordQueue, 0);
AudioConnection pc3(hp,    0, mix,  0);
AudioConnection pc4(playbackQueue, 0, mix,  1);
AudioConnection pcSd1(playSd1, 0, polyMix, 0);
AudioConnection pcSd2(playSd2, 0, polyMix, 1);
AudioConnection pcPolyMix(polyMix, 0, mix, 2);
AudioConnection pcMixToReverb(mix, 0, reverb, 0);
AudioConnection pcDry(mix, 0, outMix, 0);
AudioConnection pcWet(reverb, 0, outMix, 1);
AudioConnection pcOutMixToLP(outMix, 0, lowpassFX, 0);
AudioConnection pc7(lowpassFX,0, i2sOut, 0);
AudioConnection pc7b(lowpassFX,0, i2sOut, 1);
AudioConnection pc8(inputMix, 0, peakL, 0);
AudioConnection pcBounce(lowpassFX, 0, bounceQ, 0);


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


const int PIN_SMP_BTN_1 = 32;
const int PIN_SMP_BTN_2 = 31;

const int PIN_REC_BTN    = 33;
const int PIN_PLAY_BTN   = 34;
const int PIN_LOOP_BTN   = 30;
const int PIN_SELECT_BTN = 29;
const int PIN_OD_BTN     = 35;

const int PIN_ENC_A  = 27;
const int PIN_ENC_B  = 28;
const int PIN_ENC_SW = 37;

const int PIN_FX1_POT    = A12;
const int PIN_FX2_POT    = A11;
const int PIN_MASTER_POT = A10;

const int PIN_SMPVOL1_POT = A14;
const int PIN_SMPVOL2_POT = A15;
const int PIN_SMPVOL3_POT = A16;
const int PIN_SMPVOL4_POT = A17;

const int PCB_SPI_MOSI = 11;
const int PCB_SPI_MISO = 12;
const int PCB_SPI_SCK  = 13;

const int POT_FX  = PIN_SMPVOL1_POT;
const int POT_VOL = PIN_MASTER_POT;

bool useUsbInput = true;


BetterButton recButton(PIN_REC_BTN, 0, INPUT_PULLUP);
BetterButton playButton(PIN_PLAY_BTN, 1, INPUT_PULLUP);
BetterButton loopButton(PIN_LOOP_BTN, 2, INPUT_PULLUP);
BetterButton selectButton(PIN_SELECT_BTN, 3, INPUT_PULLUP);
BetterButton sampleBtn1(PIN_SMP_BTN_1, 4, INPUT_PULLUP);
BetterButton sampleBtn2(PIN_SMP_BTN_2, 5, INPUT_PULLUP);
BetterButton encoderBtn(PIN_ENC_SW, 6, INPUT_PULLUP);
BetterButton overdubButton(PIN_OD_BTN, 7, INPUT_PULLUP);

BetterButton* buttonArray[] = { &recButton, &playButton, &loopButton, &selectButton,
                                &sampleBtn1, &sampleBtn2, &encoderBtn, &overdubButton };
const size_t BUTTON_COUNT = sizeof(buttonArray) / sizeof(buttonArray[0]);

volatile long encoderPos = 0;
int lastEncState = 0;

int readEncoderDelta() {
  int a = digitalRead(PIN_ENC_A);
  int b = digitalRead(PIN_ENC_B);
  uint8_t state = (a << 1) | b;
  static const int8_t table[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
   -1, 0, 0, 1,
    0, 1, -1, 0
  };
  static uint8_t lastState = 0;
  static bool initialized = false;
  static int8_t movement = 0;

  if (!initialized) {
    lastState = state;
    initialized = true;
    return 0;
  }

  uint8_t idx = (lastState << 2) | state;
  int8_t step = table[idx];
  int delta = 0;
  if (step != 0) {
    movement += step;
    if (movement >= 4) {
      delta = 1;
      movement = 0;
    } else if (movement <= -4) {
      delta = -1;
      movement = 0;
    }
  }
  lastState = state;
  delta = -delta;          // flip direction to match physical rotation
  encoderPos += delta;
  return delta;
}


const int BLOCK_SAMPLES = 128;
const int MAX_BLOCKS    = 700;
int sdRecMaxBlocks = 4000;

DMAMEM __attribute__((aligned(32)))
int16_t loopBuffer[MAX_BLOCKS][BLOCK_SAMPLES];
volatile int blocksRecorded = 0;
volatile int& recordedBlockCount = blocksRecorded;
static int loopPlayIndex = 0;
static int& playbackBlockIndex = loopPlayIndex;
int16_t waveformPeak = 512;
float   waveformGain = 1.0f;
float tremoloPhaseSD = 0.0f;

const uint32_t MAX_CLIP_SAMPLES = 44100UL * 30UL;
EXTMEM int16_t clipA[MAX_CLIP_SAMPLES];
EXTMEM int16_t clipB[MAX_CLIP_SAMPLES];
uint32_t clipSamplesA = 0, clipSamplesB = 0;
uint32_t overdubPosA = 0, overdubPosB = 0;
float clipGainA = 1.0f;
float clipGainB = 1.0f;
bool clipReadyA = false, clipReadyB = false;
float clipPlayPosA = 0.0f, clipPlayPosB = 0.0f;
bool playingClipA = false, playingClipB = false;

ClipSlot recordSlot = CLIP_NONE;
void saveClipToSD(ClipSlot slot);

float tapBpm = 0.0f;
uint32_t lastTapMs = 0;
int tapCount = 0;
const int TAP_HISTORY = 8;
float tapIntervals[TAP_HISTORY] = {0};
int tapIntervalCount = 0;
int tapIntervalIndex = 0;
const int MIN_INTERVALS_FOR_BPM = 3;
uint32_t msPerBeat = 0;
int polySelectionStage = 0;
String polyPath1 = "";
String polyPath2 = "";
bool pendingPolyStart = false;
uint32_t polyStartAtMs = 0;

void showBpm() {
  tft.fillRect(0, 120, 128, 8, BLACK);
  tft.setCursor(4, 120);
  tft.setTextColor(CYAN);
  tft.print("BPM: "); tft.print(tapBpm, 1);
  if (tapBpm > 0.0f) {
    tft.print(" QNT");
  }
}

void writeFooter(const String& msg, uint16_t color) {
  tft.fillRect(0, 120, 128, 8, BLACK);
  tft.setCursor(4, 120);
  tft.setTextColor(color);
  tft.print(msg);
}

bool registerTapTempo() {
  uint32_t now = millis();
  if (now - lastTapMs > 2000) {
    tapCount = 0;
    tapBpm = 0.0f;
    tapIntervalCount = 0;
    tapIntervalIndex = 0;
  }
  tapCount++;
  bool bpmReady = false;
  if (tapCount >= 2) {
    float interval = (float)(now - lastTapMs);
    tapIntervals[tapIntervalIndex] = interval;
    tapIntervalIndex = (tapIntervalIndex + 1) % TAP_HISTORY;
    if (tapIntervalCount < TAP_HISTORY) tapIntervalCount++;

    int used = min(tapIntervalCount, TAP_HISTORY);
    if (used >= MIN_INTERVALS_FOR_BPM) {
      float sum = 0.0f;
      for (int i = 0; i < used; ++i) sum += tapIntervals[i];
      float avgInterval = sum / (float)used;
      float bpmRaw = 60000.0f / max(avgInterval, 1.0f);
      tapBpm = roundf(bpmRaw);
      msPerBeat = (uint32_t)(60000.0f / max(tapBpm, 1.0f));
      tapeSpeedRate = constrain(tapBpm / 120.0f, 0.5f, 2.0f);
      showBpm();
      bpmReady = true;
    }
  }
  lastTapMs = now;
  return bpmReady;
}

void resetPolyPrep() {
  polySelectionStage = 0;
  polyPath1 = "";
  polyPath2 = "";
  pendingPolyStart = false;
  polyStartAtMs = 0;
}

bool pendingClipA = false;
bool pendingClipB = false;
uint32_t nextBeatMs = 0;
bool isRecording   = false;
bool isPlaying     = false;
bool armedRecord   = false;
bool& recordingState = isRecording;
bool& playingState = isPlaying;
bool& armedState = armedRecord;
const float ARM_LEVEL_THRESHOLD = 0.02f;
bool loopModeEnabled = true;

bool loopCaptured = false;
bool overdubEnabled = false;
float odInputGain = 0.8f;
float odFeedback  = 0.7f;
volatile bool countsDirty = false;
bool pausedPlayback = false;

FsFile wavFile;
volatile uint32_t samplesWritten = 0;
bool isSDRecording = false;
bool bounceActive = false;
ClipSlot bounceSlot = CLIP_NONE;
FsFile bounceFile;
uint32_t bounceSamples = 0;

bool playFromSD = false;
uint32_t sdBlocksToPlay = 0;
uint32_t sdBlocksBase = 0;
float sdPosFrac = 0.0f;

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

const int CLIP_LED_PIN = 2;
AudioAnalyzePeak peakOut;
bool clipLatched = false;
AudioConnection pcOutMeter(lowpassFX, 0, peakOut, 0);



Status current = ST_READY, lastDrawn = (Status)255;


void drawStatus(Status s);


#define MAX_MENU_ITEMS 32
char menuItems[MAX_MENU_ITEMS][32];
int menuLength = 0;
int menuIndex = 0;
String currentPath = "/";


FXType sdPlayFX = FX_REVERB;
bool sdPlayFXEnabled[FX_COUNT] = {true, true, true, true};
float sdPlayFXDepth[FX_COUNT] = {0.333f, 0.0f, 0.0f, 0.3f};
bool inPlaybackScreen = false;
String playbackFile = "";

// SD playback overlay
void drawPlaybackScreen() {
  tft.fillRect(0, 20, 128, 108, BLACK);
  tft.setTextColor(GREEN); tft.setCursor(4, 22);
  tft.print("Playing: "); tft.print(playbackFile);


  tft.setTextColor(YELLOW); tft.setCursor(4, 110);
  tft.print("Menu Up/Down to exit");
}

// Scan SD path into menuItems
void scanSDMenu(const String& path) {
  if (!sdfsReady) {
    strcpy(menuItems[0], "<SD not ready>");
    menuLength = 1;
    menuIndex = 0;
    return;
  }
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

// SD browser with preview
void drawMenu() {
  tft.fillRect(0, 20, 128, 108, BLACK);
  const int visibleRows = 6;
  int y = 24;
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
  if (!sdfsReady) {
    tft.setTextColor(RED); tft.setCursor(4, 100);
    tft.print("SD not ready");
    return;
  }
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

void drawHeader(){
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  tft.fillScreen(BLACK);
  SPI.endTransaction();
}
void drawStatus(Status s){
  if (s == lastDrawn) return;
  lastDrawn = s;
  tft.fillRect(0,0,128,18,BLACK); tft.setCursor(4,2);
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
void drawFX(float volA, float volB, float masterVol){
  tft.fillRect(0,24,128,56,BLACK);

  tft.setCursor(4,26);
  tft.setTextColor(CYAN);
  tft.print("Vol: "); tft.print(volA,2);
  tft.print("  Vol: "); tft.print(volB,2);

  tft.setCursor(4,38);
  tft.setTextColor(YELLOW);
  tft.print("Rev "); tft.print(reverbDepth,2);
  tft.print("  LP "); tft.print(lowpassDepth,2);

  tft.setCursor(4,50);
  tft.setTextColor(CYAN);
  tft.print("Trem: "); tft.print(tremoloDepth,2);

  tft.setCursor(4,62);
  tft.print("Tape: "); tft.print(tapeSpeedRate,2); tft.print("x");

  tft.setCursor(4,74);
  tft.setTextColor(YELLOW);
  tft.print("Master: "); tft.print(masterVol,2);
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

// Loop waveform preview
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

// Fade ends to kill clicks
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

// Smooth loop wrap
void smoothLoopWrap() {
  if (blocksRecorded <= 1) return;
  const int CROSSFADE_SAMPLES = min(32, BLOCK_SAMPLES);
  const int firstBlockIndex = 0;
  const int lastBlockIndex = blocksRecorded - 1;
  const int fadeCount = CROSSFADE_SAMPLES;

  for (int sample = 0; sample < fadeCount; ++sample) {
    float mixT = (float)sample / (float)(fadeCount - 1);
    float gainFirst = mixT;
    float gainLast  = 1.0f - mixT;

    int lastSampleIndex = BLOCK_SAMPLES - fadeCount + sample;

    int32_t lastVal  = (int32_t)loopBuffer[lastBlockIndex][lastSampleIndex];
    int32_t firstVal = (int32_t)loopBuffer[firstBlockIndex][sample];

    int32_t mixed = (int32_t)(lastVal * gainLast + firstVal * gainFirst);

    if (mixed > 32767) mixed = 32767;
    else if (mixed < -32768) mixed = -32768;

    loopBuffer[lastBlockIndex][lastSampleIndex] = (int16_t)mixed;
  }
}

// Arm recording (RAM/SD)
void startArmedRecord(){
  isPlaying = false;
  isRecording = false;
  armedRecord = true;
  recordQueue.clear();
  if (!isSDRecording) {
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

// Start SD WAV record
void startSDRecord(const char* path) {
  if (!isSDRecording) return;
  if (!sdfsReady) {
    return;
  }
  wavFile = sdfs.open(path, O_WRITE | O_CREAT | O_TRUNC);
  if (wavFile) {
    Serial.print("startSDRecord: opened "); Serial.println(path);
    int written = writeWavHeader(wavFile, 44100, 16, 1);
    if (written != 44) {
      Serial.println("startSDRecord: header write failed");
      wavFile.close();
    } else {
      samplesWritten = 0;
    }
  } else {
    Serial.print("startSDRecord: could not open "); Serial.println(path);
  }
}

// Finish SD record
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
  mix.gain(0, 1.0f);
}

// Stop recording, finalize clip state
void stopRecording() {
  if (!(isRecording || armedRecord)) return;
  recordQueue.end();
  stopSDRecord();
  isRecording = false;
  armedRecord = false;
  mix.gain(0, 1.0f);
  bool hadAudio = false;
  if (recordSlot == CLIP_A) {
    hadAudio = (clipSamplesA > 0);
    clipReadyA = hadAudio;
    blocksRecorded = clipSamplesA / BLOCK_SAMPLES;
  } else if (recordSlot == CLIP_B) {
    hadAudio = (clipSamplesB > 0);
    clipReadyB = hadAudio;
    blocksRecorded = clipSamplesB / BLOCK_SAMPLES;
  } else {
    hadAudio = (blocksRecorded > 0);
  }
  recordSlot = CLIP_NONE;
  current = hadAudio ? ST_RECORDED : ST_NOAUDIO;
  drawStatus(current);
}

// Play RAM loop
void startPlayback(){
  if (blocksRecorded <= 0) { current = ST_NOAUDIO; drawStatus(current); return; }
  loopPlayIndex = 0;
  isPlaying = true;
  current = ST_PLAYING; drawStatus(current);
  mix.gain(0, 0.0f);
  if (tapBpm > 0.0f) showBpm();
}

// Stop playback, reset mix
void stopPlayback(){
  isPlaying = false;
  pausedPlayback = false;
  stopSDPlay();
  sdPolyPlaying = false;
  pendingPolyStart = false;
  current = ST_STOPPED; drawStatus(current);
  mix.gain(0, 1.0f);
  mix.gain(1, 0.8f);
  mix.gain(2, 0.0f);
}

void handleSDPlaybackLoop() {
  if (playFromSD && sdPlaying && playFile) {
    if (playFile.available() == 0) {
      playFile.seek(44);
    }
  }
}

// Poly SD play (2 files)
void startSDPolyPlay(const char* path1, const char* path2) {
  if (!sdfsReady) { Serial.println("startSDPolyPlay: sdfs not ready"); return; }
  stopSDPlay();
  playSd1.stop();
  playSd2.stop();
  sdPolyPlaying = true;
  polyMix.gain(0, 1.0f);
  polyMix.gain(1, 1.0f);
  polyMix.gain(2, 0.0f);
  polyMix.gain(3, 0.0f);
  mix.gain(0, overdubEnabled ? 1.0f : 0.0f);
  mix.gain(1, 0.0f);
  mix.gain(2, 1.0f);
  delay(10);
  Serial.print("startSDPolyPlay: "); Serial.print(path1); Serial.print(" , "); Serial.println(path2);
  playSd1.play(path1);
  playSd2.play(path2);
  delay(20);
  // Force gains in case anything reset post-start
  polyMix.gain(0, 1.0f);
  polyMix.gain(1, 1.0f);
  mix.gain(2, 1.0f);
  Serial.print("playSd1.isPlaying="); Serial.println(playSd1.isPlaying());
  Serial.print("playSd2.isPlaying="); Serial.println(playSd2.isPlaying());
}

void stopSDPolyPlay() {
  if (!sdPolyPlaying) return;
  playSd1.stop();
  playSd2.stop();
  sdPolyPlaying = false;
  pendingPolyStart = false;
  mix.gain(0, 1.0f);
  mix.gain(1, 0.8f);
  mix.gain(2, 0.0f);
}

// SD play (single file)
void startSDPlay(const char* path) {
  if (!sdfsReady) {
    Serial.println("SD: not ready");
    sdPlaying = false; playFromSD = false;
    return;
  }
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
  sdBlocksBase = sdBlocksToPlay;
  sdPosFrac = 0.0f;

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
  sdPosFrac = 0.0f;
  bufAReady = bufBReady = false;
  rdIndexA = rdIndexB = validA = validB = 0;
  flushPlaybackQueue();
  mix.gain(0, 1.0f);
  mix.gain(1, 0.8f);
  mix.gain(2, 0.0f);
}

// Clear loop state
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
  tft.fillRect(0, 96, 128, 32, BLACK);
}


int writeWavHeader(FsFile &f, uint32_t sampleRate, uint16_t bitsPerSample, uint16_t channels) {
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

  return f.write(hdr, 44);
}

void patchWavSizes(FsFile &f, uint32_t dataBytes) {

  uint32_t riffSize = 36 + dataBytes;
  f.seek(4);  f.write((uint8_t*)&riffSize, 4);
  f.seek(40); f.write((uint8_t*)&dataBytes, 4);
}

// Bounce RAM clip to SD with FX
bool startBounceSave(ClipSlot slot) {
  if (!sdfsReady) { writeFooter("SD not ready", RED); return false; }
  if (bounceActive) return false;

  const bool targetA = (slot == CLIP_A && clipReadyA);
  const bool targetB = (slot == CLIP_B && clipReadyB);
  if (!targetA && !targetB) { writeFooter("No clip", RED); return false; }

  const char* prefix = targetA ? "/CLIPA" : "/CLIPB";
  char fname[20];
  bool found = false;
  for (int i = 1; i < 1000; ++i) {
    snprintf(fname, sizeof(fname), "%s%03d.WAV", prefix, i);
    if (!sdfs.exists(fname)) { found = true; break; }
  }
  if (!found) { writeFooter("No file slot", RED); return false; }

  bounceFile = sdfs.open(fname, O_WRITE | O_CREAT | O_TRUNC);
  if (!bounceFile) { writeFooter("Open fail", RED); return false; }

  int written = writeWavHeader(bounceFile, 44100, 16, 1);
  if (written != 44) { writeFooter("Header fail", RED); bounceFile.close(); return false; }
  bounceSamples = 0;
  bounceSlot = slot;
  bounceActive = true;

  stopSDPlay();
  isRecording = false;
  armedRecord = false;
  playFromSD = false;
  pausedPlayback = false;
  mix.gain(0, 0.0f);
  mix.gain(1, 1.0f);
  mix.gain(2, 0.0f);

  playingClipA = targetA;
  playingClipB = targetB;
  clipPlayPosA = 0.0f;
  clipPlayPosB = 0.0f;
  isPlaying = (playingClipA || playingClipB);
  current = isPlaying ? ST_PLAYING : current;
  drawStatus(current);

  bounceQ.begin();

  tft.fillRect(0, 120, 128, 8, BLACK);
  tft.setCursor(4, 120);
  tft.setTextColor(YELLOW);
  tft.print("Bounce -> ");
  tft.print(fname);
  return true;
}

// Refill SD buffers
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


// Queue SD buffers into playback
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

    float step = 1.0f;
    float basePos = (float)(*idx);
    for (int i = 0; i < BLOCK_SAMPLES; ++i) {
      float pos = basePos + step * (float)i;
      int16_t out = 0;
      if (pos + 1.0f < (float)(*valid)) {
        uint32_t base = (uint32_t)pos;
        float frac = pos - (float)base;
        int32_t a = src[base];
        int32_t b = src[base + 1];
        int32_t s = (int32_t)((1.0f - frac) * a + frac * b);
        out = (int16_t)s;
      }
      float depth = tremoloDepth;
      if (depth > 0.0f) {
        float lfo = 0.5f + 0.5f * sinf(tremoloPhaseSD);
        float gain = 1.0f - depth + depth * lfo;
        int32_t s = (int32_t)(out * gain);
        if (s > 32767) s = 32767;
        else if (s < -32768) s = -32768;
        out = (int16_t)s;
        tremoloPhaseSD += 2.0f * 3.14159265f * (tremoloRate / 44100.0f);
        if (tremoloPhaseSD > 6.2831853f) tremoloPhaseSD -= 6.2831853f;
      }
      destBuf[i] = out;
    }
    *idx += BLOCK_SAMPLES;
    sdPosFrac = 0.0f;
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


// Init hardware/UI
void setup(){
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);
  lastEncState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);

  encoderBtn.onHold([](int){
    if (uiMode == MODE_RAM) {
      overdubEnabled = !overdubEnabled;
      tft.fillRect(0, 96, 128, 12, BLACK);
      tft.setCursor(4, 98);
      tft.setTextColor(overdubEnabled ? CYAN : RED);
      tft.print("Overdub: "); tft.print(overdubEnabled ? "ON " : "OFF");
    }
  }, 800);

  encoderBtn.onPress([](int){
    if (uiMode == MODE_RAM) {
      fxTarget = (FxTarget)(((int)fxTarget + 1) % FX_TGT_COUNT);
      tft.fillRect(0, 96, 128, 12, BLACK);
      tft.setCursor(4, 98);
      tft.setTextColor(CYAN);
      switch (fxTarget) {
        case FX_TGT_BOTH: tft.print("FX Target: Both"); break;
        case FX_TGT_A:    tft.print("FX Target: Clip A"); break;
        case FX_TGT_B:    tft.print("FX Target: Clip B"); break;
        default: break;
      }
      return;
    }
    if (inPlaybackScreen) {
      return;
    }
    if (uiMode == MODE_MENU) {
      if (!sdfsReady) {
        tft.fillRect(0, 120, 128, 8, BLACK);
        tft.setCursor(4, 120);
        tft.setTextColor(RED);
        tft.print("SD not ready");
        return;
      }
      String selName = String(menuItems[menuIndex]);
      bool isDir = selName.endsWith("/");
      if (sdPolyMode) {
        if (polySelectionStage < 2) {
          if (isDir) return;
          String fullPath = currentPath + selName;
          if (polySelectionStage == 0) {
            polyPath1 = fullPath;
            polySelectionStage = 1;
            writeFooter("Track 1: " + selName, CYAN);
          } else {
            polyPath2 = fullPath;
            polySelectionStage = 2;
            tapCount = 0;
            tapBpm = 0.0f;
            tapIntervalCount = 0;
            tapIntervalIndex = 0;
            writeFooter("Tap to set BPM", YELLOW);
          }
        } else {
          bool bpmReady = registerTapTempo();
          if (polyPath1.length() && polyPath2.length()) {
            if (bpmReady && msPerBeat > 0) {
              uint32_t now = millis();
              uint32_t mod = msPerBeat ? (now % msPerBeat) : 0;
              polyStartAtMs = now + (msPerBeat ? (msPerBeat - mod) : 0);
              pendingPolyStart = true;
              writeFooter("Waiting for beat", CYAN);
            } else if (!pendingPolyStart && msPerBeat == 0) {
              startSDPolyPlay(polyPath1.c_str(), polyPath2.c_str());
              current = ST_PLAYING;
              drawStatus(current);
              inPlaybackScreen = true;
              playbackFile = polyPath1 + " + " + polyPath2;
              drawPlaybackScreen();
              polySelectionStage = 0;
            }
          }
        }
      } else {
        if (!isDir) {
          String fullPath1 = currentPath + selName;
          stopSDPolyPlay();
          stopSDPlay();
          flushPlaybackQueue();
          startSDPlay(fullPath1.c_str());
          current = ST_PLAYING;
          drawStatus(current);
        }
      }
    }
  });
  sampleBtn1.onPress([](int){
    if (uiMode == MODE_MENU && inPlaybackScreen) {
      inPlaybackScreen = false;
      drawMenu();
      return;
    }
    if (uiMode != MODE_RAM) return;

  if (!overdubEnabled && !isRecording && !armedRecord && clipReadyA) {
      if (tapBpm > 0.0f && msPerBeat > 0) {
        uint32_t now = millis();
        uint32_t mod = now % msPerBeat;
        nextBeatMs   = now + (msPerBeat - mod);
        pendingClipA = true;
        pendingClipB = false;
        isPlaying    = false;
        tft.fillRect(0, 120, 128, 8, BLACK);
        tft.setCursor(4, 120);
        tft.setTextColor(CYAN);
        tft.print("Pad A: wait beat");
      } else {
        playingClipA = true;
        playingClipB = false;
        clipPlayPosA = 0;
        clipPlayPosB = 0;
        isPlaying    = true;
        current      = ST_PLAYING;
        drawStatus(current);
      }
      return;
    }

    if (isRecording || armedRecord) stopRecording();
    recordSlot    = CLIP_A;
    if (overdubEnabled && clipReadyA) {
      overdubPosA = 0;
    } else {
      clipSamplesA  = 0;
      clipReadyA    = false;
      overdubPosA   = 0;
    }
    startArmedRecord();
    tft.fillRect(0, 96, 128, 12, BLACK);
    tft.setCursor(4, 98); tft.setTextColor(CYAN);
    tft.print("Rec Smp 1");
  });
  sampleBtn1.onHold([](int){
    if (uiMode != MODE_RAM) return;
    if (isRecording || armedRecord) stopRecording();
    if (isPlaying) stopPlayback();
    startBounceSave(CLIP_A);
  }, 500);
  sampleBtn2.onPress([](int){
    if (uiMode != MODE_RAM) return;
    if (!overdubEnabled && !isRecording && !armedRecord && clipReadyB) {
      if (tapBpm > 0.0f && msPerBeat > 0) {
        uint32_t now = millis();
        uint32_t mod = now % msPerBeat;
        nextBeatMs   = now + (msPerBeat - mod);
        pendingClipA = false;
        pendingClipB = true;
        isPlaying    = false;
        tft.fillRect(0, 120, 128, 8, BLACK);
        tft.setCursor(4, 120);
        tft.setTextColor(CYAN);
        tft.print("Pad B: wait beat");
      } else {
        playingClipA = false;
        playingClipB = true;
        clipPlayPosA = 0;
        clipPlayPosB = 0;
        isPlaying    = true;
        current      = ST_PLAYING;
        drawStatus(current);
      }
      return;
    }

    if (isRecording || armedRecord) stopRecording();
    recordSlot    = CLIP_B;
    if (overdubEnabled && clipReadyB) {
      overdubPosB = 0;
    } else {
      clipSamplesB  = 0;
      clipReadyB    = false;
      overdubPosB   = 0;
    }
    startArmedRecord();
    tft.fillRect(0, 96, 128, 12, BLACK);
    tft.setCursor(4, 98); tft.setTextColor(CYAN);
    tft.print("Rec Smp 2");
  });
  sampleBtn2.onHold([](int){
    if (uiMode != MODE_RAM) return;
    if (isRecording || armedRecord) stopRecording();
    if (isPlaying) stopPlayback();
    startBounceSave(CLIP_B);
  }, 500);

  recButton.onPress([](int){
    if (uiMode == MODE_RAM) {
      stopRecording();
      stopPlayback();
      pausedPlayback = false;
      pendingClipA = pendingClipB = false;
      clipSamplesA = clipSamplesB = 0;
      clipReadyA = clipReadyB = false;
      clipPlayPosA = clipPlayPosB = 0;
      playingClipA = playingClipB = false;
      blocksRecorded = 0;
      loopPlayIndex = 0;
      current = ST_READY; drawStatus(current);
      drawCounts(0);
      tft.fillRect(0, 96, 128, 32, BLACK);
      return;
    } else if (uiMode == MODE_MENU) {
      sdPolyMode = !sdPolyMode;
      resetPolyPrep();
      tft.fillRect(0, 120, 128, 8, BLACK);
      tft.setCursor(4, 120);
      tft.setTextColor(CYAN);
      tft.print(sdPolyMode ? "Poly SDPlay" : "Mono SDPlay");
      return;
    }
  });

  playButton.onPress([](int){
    if (uiMode == MODE_MENU) {
      stopSDPolyPlay();
      stopSDPlay();
      inPlaybackScreen = false;
      tft.fillRect(0, 120, 128, 8, BLACK);
      tft.setCursor(4, 120);
      tft.setTextColor(CYAN);
      tft.print("SD Play: Stopped");
      return;
    }
    if (uiMode != MODE_RAM) return;
    if (isPlaying) {
      isPlaying = false;
      pausedPlayback = true;
      current = ST_STOPPED; drawStatus(current);
      return;
    }
    if (isRecording || armedRecord) stopRecording();
    if (pausedPlayback && (clipReadyA || clipReadyB)) {
      if (clipPlayPosA >= clipSamplesA) playingClipA = false;
      if (clipPlayPosB >= clipSamplesB) playingClipB = false;
      playingClipA = playingClipA || (clipReadyA && clipPlayPosA < clipSamplesA);
      playingClipB = playingClipB || (clipReadyB && clipPlayPosB < clipSamplesB);
      pausedPlayback = false;
      if (playingClipA || playingClipB) {
        isPlaying = true;
        current = ST_PLAYING; drawStatus(current);
      }
      return;
    }
    playingClipA = clipReadyA;
    playingClipB = clipReadyB;
    clipPlayPosA = 0;
    clipPlayPosB = 0;
    if (!(playingClipA || playingClipB)) return;
    if (tapBpm > 0.0f && msPerBeat > 0) {
      uint32_t now = millis();
      uint32_t mod = (now % msPerBeat);
      nextBeatMs = now + (msPerBeat - mod);
      pendingClipA = playingClipA;
      pendingClipB = playingClipB;
      isPlaying = false;
      tft.fillRect(0, 120, 128, 8, BLACK);
      tft.setCursor(4, 120);
      tft.setTextColor(CYAN);
      tft.print("Waiting for beat");
    } else {
      isPlaying = true;
      current = ST_PLAYING; drawStatus(current);
    }
  });

  playButton.onHold([](int){
    if (uiMode != MODE_RAM) return;
    if (isRecording || armedRecord) stopRecording();
    if (isPlaying) stopPlayback();
  }, 800);

  loopButton.onPress([](int){
    if (uiMode == MODE_RAM) {
      bool bpmReady = registerTapTempo();
      if (bpmReady || tapBpm > 0.0f) showBpm();
      return;
    }
    if (uiMode == MODE_MENU) {
      if (!sdfsReady) return;
      bool bpmReady = registerTapTempo();
      if (sdPolyMode) {
        if (bpmReady && msPerBeat > 0 && polyPath1.length() && polyPath2.length()) {
          uint32_t now = millis();
          uint32_t mod = msPerBeat ? (now % msPerBeat) : 0;
          polyStartAtMs = now + (msPerBeat ? (msPerBeat - mod) : 0);
          pendingPolyStart = true;
          writeFooter("Waiting for beat", CYAN);
        }
      } else if (tapBpm > 0.0f) {
        showBpm();
      }
    }
  });

  overdubButton.onPress([](int){
    if (uiMode == MODE_RAM) {
      overdubEnabled = !overdubEnabled;
      tft.fillRect(0, 96, 128, 12, BLACK);
      tft.setCursor(4, 98);
      tft.setTextColor(overdubEnabled ? CYAN : RED);
      tft.print("Overdub: "); tft.print(overdubEnabled ? "ON " : "OFF");
    } else if (uiMode == MODE_MENU) {
      if (!sdfsReady) return;
      String selName = String(menuItems[menuIndex]);
      if (selName.endsWith("/")) return;
      String fullPath = currentPath + selName;
      if (sdfs.exists(fullPath.c_str())) {
        bool removed = sdfs.remove(fullPath.c_str());
        tft.fillRect(0, 120, 128, 8, BLACK);
        tft.setCursor(4, 120);
        tft.setTextColor(removed ? RED : YELLOW);
        tft.print(removed ? "Deleted " : "Delete fail ");
        tft.print(selName);
        scanSDMenu(currentPath);
        drawMenu();
      }
    }
  });

  selectButton.onPress([](int){
    uiMode = static_cast<UIMode>((uiMode + 1) % 3);
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
    resetPolyPrep();
    inPlaybackScreen = false;
    isSDRecording = (uiMode == MODE_SDREC);
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
    } else if (uiMode == MODE_MENU) {
      tft.setTextColor(CYAN); tft.print("MENU MODE   ");
      scanSDMenu(currentPath);
      drawMenu();
    }
  });
  selectButton.onHold([](int){
    useUsbInput = !useUsbInput;
    tft.fillRect(0, 120, 128, 8, BLACK);
    tft.setCursor(4, 120);
    tft.setTextColor(CYAN);
    tft.print("Input: ");
    tft.print(useUsbInput ? "USB" : "AUX");
  }, 600);
  Serial.begin(115200);
  while (!Serial && millis() < 2000) ;
  SPI.setMOSI(PCB_SPI_MOSI);
  SPI.setSCK(PCB_SPI_SCK);
  SPI.begin();
  tft.begin();
  drawHeader(); drawStatus(ST_READY); drawFX(clipGainA, clipGainB, readPotInv(POT_VOL)); drawCounts(0);

  AudioMemory(320);
  sgtl5000.enable();
  sgtl5000.inputSelect(AUDIO_INPUT_LINEIN);
  sgtl5000.lineOutLevel(6);
  sgtl5000.lineInLevel(15);
  sgtl5000.volume(0.60f);

  hp.setHighpass(0, 20.0f, 0.707f);
  mix.gain(0, 1.0f);
  mix.gain(1, 0.8f);
  mix.gain(2, 0.0f);
  outMix.gain(0, 1.0f);
  outMix.gain(1, reverbDepth);
  reverb.roomsize(0.25f);
  reverb.damping(0.50f);

  inputMix.gain(0, 0.0f);
  inputMix.gain(1, 1.0f);

  pinMode(CLIP_LED_PIN, OUTPUT);
  digitalWrite(CLIP_LED_PIN, LOW);

if (!sdfs.begin(SdioConfig(FIFO_SDIO))) {
  tft.setCursor(4, 80); tft.setTextColor(RED); tft.println("SDIO init failed");
} else {
  sdfsReady = true;
}

  drawMenu();
}


// Main loop (UI + audio)
void loop(){
  static elapsedMillis ramTimer = 0;
  if (ramTimer > 2000) {
    ramTimer = 0;
  }
  for (size_t i = 0; i < BUTTON_COUNT; ++i) buttonArray[i]->process();

  int encDelta = readEncoderDelta();
  if (encDelta != 0) {
    if (uiMode == MODE_MENU) {
      if (!inPlaybackScreen) {
        menuIndex += encDelta;
        if (menuIndex < 0) menuIndex = (menuLength > 0) ? (menuLength - 1) : 0;
        if (menuIndex >= menuLength) menuIndex = 0;
        drawMenu();
      }
    } else if (uiMode == MODE_SDREC) {
      sdRecMaxBlocks += encDelta * 500;
      if (sdRecMaxBlocks < 100) sdRecMaxBlocks = 100;
      if (sdRecMaxBlocks > 4000) sdRecMaxBlocks = 4000;
      tft.fillRect(0, 108, 128, 12, BLACK);
      tft.setCursor(4, 108);
      tft.setTextColor(CYAN);
      tft.print("SD Blocks: ");
      tft.print(sdRecMaxBlocks);
    } else if (uiMode == MODE_RAM) {
    }
  }

  if (pendingClipA || pendingClipB) {
    if (tapBpm > 0.0f && msPerBeat > 0 && millis() >= nextBeatMs) {
      playingClipA = pendingClipA;
      playingClipB = pendingClipB;
      pendingClipA = pendingClipB = false;
      clipPlayPosA = 0;
      clipPlayPosB = 0;
      isPlaying = (playingClipA || playingClipB);
      current = isPlaying ? ST_PLAYING : current;
      drawStatus(current);
      showBpm();
    }
  }

  if (useUsbInput) {
    inputMix.gain(0, 1.0f);
    inputMix.gain(1, 0.0f);
  } else {
    inputMix.gain(0, 0.0f);
    inputMix.gain(1, 1.0f);
    sgtl5000.inputSelect(AUDIO_INPUT_LINEIN);
  }

  if (playFromSD && sdPlaying) {
    refillIfNeeded();
  int slots = playbackQueue.available();
    int toQueue = min(slots, 8);
    while (toQueue-- > 0 && slots-- > 0) {
      queueFromBuffer();
    }
    mix.gain(0, overdubEnabled ? 1.0f : 0.0f);
    mix.gain(1, 1.0f);
    mix.gain(2, 0.0f);
  }

  if (pendingPolyStart && msPerBeat > 0 && polyPath1.length() && polyPath2.length()) {
    uint32_t now = millis();
    if (now >= polyStartAtMs) {
      pendingPolyStart = false;
      startSDPolyPlay(polyPath1.c_str(), polyPath2.c_str());
      current = ST_PLAYING;
      drawStatus(current);
      inPlaybackScreen = true;
      playbackFile = polyPath1 + " + " + polyPath2;
      drawPlaybackScreen();
      polySelectionStage = 0;
    }
  }
  if (pendingPolyStart && msPerBeat == 0 && polyPath1.length() && polyPath2.length()) {
    pendingPolyStart = false;
    startSDPolyPlay(polyPath1.c_str(), polyPath2.c_str());
    current = ST_PLAYING;
    drawStatus(current);
    inPlaybackScreen = true;
    playbackFile = polyPath1 + " + " + polyPath2;
    drawPlaybackScreen();
    polySelectionStage = 0;
  }

  if (uiMode == MODE_MENU) {
    if (inPlaybackScreen) {
      static uint32_t lastPlayDraw = 0;
      static String lastPlayFile = "";
      if (playbackFile != lastPlayFile || millis() - lastPlayDraw > 1000) {
        drawPlaybackScreen();
        lastPlayDraw = millis();
        lastPlayFile = playbackFile;
      }
    }
    if (!((playFromSD && sdPlaying) || sdPolyPlaying)) return;
  }
  if (sdPolyPlaying) {
    // Keep SD poly routed even if other states tweak the mixer
    polyMix.gain(0, 1.0f);
    polyMix.gain(1, 1.0f);
    mix.gain(0, overdubEnabled ? 1.0f : 0.0f);
    mix.gain(1, 0.0f);
    mix.gain(2, 1.0f);
  }
  float clipVolA = readPotInv(PIN_FX1_POT);
  float clipVolB = readPotInv(PIN_FX2_POT);
  float fxReverbVal   = readPotInv(PIN_SMPVOL1_POT);
  float fxLowpassVal  = readPotInv(PIN_SMPVOL2_POT);
  float fxTremoloVal  = readPotInv(PIN_SMPVOL3_POT);
  float fxTapeVal     = readPotInv(PIN_SMPVOL4_POT);
  float masterVolume  = readPotInv(POT_VOL);

  clipGainA = clipVolA;
  clipGainB = clipVolB;

  static float lastRev = -1.0f, lastLp = -1.0f, lastTrem = -1.0f, lastTape = -1.0f;
  bool revMoved = false, lpMoved = false, tremMoved = false, tapeMoved = false;
  if (lastRev < 0.0f)  { lastRev  = fxReverbVal;  revMoved  = true; }
  if (lastLp  < 0.0f)  { lastLp   = fxLowpassVal; lpMoved   = true; }
  if (lastTrem< 0.0f)  { lastTrem = fxTremoloVal; tremMoved = true; }
  if (lastTape< 0.0f)  { lastTape = fxTapeVal;    tapeMoved = true; }
  if (fabs(fxReverbVal  - lastRev)  > 0.01f) { lastRev  = fxReverbVal;  revMoved  = true; }
  if (fabs(fxLowpassVal - lastLp)   > 0.01f) { lastLp   = fxLowpassVal; lpMoved   = true; }
  if (fabs(fxTremoloVal - lastTrem) > 0.01f) { lastTrem = fxTremoloVal; tremMoved = true; }
  if (fabs(fxTapeVal    - lastTape) > 0.01f) { lastTape = fxTapeVal;    tapeMoved = true; }
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
    if (revMoved) {
      reverbDepth = fxReverbVal;
      reverb.roomsize(reverbDepth > 0.01f ? (reverbDepth * 0.95f) : 0.01f);
    }
    if (lpMoved) {
      lowpassDepth = fxLowpassVal;
      float minCutoff = 500.0f;
      float maxCutoff = 12000.0f;
      float cutoff = maxCutoff - lowpassDepth * (maxCutoff - minCutoff);
      if (lowpassDepth < 0.01f) cutoff = maxCutoff;
      lowpassFX.setLowpass(0, cutoff, 0.707f);
    }
    if (tremMoved) {
      tremoloDepth = fxTremoloVal;
    }
    if (tapeMoved) {
      tapeSpeedRate = 0.5f + fxTapeVal * 1.5f;
    }
  } else if (uiMode == MODE_SDREC) {
    if (!isRecording && !armedRecord && !isPlaying && !sdPolyPlaying) {
      mix.gain(0, 1.0f);
      mix.gain(1, 0.8f);
      mix.gain(2, 0.0f);
    }
  }
  if (playFromSD && sdPlaying) {
    reverbDepth = fxReverbVal;
    reverb.roomsize(reverbDepth > 0.01f ? (reverbDepth * 0.95f) : 0.01f);
    lowpassDepth = fxLowpassVal;
    float minCutoff = 500.0f;
    float maxCutoff = 12000.0f;
    float cutoff = maxCutoff - lowpassDepth * (maxCutoff - minCutoff);
    if (lowpassDepth < 0.01f) cutoff = maxCutoff;
    lowpassFX.setLowpass(0, cutoff, 0.707f);
  }
  sgtl5000.volume(constrain(masterVolume, 0.0f, 1.0f));
  if (!isSDRecording) {
    static elapsedMillis uiTick = 0;
    if (uiTick > 150){
      if (uiMode == MODE_RAM) {
        drawFX(clipGainA, clipGainB, masterVolume);
      }
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
      if (countsDirty) {
        drawCounts(blocksRecorded);
        countsDirty = false;
      }
      uiTick = 0;
    }
  }

  if (armedRecord || isRecording){
    while (recordQueue.available()){
      const int16_t* b = (const int16_t*)recordQueue.readBuffer();
      if (armedRecord) {
        float lvl = peakL.available() ? peakL.read() : 0.0f;
        if (lvl >= ARM_LEVEL_THRESHOLD) {
          startRecordingNow();
        }
        recordQueue.freeBuffer();
        continue;
      }

      if (isRecording) {
        if (recordSlot == CLIP_A) {
          bool overdubActive = overdubEnabled && clipReadyA;
          uint32_t writePos = overdubActive ? overdubPosA : clipSamplesA;
          uint32_t existingLen = clipSamplesA;
          uint32_t remaining = (writePos + BLOCK_SAMPLES <= MAX_CLIP_SAMPLES) ? BLOCK_SAMPLES : (MAX_CLIP_SAMPLES - writePos);
          if (remaining > 0) {
            for (uint32_t i = 0; i < remaining; ++i) {
              uint32_t idx = writePos + i;
              int32_t in = b[i];
              int32_t existing = (overdubActive && idx < existingLen) ? clipA[idx] : 0;
              int32_t mixed = overdubActive ? (int32_t)(existing * odFeedback + in * odInputGain) : in;
              if (mixed > 32767) mixed = 32767;
              else if (mixed < -32768) mixed = -32768;
              clipA[idx] = (int16_t)mixed;
            }
            writePos += remaining;
            clipSamplesA = overdubActive ? max(existingLen, writePos) : (clipSamplesA + remaining);
            overdubPosA = overdubActive ? writePos : overdubPosA;
          }
          if (clipSamplesA >= MAX_CLIP_SAMPLES) { stopRecording(); }
        } else if (recordSlot == CLIP_B) {
          bool overdubActive = overdubEnabled && clipReadyB;
          uint32_t writePos = overdubActive ? overdubPosB : clipSamplesB;
          uint32_t existingLen = clipSamplesB;
          uint32_t remaining = (writePos + BLOCK_SAMPLES <= MAX_CLIP_SAMPLES) ? BLOCK_SAMPLES : (MAX_CLIP_SAMPLES - writePos);
          if (remaining > 0) {
            for (uint32_t i = 0; i < remaining; ++i) {
              uint32_t idx = writePos + i;
              int32_t in = b[i];
              int32_t existing = (overdubActive && idx < existingLen) ? clipB[idx] : 0;
              int32_t mixed = overdubActive ? (int32_t)(existing * odFeedback + in * odInputGain) : in;
              if (mixed > 32767) mixed = 32767;
              else if (mixed < -32768) mixed = -32768;
              clipB[idx] = (int16_t)mixed;
            }
            writePos += remaining;
            clipSamplesB = overdubActive ? max(existingLen, writePos) : (clipSamplesB + remaining);
            overdubPosB = overdubActive ? writePos : overdubPosB;
          }
          if (clipSamplesB >= MAX_CLIP_SAMPLES) { stopRecording(); }
        }
        recordQueue.freeBuffer();
      }
    }
  }

  if (isPlaying && !playFromSD) {
    int slots = playQ.available();
    int toQueue = min(slots, 6);
    while (toQueue-- > 0 && slots-- > 0) {
      int16_t *outBuffer = playbackQueue.getBuffer();
      if (!outBuffer) break;
      memset(outBuffer, 0, BLOCK_SAMPLES * sizeof(int16_t));

      float speedA = tapeSpeedRate;
      float speedB = tapeSpeedRate;
      float depthA = (fxTarget == FX_TGT_BOTH || fxTarget == FX_TGT_A) ? tremoloDepth : 0.0f;
      float depthB = (fxTarget == FX_TGT_BOTH || fxTarget == FX_TGT_B) ? tremoloDepth : 0.0f;
      float phaseA = tremoloPhaseA;
      float phaseB = tremoloPhaseB;
      bool anySample = false;

      for (int i = 0; i < BLOCK_SAMPLES; ++i) {
        float mixedSample = 0.0f;

        if (playingClipA) {
          if (loopModeEnabled && !bounceActive && clipReadyA && clipSamplesA > 0 && clipPlayPosA >= clipSamplesA) {
            clipPlayPosA -= clipSamplesA;
          }
          if (clipPlayPosA < clipSamplesA) {
            uint32_t idx = (uint32_t)clipPlayPosA;
            uint32_t nextIdx = (idx + 1 < clipSamplesA) ? (idx + 1) : idx;
            float frac = clipPlayPosA - (float)idx;
            float s0 = (float)clipA[idx];
            float s1 = (float)clipA[nextIdx];
            float sample = s0 + (s1 - s0) * frac;
            if (depthA > 0.0f) {
              float lfo = 0.5f + 0.5f * sinf(phaseA);
              sample *= (1.0f - depthA + depthA * lfo);
              phaseA += 2.0f * 3.14159265f * (tremoloRate / 44100.0f);
              if (phaseA > 6.2831853f) phaseA -= 6.2831853f;
            }
            mixedSample += sample * clipGainA;
            clipPlayPosA += speedA;
          } else {
            playingClipA = false;
          }
        }

        if (playingClipB) {
          if (loopModeEnabled && !bounceActive && clipReadyB && clipSamplesB > 0 && clipPlayPosB >= clipSamplesB) {
            clipPlayPosB -= clipSamplesB;
          }
          if (clipPlayPosB < clipSamplesB) {
            uint32_t idx = (uint32_t)clipPlayPosB;
            uint32_t nextIdx = (idx + 1 < clipSamplesB) ? (idx + 1) : idx;
            float frac = clipPlayPosB - (float)idx;
            float s0 = (float)clipB[idx];
            float s1 = (float)clipB[nextIdx];
            float sample = s0 + (s1 - s0) * frac;
            if (depthB > 0.0f) {
              float lfo = 0.5f + 0.5f * sinf(phaseB);
              sample *= (1.0f - depthB + depthB * lfo);
              phaseB += 2.0f * 3.14159265f * (tremoloRate / 44100.0f);
              if (phaseB > 6.2831853f) phaseB -= 6.2831853f;
            }
            mixedSample += sample * clipGainB;
            clipPlayPosB += speedB;
          } else {
            playingClipB = false;
          }
        }

        if (mixedSample != 0.0f) {
          if (mixedSample > 32767.0f) mixedSample = 32767.0f;
          else if (mixedSample < -32768.0f) mixedSample = -32768.0f;
          outBuffer[i] = (int16_t)mixedSample;
          anySample = true;
        } else {
          outBuffer[i] = 0;
        }
      }

      tremoloPhaseA = phaseA;
      tremoloPhaseB = phaseB;

      playbackQueue.playBuffer();

      if (!anySample) {
        if (bounceActive) {
          playingClipA = playingClipB = false;
          isPlaying = false;
          current = ST_STOPPED; drawStatus(current);
          break;
        } else if (loopModeEnabled && (clipReadyA || clipReadyB)) {
          clipPlayPosA = 0.0f;
          clipPlayPosB = 0.0f;
          playingClipA = clipReadyA;
          playingClipB = clipReadyB;
        } else {
          isPlaying = false;
          current = ST_STOPPED; drawStatus(current);
          break;
        }
      }
    }
  }

  if (playFromSD && sdPlaying) {
    refillIfNeeded();
  int slots = playbackQueue.available();
    int toQueue = min(slots, 8);
    while (toQueue-- > 0 && slots-- > 0) {
      queueFromBuffer();
    }
  }

  if (bounceActive) {
    while (bounceQ.available()) {
      const int16_t* b = (const int16_t*)bounceQ.readBuffer();
      bounceFile.write((uint8_t*)b, BLOCK_SAMPLES * sizeof(int16_t));
      bounceSamples += BLOCK_SAMPLES;
      bounceQ.freeBuffer();
    }
    if (!isPlaying && !playingClipA && !playingClipB && bounceQ.available() == 0) {
      bounceQ.end();
      patchWavSizes(bounceFile, bounceSamples * sizeof(int16_t));
      bounceFile.close();
      bounceActive = false;
      bounceSlot = CLIP_NONE;
      tft.fillRect(0, 120, 128, 8, BLACK);
      tft.setCursor(4, 120);
      tft.setTextColor(GREEN);
      tft.print("Bounce saved");
    }
  }
}
