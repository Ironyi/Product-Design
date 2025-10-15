#ifndef BETTER_BUTTON_H
#define BETTER_BUTTON_H

#include <Arduino.h>

class BetterButton {
public:
  // If your buttons are wired to GND, leave activeLow = true and use INPUT_PULLUP
  BetterButton(uint8_t pin, int id, bool activeLow = true, uint16_t debounceMs = 15);

  void process();                         // call every loop()
  void onPress(void (*f)(int));           // edge: idle -> active
  void onRelease(void (*f)(int));         // edge: active -> idle
  void onHold(void (*f)(int), uint16_t holdMs = 600); // hold callback
  bool isPressed() const;                 // current debounced state

  // public meta
  int   val;                              // user ID
  uint8_t buttonPin;

private:
  bool rawRead() const;

  // config
  bool activeLow;
  uint16_t debounceMs;

  // state
  bool curr;               // debounced state (true = active/pressed)
  bool prev;               // previous debounced
  bool lastRaw;            // last raw read
  unsigned long lastChangeMs;

  // hold detection
  void (*holdCb)(int) = nullptr;
  uint16_t holdMs = 600;
  unsigned long pressedTime = 0;
  bool holdFired = false;

  // callbacks
  void (*pressCb)(int)   = nullptr;
  void (*releaseCb)(int) = nullptr;
};

#endif
