#include "BetterButton.h"

BetterButton::BetterButton(uint8_t pin, int id, bool activeLow_, uint16_t debounceMs_)
: val(id), buttonPin(pin), activeLow(activeLow_), debounceMs(debounceMs_) {
  // For activeLow buttons (wired to GND), use INPUT_PULLUP
  pinMode(buttonPin, activeLow ? INPUT_PULLUP : INPUT);

  // Initialize states
  lastRaw = rawRead();
  curr    = lastRaw;
  prev    = curr;
  lastChangeMs = millis();
}

bool BetterButton::rawRead() const {
  int r = digitalRead(buttonPin);
  // Map raw level to logical "active"
  return activeLow ? (r == LOW) : (r == HIGH);
}

void BetterButton::process() {
  bool r = rawRead();
  unsigned long now = millis();

  if (r != lastRaw) {
    lastChangeMs = now;       // raw bounced; reset timer
    lastRaw = r;
  }

  // adopt new debounced state after debounceMs stable
  if ((now - lastChangeMs) >= debounceMs && r != curr) {
    prev = curr;
    curr = r;

    // edge events
    if (curr && !prev) {          // idle -> active  => PRESS
      if (pressCb) pressCb(val);
      pressedTime = now;
      holdFired = false;
    } else if (!curr && prev) {   // active -> idle  => RELEASE
      if (releaseCb) releaseCb(val);
      pressedTime = 0;
      holdFired = false;
    }
  }

  // Hold detection
  if (curr && holdCb && !holdFired && pressedTime > 0 && (now - pressedTime >= holdMs)) {
    holdFired = true;
    holdCb(val);
  }
}

void BetterButton::onHold(void (*f)(int), uint16_t holdMs_) {
  holdCb = f;
  holdMs = holdMs_;
}

void BetterButton::onPress(void (*f)(int))   { pressCb   = f; }
void BetterButton::onRelease(void (*f)(int)) { releaseCb = f; }
bool BetterButton::isPressed() const         { return curr; }
