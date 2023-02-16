#pragma once

#include "Arduino.h"

class KEM3361 
{
  public:
    void print(String);
    KEM3361(uint8_t, uint8_t, uint8_t);
    void tick();

  protected:
    char digits[3];
    bool dots[3];
    uint16_t registerToOut;
    uint8_t digitIndex;
    uint8_t pinClock;
    uint8_t pinData;
    uint8_t pinLatch;

    uint16_t encode(char, uint8_t, bool);
    void wordOut();
};