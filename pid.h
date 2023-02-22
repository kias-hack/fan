#pragma once

#include "Arduino.h"


class PID {
  private:
    double P;
    double I;
    double D;
    
    double minValue;
    double maxValue;
    
    double integral;
    
    double lastError;
    unsigned long lastTime;
  
    char sign(double);

  public:
    PID(double, double, double, double, double);
    double compute(double);
};

PID::PID(double P, double I, double D, double minValue, double maxValue) : P(P), I(I), D(D), minValue(minValue), maxValue(maxValue){}

char PID::sign(double v){
  if(v >= 0) return 1;
  return -1;
}

double PID::compute(double error) {
  double dt = (millis() - lastTime) / 1000.0;

  double valueP = error * P;

  double valueI = error * dt * I;

  double valueD = (error - lastError) * D;

  double valuePID = constrain(valueP + valueI + valueD, minValue, maxValue);

  if(!((valuePID >= maxValue) && (sign(error) && sign(valuePID)))) {
    integral += valueI;
  }

  lastError = error;
  lastTime = millis();
}
