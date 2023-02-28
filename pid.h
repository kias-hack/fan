#pragma once

#include "Arduino.h"


class PID {
  private:
    double P;
    double I;
    double D;
    
    double minValue;
    double maxValue;
    
    double lastError;
    
    unsigned long lastTime;
  
    char sign(double);

  public:
    double integral;
    PID(double, double, double, double, double);
    int compute(int);
    void reset();
};

PID::PID(double P, double I, double D, double minValue, double maxValue) : P(P), I(I), D(D), minValue(minValue), maxValue(maxValue){
  integral = 0;
  lastError = 0;  
}

void PID::reset(){
  integral = 0;
  lastError= 0;
  lastTime = millis();
}

char PID::sign(double v){
  if(v >= 0) return 1;
  return -1;
}

int PID::compute(int error) {
  double dt = (millis() - lastTime) / 1000.0;

  int valueP = error * P;
  double valueI = error * dt * I;
  int valueD = (error - lastError) * D;

  int pidValue = valueP + valueI + valueD + integral;

  if(pidValue > maxValue) pidValue = maxValue;
  if(pidValue < minValue) pidValue = minValue;

  if(!((pidValue >= maxValue) && (sign(error) && sign(pidValue)))) {
    integral += valueI;
  }

  lastError = error;
  lastTime = millis();

  return pidValue;
}
