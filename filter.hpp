#pragma once

#include "settings.hpp"

class MeanFilter {
  public:
  MeanFilter(){
    memset(buffer, 0, BUFFER_SIZE);
  }

  void push(double v){
    shift();
    buffer[BUFFER_SIZE-1] = v;
  }

  double mean(){
    double sum = 0;
    for (int i = 0; i < BUFFER_SIZE; ++i){
      sum += buffer[i];  
    }
    
    return sum/BUFFER_SIZE; 
  }

  protected:

  void shift(){
    for (int i = 0; i < BUFFER_SIZE - 1; ++i){
        buffer[i] = buffer[i+1];
    }
  }
  
  int buffer[BUFFER_SIZE];
  unsigned int pointer;
};
