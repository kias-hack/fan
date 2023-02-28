#include "kem3361.hpp"

KEM3361::KEM3361(uint8_t pinData, uint8_t pinClock, uint8_t pinLatch) : pinData(pinData), pinClock(pinClock), pinLatch(pinLatch) {}

void KEM3361::tick(){
  if(digitIndex > 2) digitIndex = 0;

  registerToOut = encode(digits[digitIndex], digitIndex+1, dots[digitIndex]);

  wordOut();

  ++digitIndex;
}

void KEM3361::print(String str){
  int8_t i = str.length() - 1;
  int8_t d = 2;

  digits[0] = ' ';
  digits[1] = ' ';
  digits[2] = ' ';
  dots[0] = false;
  dots[1] = false;
  dots[2] = false;

  bool dot = false;

  while(i >= 0 && d >= 0) {
    if(str.charAt(i) == '.'){
      dot = true;
      --i;
      continue;
    }

    digits[d] = str.charAt(i);
    dots[d] = dot;

    dot = false;

    --d;
    --i;
  }
}


// c . d g    b 3 2 f     a 1 e
// 0 1 2 3    4 5 6 7     8 9 10
uint16_t KEM3361::encode(char ch, uint8_t digit, bool dot = false){
  uint16_t out = 0b10110011111;

  switch(ch){
    case '1':
      out = 0b0000010110001110;
      break;
    case '2':
      out = 0b0000000010000011;
      break;
    case '3':
      out = 0b0000010010000010;
      break;
    case '4':
      out = 0b0000010100000110;
      break;
    case '5':
      out = 0b0000010000010010;
      break;
    case '6':
      out = 0b0000000000010010;
      break;
    case '7':
      out = 0b0000010010001110;
      break;
    case '8':
      out = 0b0000000000000010;
      break;
    case '9':
      out = 0b0000010000000010;
      break;
    case '0':
      out = 0b0000000000001010;
      break;
    default:
      return out;
  }

  switch (digit) {
    case 1: 
      out |= 0b0000001000000000;
      break;
    case 2: 
      out |= 0b0000000001000000;
      break;
    case 3: 
      out |= 0b0000000000100000;
      break;
  }

  if(dot){
    out ^= 0b10;
  }

  return out;
}

void KEM3361::wordOut(){
  digitalWrite(pinLatch, HIGH);
  for (int i = 0; i < 11; ++i){
    digitalWrite(pinData, (registerToOut >> i) & 0b1);
    digitalWrite(pinClock, HIGH);
    digitalWrite(pinClock, LOW);
    digitalWrite(pinData, LOW);
  }
  digitalWrite(pinLatch, LOW);
}
