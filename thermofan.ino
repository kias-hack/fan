#define dataPin 10
#define clockPin 11
#define latchPin 12

#define leftPin 4
#define rightPin 5
#define switchPin 6

#include "KEM3361.hpp"

KEM3361 display(dataPin, clockPin, latchPin);

struct buttons_t {
  uint8_t last_left;
  uint8_t last_right;

  bool left;
  bool right;
  bool sw;
} buttons;

void setup() {
  Serial.begin(9600);

  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);

  pinMode(leftPin, INPUT);
  pinMode(rightPin, INPUT);
  pinMode(switchPin, INPUT);

  buttons.last_left = digitalRead(leftPin);
  buttons.last_right = digitalRead(rightPin);
}

int num = 0;

void loop() {
  buttons.left = buttons.last_left != digitalRead(leftPin);
  buttons.right = buttons.last_right != digitalRead(rightPin);
  buttons.last_left = digitalRead(leftPin);
  buttons.last_right = digitalRead(rightPin);

  if(buttons.left && num > 0){
    --num;
  }

  if(buttons.right && num < 999){
    ++num;
  }

  display.print(String(num));
  display.tick();

  delay(5);
}
