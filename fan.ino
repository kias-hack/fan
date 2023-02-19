#define dataPin 10
#define clockPin 11
#define latchPin 12

#define fanPwmPin 9 // TODO сменить вывод управления феном

#define reedSwitchPin 13

#define dimmerPin A1

#define leftPin 4
#define rightPin 5
#define switchPin 6

#define TICK_PERIOD 5 // период обновления дисплея и кнопок
#define MIN_FAN_SPEED 1 // минимальная скорость
#define MAX_FAN_SPEED 99 // максимальная скорость
#define MAX_TEMPERATURE 500 // максимальное значние температуры
#define MIN_TEMPERATURE 100 // минимальное значние температуры

#define COOLING_TEMPERATURE_LEVEL 50 // до какой температуры охлаждать

#define NOTIFY_TIME 2 * 1000 // время показа измененного значения в секундах

#include "kem3361.hpp";

KEM3361 display(dataPin, clockPin, latchPin);

uint16_t target_temperature = 300;
uint16_t temperature = 27;
uint8_t fan_speed = MIN_FAN_SPEED;
bool isNotifyActive = false;
bool lastZeroCross;
uint16_t dimmerValue = 9800;
unsigned long notifyTimer = millis();
unsigned long tickTimer = millis();

void increase_fan_speed(void)
{
  fan_speed += 1;

  if(fan_speed > MAX_FAN_SPEED)
  {
    fan_speed = MAX_FAN_SPEED;
  }
}

void decrease_fan_speed(void)
{
  fan_speed -= 1;

  if(fan_speed < MIN_FAN_SPEED)
  {
    fan_speed = MIN_FAN_SPEED;
  }
}

void increase_temperature(void)
{
  ++target_temperature;

  bool needUpdate = target_temperature > MAX_TEMPERATURE;

  if(needUpdate)
    target_temperature = MAX_TEMPERATURE;
}

void decrease_temperature(void)
{
  --target_temperature;

  bool needUpdate = target_temperature < MIN_TEMPERATURE;

  if(needUpdate)
    target_temperature = MIN_TEMPERATURE;
}

void resetView()
{
  isNotifyActive = true;
  notifyTimer = millis();
}

struct buttons_t {
  uint8_t last_left;
  uint8_t last_right;

  bool left;
  bool right;
  bool sw;
} buttons;

void setup() {
  // инициализация Timer1
  cli(); // отключить глобальные прерывания
  TCCR1A = 0; // установить TCCR1A регистр в 0
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
  // включить прерывание Timer1 overflow:
  TIMSK1 = (1 << OCIE1A);

  sei();  // включить глобальные прерывания
  
  Serial.begin(9600);

  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(dimmerPin, OUTPUT);

  pinMode(leftPin, INPUT);
  pinMode(rightPin, INPUT);
  pinMode(switchPin, INPUT);

  pinMode(reedSwitchPin, INPUT_PULLUP);
  
  digitalWrite(dimmerPin, LOW);

  buttons.last_left = digitalRead(leftPin);
  buttons.last_right = digitalRead(rightPin);

  attachInterrupt(0, dimmerReset, RISING);
}

void dimmerReset(){
    digitalWrite(dimmerPin, LOW);
    OCR1A = map(fan_speed, 0, 99, 2500, 0);
    TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
}

ISR(TIMER1_COMPA_vect)
{
  digitalWrite(dimmerPin, HIGH);
  TCCR1B = 0;
}

void checkButtons()
{
  buttons.sw = digitalRead(switchPin) == LOW;
  buttons.left = buttons.last_left != digitalRead(leftPin);
  buttons.right = buttons.last_right != digitalRead(rightPin);
  buttons.last_left = digitalRead(leftPin);
  buttons.last_right = digitalRead(rightPin);
}

void tick()
{
  checkButtons();

  if(buttons.left && buttons.sw)
  {
    resetView();
    display.print(String(fan_speed));
    decrease_fan_speed();
  } 
  else if(buttons.right && buttons.sw)
  {
    resetView();
    display.print(String(fan_speed));
    increase_fan_speed();
  } 
  else if(buttons.left)
  {
    resetView();
    display.print(String(target_temperature));
    decrease_temperature();
  } 
  else if(buttons.right)
  {
    resetView();
    display.print(String(target_temperature));
    increase_temperature();
  }

  bool resetNotifyActive = isNotifyActive && (millis() - notifyTimer) >= NOTIFY_TIME;

  if(resetNotifyActive)
  {
    isNotifyActive = false;
  }

  if(!isNotifyActive)
  {
    display.print(String(temperature)); 
  }

  display.tick(); 
}

void loop(void) {
  if(digitalRead(reedSwitchPin))
  { 
//    analogWrite(fanPwmPin, map(fan_speedб 0, MAX_FAN_SPEED, 0, 255));
  } else 
  {
    if(temperature > COOLING_TEMPERATURE_LEVEL)
      analogWrite(fanPwmPin, map(fan_speed, 0, MAX_FAN_SPEED, 0, 255)); 
    else
      analogWrite(fanPwmPin, 0); 

    digitalWrite(dimmerPin, LOW);
  }
  
  if(millis() - tickTimer >= TICK_PERIOD)
  {
    tickTimer = millis();
    tick();
  }
}
