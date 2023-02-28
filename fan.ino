#include "kem3361.hpp";
#include "pid.h";
#include "filter.hpp";
#include "settings.hpp";

KEM3361 display(dataPin, clockPin, latchPin);
PID pid(P_COEF, I_COEF, D_COEF, 0, DIMMER_WORK_VALUE);
MeanFilter filter;

uint16_t target_temperature = DEFAULT_TEMPERATURE;
uint16_t tempTargetTemperature = target_temperature;
uint16_t temperature;
uint8_t fan_speed = MIN_FAN_SPEED;
bool isNotifyActive = false;
bool needComputePID = false;
uint16_t dimmerValue = DIMMER_MAX_VALUE;
unsigned long notifyTimer = millis();
unsigned long tickTimer = millis();
unsigned long getTemperatureTimer = micros();
unsigned long waitingTimer = WAITING_TIME;
unsigned int dimmerCalcCounter = 0;
int lastReedSwitchState = 0;
int coolingTemperatureLevel = COOLING_TEMPERATURE_LEVEL;
bool coolingProcess = true;


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
  target_temperature += 5;

  bool needUpdate = target_temperature > MAX_TEMPERATURE;

  if(needUpdate)
    target_temperature = MAX_TEMPERATURE;
}

void decrease_temperature(void)
{
  target_temperature -= 5;

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
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
  TIMSK1 = (1 << OCIE1A);

  sei();  // включить глобальные прерывания
  
//  Serial.begin(115200);

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

  attachInterrupt(zeroCrossInterrupt, dimmerReset, RISING);
}

void dimmerReset(){
  digitalWrite(dimmerPin, LOW);
  if(!coolingProcess){
     OCR1A = dimmerValue;
     TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
  }
}

ISR(TIMER1_COMPA_vect)
{
  digitalWrite(dimmerPin, HIGH);
  TCCR1B = 0;
  OCR1A = DIMMER_MAX_VALUE;
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

void getTemperature(){
    int thermoCoupleRaw = analogRead(thermoCouplePin) >> 2;

    filter.push(thermoCoupleRaw * ADC_COEF);

    temperature = filter.mean();
}

void loop(void) {
  bool reedSwitch = digitalRead(reedSwitchPin);
  
  if(lastReedSwitchState != reedSwitch){
    if(reedSwitch){
      target_temperature = tempTargetTemperature;
    } else {
      tempTargetTemperature = target_temperature;
      target_temperature = WAITING_TEMPERATURE;
      waitingTimer = millis();
    }
    
    pid.reset();
  }

  lastReedSwitchState = reedSwitch;
  
  if(reedSwitch)
  { 
    coolingProcess = false;
    coolingTemperatureLevel = COOLING_TEMPERATURE_LEVEL;
    analogWrite(fanPwmPin, map(fan_speed, 0, MAX_FAN_SPEED, 0, 255));
  } else 
  { 
    if(millis() - waitingTimer > WAITING_TIME){
      coolingProcess = true;
      
      if(temperature > coolingTemperatureLevel){
        coolingTemperatureLevel = COOLING_TEMPERATURE_LEVEL;
        analogWrite(fanPwmPin, 255); 
      } else {
        coolingTemperatureLevel = COOLING_TEMPERATURE_LEVEL + COOLING_HYSTERESIS;
        analogWrite(fanPwmPin, 0);
      } 
    }
  }

  if(micros() - getTemperatureTimer >= TEMPERATURE_TICK_PERIOD){
    dimmerCalcCounter += 1;
    getTemperature();
    getTemperatureTimer = micros();

    if(dimmerCalcCounter >= BUFFER_SIZE) {
      dimmerCalcCounter = 0;
      dimmerValue = DIMMER_POWER_OFFSET + (DIMMER_WORK_VALUE - pid.compute(target_temperature - temperature));
    }
  }
  
  if(millis() - tickTimer >= TICK_PERIOD)
  {
    tickTimer = millis();
    tick();
  }
}
