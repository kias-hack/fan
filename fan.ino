/**
 * Пины управления семисегментным индикатором
 */
#define dataPin 10
#define clockPin 11
#define latchPin 12

#define fanPwmPin 9 // TODO сменить вывод управления вентилятором

/**
 * Пин с герконом
 */
#define reedSwitchPin 13

/**
 * Пин с термопарой и заземленным АЦП, чтобы компенсировать ноль АЦП
 */
#define thermoCouplePin 0// TODO указать порт
#define groundedADCPin 0// TODO указать порт

/**
 * Пины для управления диммером
 */
#define dimmerPin A1
#define zeroCrossPin 2
#define zeroCrossInterrupt 0

/**
 * Пины обработки энкодера
 */
#define leftPin 4
#define rightPin 5
#define switchPin 6

/**
 * Настройки для диммера
 */
#define DIMMER_MAX_VALUE 2500 // 0.010 / (16000000 / 64) - максимальный период 9 миллисекунд, максимальное значение зависит от делителя
#define DIMMER_WORK_VALUE DIMMER_MAX_VALUE - 100

/**
 * Период обновления дисплея и кнопок (кадры в сеунду умножаем на 3 сегмента)
 */
#define TICK_PERIOD 5 // период обновления дисплея и кнопок, мс

/**
 * Настройки отображения и урпвления скорости вентилятора
 */
#define MIN_FAN_SPEED 50 // минимальная скорость вентилятора в процентах
#define MAX_FAN_SPEED 99 // максимальная скорость вентилятора в процентах

/**
 * Настройки для температуры фена
 */
#define MAX_TEMPERATURE 500 // максимальное значние температуры
#define MIN_TEMPERATURE 100 // минимальное значние температуры
#define DEFAULT_TEMPERATURE 300

/**
 * Коэффициенты термопары
 */
#define THERMO_GAIN 101.0
#define ADC_REFERENCE 3.3
#define ADC_RESOLUTION 1024.0
#define VOLTS_PER_DEGREES_TYPE_K 0.000044
#define ADC_COEF ADC_REFERENCE/ADC_RESOLUTION/THERMO_GAIN/VOLTS_PER_DEGREES_TYPE_K

/**
 * коэффициенты PID регулятора
 */
#define P_COEF 30
#define I_COEF 0
#define D_COEF 0

/**
 * Настройка, до какой температуры охлаждать фен когда сработает геркон
 */
#define COOLING_TEMPERATURE_LEVEL 50 // до какой температуры охлаждать

/**
 * Настройка, в течении какого времени показывать новое установленное значение
 */
#define NOTIFY_TIME 2 * 1000 // время показа измененного значения в секундах

#include "kem3361.hpp";
#include "pid.h";

KEM3361 display(dataPin, clockPin, latchPin);
PID pid(P_COEF, I_COEF, D_COEF, 0, DIMMER_WORK_VALUE);

uint16_t target_temperature = DEFAULT_TEMPERATURE;
uint16_t temperature;
uint8_t fan_speed = MIN_FAN_SPEED;
bool isNotifyActive = false;
bool needComputePID = false;
uint16_t dimmerValue = DIMMER_MAX_VALUE;
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
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12);
  TIMSK1 = (1 << OCIE1A);

  sei();  // включить глобальные прерывания
  
//  Serial.begin(9600);

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
    OCR1A = dimmerValue;
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

void getTemperature(){
    int thermoCoupleRaw = analogRead(thermoCouplePin);
    int groundedADCRaw = analogRead(groundedADCPin);

    thermoCoupleRaw -= groundedADCRaw;

    temperature = thermoCoupleRaw * ADC_COEF;
}

void loop(void) {
  if(digitalRead(reedSwitchPin))
  { 
    if(needComputePID){
      needComputePID = false;
      getTemperature();
      dimmerValue = DIMMER_WORK_VALUE - pid.compute(target_temperature - temperature);
    }
    analogWrite(fanPwmPin, map(fan_speed, 0, MAX_FAN_SPEED, 0, 255));
  } else 
  {
    if(needComputePID){
      needComputePID = false;
      getTemperature();
    }
    
    if(temperature > COOLING_TEMPERATURE_LEVEL)
      analogWrite(fanPwmPin, map(fan_speed, 0, MAX_FAN_SPEED, 0, 255)); 
    else
      analogWrite(fanPwmPin, 0); 

    digitalWrite(dimmerPin, LOW);
    uint16_t dimmerValue = DIMMER_MAX_VALUE;
  }
  
  if(millis() - tickTimer >= TICK_PERIOD)
  {
    tickTimer = millis();
    tick();
  }
}
