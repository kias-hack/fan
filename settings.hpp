#pragma once

/**
 * Пины управления семисегментным индикатором
 */
#define dataPin 10
#define clockPin 11
#define latchPin 12

#define fanPwmPin 3

/**
 * Пин с герконом
 */
#define reedSwitchPin 13

/**
 * Пин с термопарой и заземленным АЦП, чтобы компенсировать ноль АЦП
 */
#define thermoCouplePin A0
#define groundedADCPin A7

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
#define DIMMER_POWER_OFFSET 50
#define DIMMER_MAX_VALUE 2400 // 0.010 / (16000000 / 64) - максимальный период 10 миллисекунд, максимальное значение зависит от делителя
#define DIMMER_WORK_VALUE DIMMER_MAX_VALUE - DIMMER_POWER_OFFSET

/**
 * Период обновления дисплея и кнопок (кадры в сеунду умножаем на 3 сегмента)
 */
#define TICK_PERIOD 5 // период обновления дисплея и кнопок, мс

#define TEMPERATURE_TICK_PERIOD 250 // период обновления дисплея и кнопок, мкс

/**
 * Настройки отображения и урпвления скорости вентилятора
 */
#define MIN_FAN_SPEED 50 // минимальная скорость вентилятора в процентах
#define MAX_FAN_SPEED 99 // максимальная скорость вентилятора в процентах

/**
 * Настройки для температуры фена
 */
#define MAX_TEMPERATURE 500 // максимальное значние температуры
#define MIN_TEMPERATURE 50 // минимальное значние температуры
#define DEFAULT_TEMPERATURE 300
#define WAITING_TEMPERATURE 100

#define WAITING_TIME 60000


/**
 * Коэффициенты термопары
 */
#define THERMO_GAIN 102.0
#define ADC_REFERENCE 3.3
#define ADC_RESOLUTION 256.0
#define VOLTS_PER_DEGREES_TYPE_K 0.00003
#define ADC_COEF ADC_REFERENCE/ADC_RESOLUTION/THERMO_GAIN/VOLTS_PER_DEGREES_TYPE_K

/**
 * коэффициенты PID регулятора
 */
#define P_COEF 35
#define I_COEF 5
#define D_COEF 0.07

/**
 * Настройка, до какой температуры охлаждать фен когда сработает геркон
 */
#define COOLING_TEMPERATURE_LEVEL 50 // до какой температуры охлаждать
#define COOLING_HYSTERESIS 30 // до какой температуры охлаждать

/**
 * Настройка, в течении какого времени показывать новое установленное значение
 */
#define NOTIFY_TIME 2 * 1000 // время показа измененного значения в секундах

#define BUFFER_SIZE 200
