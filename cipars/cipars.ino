// Генератор для катушки Мишина на основе DDS AD9833
// Добавить библиотеку Гайвера -   https://github.com/GyverLibs/GyverButton
// Определения

//#define DEBUG                          // Замаркировать если не нужны тесты
#define UA6EM  // Замаркировать, если скетч для CIPARS
#define SECONDS(x) ((x)*1000UL)
#define MINUTES(x) (SECONDS(x) * 60UL)
#define HOURS(x) (MINUTES(x) * 60UL)
#define DAYS(x) (HOURS(x) * 24UL)
#define WEEKS(x) (DAYS(x) * 7UL)
#define ON_OFF_CASCADE_PIN 5  // Для выключения выходного каскада
#define PIN_ENCODER1 6
#define PIN_ENCODER2 7
#define PIN_ENCODER3 3  // Шимится, по прерыванию обрабатывается энкодер
#define PIN_ENC_BUTTON 8
#define PIN_ZUM 9
#define CORRECT_PIN A7  // Пин для внешней корректировки частоты.

//AD9833
//#define AD9833_MISO
#define AD9833_MOSI A1
#define AD9833_SCK A2
#define AD9833_CS A3

//LCD1602_I2C OR LCD2004_I2C
#define I2C_SDA A4  // LCD1602 SDA
#define I2C_SCK A5  // LCD1602 SCK

//MCP4151
#define MCP41x1_SCK 13   // Define SCK pin for MCP4131 or MCP4151
#define MCP41x1_MOSI 11  // Define MOSI pin for MCP4131 or MCP4151
#define MCP41x1_MISO 12  // Define MISO pin for MCP4131 or MCP4151
#define MCP41x1_CS A0    // Define chipselect pin for MCP4131 or MCP4151

#define zFreq 2  // Делитель интервала - секунда/2

// Глобальные переменные
unsigned long interval = MINUTES(1);
unsigned long oneMinute = MINUTES(1);
unsigned long timers = MINUTES(5);  // время таймера 15, 30, 45 или 60 минут
unsigned long memTimers = 0;        //здесь будем хранить установленное время таймера
unsigned long oldmemTimers = 0;
byte isWorkStarted = 0;  // флаг запуска таймера

unsigned long timMillis = 0;
unsigned long oldMillis = 0;
unsigned long mill;  // переменная под millis()
unsigned long prevCorrectTime = 0;
unsigned long prevReadAnalogTime = 0;  // для отсчета 10 секунд между подстройкой частоты
unsigned long prevUpdateDataIna = 0;   // для перерыва между обновлениями данных ina
unsigned int wiperValue;               // variable to hold wipervalue for MCP4131 or MCP4151
unsigned int Data_ina219 = 0;
long FREQ_MIN = 200000;  // 200kHz
long FREQ_MAX = 500000;  // 500kHz
long ifreq = FREQ_MIN;
long freq = FREQ_MIN;
const unsigned long freqSPI = 250000;  // Частота только для HW SPI AD9833
// UNO SW SPI = 250kHz
const unsigned long availableTimers[] = { oneMinute * 15, oneMinute * 30, oneMinute * 45, oneMinute * 60 };
const byte maxTimers = 4;
int timerPosition = 0;
volatile int newEncoderPos;        // Новая позиция энкодера
static int currentEncoderPos = 0;  // Текущая позиция энкодера
volatile int d_resis = 127;
bool zepper_flag;

#ifndef UA6EM
#define I2C_ADDR 0x27  //0x3F //0x27
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(I2C_ADDR, 20, 4);
#else
#define I2C_ADDR 0x3F
#include <LCD_1602_RUS.h>
LCD_1602_RUS lcd(I2C_ADDR, 16, 2);
#endif

#include <Wire.h>
#include <SPI.h>
#define MCP4151MOD  // снять ремарки если используем библиотеку MCP4151(SPI у неё 16 битный)
#ifndef MCP4151MOD
#include <MCP4131.h>  // https://github.com/UA6EM/Arduino-MCP4131/tree/mpgsp
MCP4131 Potentiometer(MCP41x1_CS);
#else
#include <MCP4151.h>  // https://github.com/UA6EM/MCP4151/tree/mpgsp
MCP4151 Potentiometer(MCP41x1_CS, MCP41x1_MOSI, MCP41x1_MISO, MCP41x1_SCK, 250000UL, 250000UL, SPI_MODE0);
//MCP4151 Potentiometer(MCP41x1_CS);
#endif

#include "INA219.h"
INA219 ina219;

// по умолчанию 50% потенциометра
#ifdef MCP4151MOD
int currentPotenciometrPercent = 127;
#else
int currentPotenciometrPercent = 63;
#endif

//--------------- Create an AD9833 object ----------------
#include <AD9833.h>  // Пробуем новую по ссылкам в README закладке
//AD9833 AD(10, 11, 13);     // SW SPI over the HW SPI pins (UNO);
//AD9833 Ad9833(AD9833_CS);  // HW SPI Defaults to 25MHz internal reference frequency
AD9833 Ad9833(AD9833_CS, AD9833_MOSI, AD9833_SCK);  // SW SPI speed 250kHz

//    *** Используемые подпрограммы выносим сюда ***   //

/*** Обработчик кнопки энкодера ***/
#include "GyverButton.h"  // https://github.com/GyverLibs/GyverButton
GButton Btn1(PIN_ENC_BUTTON);
/*
  boolean isPress();    // возвращает true при нажатии на кнопку. Сбрасывается после вызова
  boolean isRelease();  // возвращает true при отпускании кнопки. Сбрасывается после вызова
  boolean isClick();    // возвращает true при клике. Сбрасывается после вызова
  boolean isHolded();   // возвращает true при удержании дольше timeout. Сбрасывается после вызова
  boolean isHold();     // возвращает true при нажатой кнопке, не сбрасывается
  boolean state();      // возвращает состояние кнопки
  boolean isSingle();   // возвращает true при одиночном клике. Сбрасывается после вызова
  boolean isDouble();   // возвращает true при двойном клике. Сбрасывается после вызова
  boolean isTriple();   // возвращает true при тройном клике. Сбрасывается после вызова
*/

/******* Простой энкодер *******/
#include <util/atomic.h>    // для атомарности чтения данных в прерываниях
#include <RotaryEncoder.h>  // https://www.arduino.cc/reference/en/libraries/rotaryencoder/
RotaryEncoder encoder(PIN_ENCODER1, PIN_ENCODER2);

/*** Обработчик прерывания для энкодера ***/
ISR(PCINT2_vect) {
  encoder.tick();
  // Btn1.tick(); // в прерывании кнопка от Гайвера не работает
}

// функция выбора времени работы
void setTimer() {
  // если энкодер крутим по часовой
  if (newEncoderPos - currentEncoderPos > 0) {
    if (timerPosition == maxTimers - 1) {
      timerPosition = 0;
    } else {
      timerPosition += 1;
    }
  } else if (newEncoderPos - currentEncoderPos < 0) {
    // если энкодер крутим против часовой
    if (timerPosition == 0) {
      timerPosition = maxTimers - 1;
    } else {
      timerPosition -= 1;
    }
  }
  memTimers = availableTimers[timerPosition];
}

void testMCP4151() {
#ifdef MCP4151MOD
  d_resis = 255;
#else
  d_resis = 127;
#endif

  Serial.println("START Test MCP4151");
  for (int i = 0; i < d_resis; i++) {
    Potentiometer.writeValue(i);
    delay(100);
    Serial.print("MCP4151 = ");
    Serial.println(i);
  }
  for (int j = d_resis; j >= 1; --j) {
    Potentiometer.writeValue(j);
    delay(100);
    Serial.print("MCP4151 = ");
    Serial.println(j);
  }
  Serial.println("STOP Test MCP4151");
}

void resetPotenciometer() {
  // Понижаем сопротивление до 0%:
  wiperValue = 1;
  Potentiometer.writeValue(wiperValue);  // Set MCP4151 to position 1
}

// Уровень percent - от 0 до 100% от максимума.
void setResistance(int percent) {
  // resetPotenciometer();
  // for (int i = 0; i < percent; i++) {
  wiperValue = percent;
  ;
  // }
  Potentiometer.writeValue(wiperValue);  // Set MCP4151
}

void processPotenciometr() {
  // если энкодер крутим по часовой
  if (newEncoderPos - currentEncoderPos > 0) {
    if (currentPotenciometrPercent >= d_resis) {
      currentPotenciometrPercent = d_resis;
      wiperValue = d_resis;
    } else {
      currentPotenciometrPercent += 1;
      wiperValue += 1;
    }
  } else if (newEncoderPos - currentEncoderPos < 0) {
    // если энкодер крутим против часовой
    if (currentPotenciometrPercent <= 1) {
      currentPotenciometrPercent = 1;
      wiperValue = 1;
    } else {
      currentPotenciometrPercent -= 1;
      wiperValue -= 1;
    }
  }
  setResistance(currentPotenciometrPercent);
  Potentiometer.writeValue(wiperValue);  // Set MCP4131 to mid position
  Serial.print("wiperValue = ");
  Serial.println(wiperValue);
}

/*** Обработчик энкодера через ШИМ ***/
void startEncoder() {
  attachInterrupt(1, Encoder2, RISING);
  analogWrite(PIN_ENCODER3, 0x80);  // Установим на пине частоту 490 гц скважность 2
}

void Encoder2(void) {  // Процедура вызываемая прерыванием (обрабатываем энкодер)
  encoder.tick();
}

/********* Таймер обратного отсчёта экспозиции **********/
unsigned long setTimerLCD(unsigned long timlcd) {
  if (millis() - timMillis >= 1000) {
    timlcd = timlcd - 1000;
    timMillis += 1000;
  }
  if (timlcd == 0) {
    timlcd = oldmemTimers;
    isWorkStarted = 0;
    lcd.setCursor(0, 3);
#ifdef UA6EM
    lcd.print("    СТОП!     ");
#else
    lcd.print("    STOP!     ");
#endif
    digitalWrite(ON_OFF_CASCADE_PIN, LOW);
    start_Buzzer();
    delay(3000);
    stop_Buzzer();
  }
  return timlcd;
}

/*******************ПИЩАЛКА ********************/
void start_Buzzer() {
  digitalWrite(PIN_ZUM, HIGH);
}

void stop_Buzzer() {
  digitalWrite(PIN_ZUM, LOW);
}

// ******************* Обработка AD9833 ***********************
void /*long*/ readAnalogAndSetFreqInSetup() {
  int maxValue = 0;
  long freqWithMaxI = FREQ_MIN;
  long freqIncrease = 1000;                                   // 1kHz
  int iterations = (FREQ_MAX - FREQ_MIN) / freqIncrease - 1;  // (500000 - 200000) / 1000 - 1 = 199

  for (int j = 1; j <= iterations; j++) {
    // читаем значение аналогового входа
    int tempValue = analogRead(CORRECT_PIN);
    // если значение тока больше предыдущего, запоминаем это значение и текущую частоту
    if (tempValue > maxValue) {
      maxValue = tempValue;
      freqWithMaxI = freq;
    }
    // увеличиваем частоту для дальнейшего измерения тока
    freq = freq + freqIncrease;
    if (freq > FREQ_MAX) {
      freq = FREQ_MAX;
    }
    // подаём частоту на генератор
    Ad9833.setFrequency((float)freq, 0);
    delay(20);
  }
  ifreq = freqWithMaxI;
  // подаём частоту на генератор
  Ad9833.setFrequency((float)ifreq, 0);
  prevReadAnalogTime = millis();
}

/**** Подстройка частоты каждые 1-10 секунд относительно аналогового сигнала ***/
void readAnalogAndSetFreqInLoop() {
  unsigned long curr = millis();

  // если прошло N секунд с момента последней проверки
  if (curr - prevReadAnalogTime > 1000 * 5) {  //выбор времени изменения частоты.1-10 сек.
    long availableDiff = 5000;                 // 1kHz-10kHz разница частот
    long freqIncrease = 500;                   // 100Hz-1kHz шаг увеличения частоты при сканировании

    int iterations = (availableDiff * 2) / freqIncrease - 1;  // (10000 * 2) / 1000 - 1 = 19

    long minimalFreq = ifreq - availableDiff;
    if (minimalFreq < FREQ_MIN) {
      minimalFreq = FREQ_MIN;
    }
    // подаём на генератор минимальную частоту из диапазона +-10кГц
    Ad9833.setFrequency((float)minimalFreq, 0);
    delay(20);

    int maxValue = 0;
    long freqWithMaxI = minimalFreq;
    freq = minimalFreq;

    for (int j = 1; j <= iterations; j++) {
      // читаем значение аналогового входа
      int tempValue = analogRead(CORRECT_PIN);
      // если значение тока больше предыдущего, запоминаем это значение и текущую частоту
      if (tempValue > maxValue) {
        maxValue = tempValue;
        freqWithMaxI = freq;
      }
      // увеличиваем частоту для дальнейшего измерения тока
      freq = freq + freqIncrease;
      if (freq > FREQ_MAX) {
        freq = FREQ_MAX;
      }
      // подаём частоту на генератор
      Ad9833.setFrequency((float)freq, 0);
      delay(10);
    }
    ifreq = freqWithMaxI;
    Ad9833.setFrequency((float)ifreq, 0);
    prevReadAnalogTime = millis();
  }
}


// *** Вывод на дисплей ***
void myDisplay() {
  // 1 строка
  lcd.setCursor(0, 0);
  if (!isWorkStarted) {
#ifdef UA6EM
    lcd.print("Время-");
#else
    lcd.print("Times-");
#endif
    lcd.print(memTimers / 60000);
    if (memTimers / 60000 > 0) {
#ifdef UA6EM
      lcd.print(" мин. ");
#else
      lcd.print(" min. ");
#endif
    } else {
#ifdef UA6EM
      lcd.print("0 мин. ");
#else
      lcd.print("0 min. ");
#endif
    }
  } else {
    lcd.print("Т-");
    if (memTimers > 60000) {
      // если больше минуты, то показываем минуты
      lcd.print(memTimers / 1000 / 60);
#ifdef UA6EM
      lcd.print("мин.");
#else
      lcd.print("min.");
#endif
    } else {
      // если меньше минуты, то показываем секунды
      lcd.print(memTimers / 1000);
#ifdef UA6EM
      lcd.print("сек.");
#else
      lcd.print("sek.");
#endif
    }
    lcd.print(" U=");
#ifdef MCP4151MOD  // можно замапить в реальный % выходного сигнала
    lcd.print(map(currentPotenciometrPercent, 1, 255, 1, 100));
#else
    lcd.print(map(currentPotenciometrPercent, 1, 127, 1, 100));
#endif
    lcd.print("%  ");
  }

  // 2 строка
  lcd.setCursor(0, 1);
  lcd.print("F=");
  //lcd.setCursor(3, 1);                   //1 строка 7 позиция
  float freq_tic = ifreq;
  float kHz = freq_tic / 1000;
  lcd.print(kHz, 0);
  lcd.print("kHz");

  // 2 строка
  lcd.setCursor(9, 1);
  lcd.print("I=");
  lcd.setCursor(11, 1);
  lcd.print(Data_ina219 * 2);
  lcd.print("ma");
}


// *** Вывод на дисплей в режиме ZEPPER ***
void zepDisplay() {
  // 1 строка
  lcd.setCursor(0, 0);
  lcd.print("Т-");
  lcd.print(memTimers / 1000);
#ifdef UA6EM
  lcd.print("сек ");
#else
  lcd.print("sek.");
#endif
  lcd.setCursor(9, 0);
  lcd.print("U=");
#ifdef MCP4151MOD  // можно замапить в реальный % выходного сигнала
  lcd.print(map(currentPotenciometrPercent, 1, 255, 1, 100));
#else
  lcd.print(map(currentPotenciometrPercent, 1, 127, 1, 100));
#endif
  lcd.print("%  ");


  // 2 строка
  if (digitalRead(ON_OFF_CASCADE_PIN)) {
    lcd.setCursor(0, 1);
    if (ifreq / 1000 < 100 && !zepper_flag) {
      lcd.print("          ");
      zepper_flag = true;
    }

    lcd.setCursor(0, 1);
    lcd.print("F=");
    //lcd.setCursor(3, 1);                   //1 строка 7 позиция
    float freq_tic = ifreq;
    float kHz = freq_tic / 1000;
    lcd.print(kHz, 0);
    lcd.print("kHz ");

    // 2 строка

    lcd.setCursor(9, 1);
    lcd.print("I=");
    lcd.setCursor(11, 1);
    Data_ina219 = ina219.shuntCurrent() * 1000;
    lcd.print(Data_ina219 * 2);
    lcd.print("ma");

  } else {

    // 2 строка
    lcd.setCursor(0, 1);
    lcd.print("Freq = OFF      ");
  }
}

// Установка частоты и мощности для режима ZEPPER
// Частота в герцах, напряжение в вольтах от 0 до 12
unsigned long SetFreqToZepper(float zepper_freq, uint8_t power) {
  unsigned long t_zepper = millis();
  ifreq = zepper_freq;
  Ad9833.setFrequency((float)ifreq, 0);
  uint8_t v_out = map(power, 0, 12, 0, 255);  // Переведём напряжение в значение потенциометра
  Potentiometer.writeValue(v_out);
  digitalWrite(ON_OFF_CASCADE_PIN, HIGH);  // Включили выход генератора
  return t_zepper;
}


//************************** SETUP *************************/
void setup() {
  Serial.begin(115200);
  Serial.println("START");

#ifdef MCP4151MOD
  //  lcd.begin();  // Зависит от версии библиотеки
  lcd.init();
#else
  lcd.init();  // https://www.arduino.cc/reference/en/libraries/liquidcrystal-i2c/
#endif

  lcd.backlight();
  delay(1000);

  // сбрасываем потенциометр в 0%
  resetPotenciometer();
  // после сброса устанавливаем значение по умолчанию
  setResistance(currentPotenciometrPercent);

  // ждем секунду после настройки потенциометра
  delay(1000);

  pinMode(ON_OFF_CASCADE_PIN, OUTPUT);
  pinMode(PIN_ZUM, OUTPUT);
  pinMode(CORRECT_PIN, INPUT);

  digitalWrite(PIN_ZUM, LOW);
  digitalWrite(ON_OFF_CASCADE_PIN, HIGH);

  analogReference(INTERNAL);

  ina219.begin(0x40);                 // (44) i2c address 64=0x40 68=0х44 исправлять и в ina219.h одновременно
  ina219.configure(0, 2, 12, 12, 7);  // 16S -8.51ms
  ina219.calibrate(0.100, 0.32, 16, 3.2);

  SPI.begin();
  // This MUST be the first command after declaring the AD9833 object
  Ad9833.begin();               // The loaded defaults are 1000 Hz SINE_WAVE using REG0
  Ad9833.reset();               // Ресет после включения питания
  Ad9833.setSPIspeed(freqSPI);  // Частота SPI для AD9833 установлена 4 MHz
  Ad9833.setWave(AD9833_OFF);   // Turn OFF the output
  delay(10);
  Ad9833.setWave(AD9833_SINE);  // Turn ON and freq MODE SINE the output

  // выставляем минимальную частоту для цикла определения максимального тока
  Ad9833.setFrequency((float)FREQ_MIN, 0);

  Serial.print("freq=");
  Serial.println(FREQ_MIN);

  // Настраиваем частоту под катушку
  readAnalogAndSetFreqInSetup();

  Data_ina219 = ina219.shuntCurrent() * 1000;
  myDisplay();
  delay(1000);
  PCICR |= (1 << PCIE2);  // инициализируем порты для энкодера
  PCMSK2 |= (1 << PCINT20) | (1 << PCINT21);
  startEncoder();

  memTimers = availableTimers[0];  // выставляем 15 минут по умолчанию
#ifdef DEBUG
  testMCP4151();
#endif
  wiperValue = d_resis / 2;
  //currentEncoderPos = wiperValue;
  Potentiometer.writeValue(wiperValue);  // Set MCP4131 or MCP4151 to mid position
}  //******** END SETUP ********//


// *** ТЕЛО ПРОГРАММЫ ***
void loop() {
  mill = millis();
  Btn1.tick();
  if (Btn1.isDouble()) {
    zepper_01();
  }

  if (Btn1.isHolded()) {
    oldmemTimers = memTimers;
    timMillis = millis();
    isWorkStarted = 1;
  }

  if (mill - prevUpdateDataIna > 1000 * 2) {
    Data_ina219 = ina219.shuntCurrent() * 1000;
    prevUpdateDataIna = millis();
  }
  myDisplay();
  if (isWorkStarted == 1) {
    memTimers = setTimerLCD(memTimers);
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    newEncoderPos = encoder.getPosition();
  }

  // если значение экодера поменялось
  if (currentEncoderPos != newEncoderPos) {
    // если работа ещё не началась, то можем устанавливать время
    if (isWorkStarted == 0) {
      setTimer();
    } else if (isWorkStarted == 1) {
      // если работа ещё не началась, то можем редактировать потенциометр
      processPotenciometr();
    }
    currentEncoderPos = newEncoderPos;
  }
  readAnalogAndSetFreqInLoop();

} /****************** E N D  L O O P *****************/


// ************* ПРОГРАММА ЦЕППЕР для ОЧКОВ ************
// Очки, программа 1
// 473kHz - 2 минуты
// 395kHz - 2 минуты
// 403.85 - 2 минуты
// 397.6  - 2 минуты
// Электроды
// 30kHz - 7 минут
// ПЕРЕРЫВ - 20 минут
// 30kHz - 7 минут

void zepper_01() {

  digitalWrite(ON_OFF_CASCADE_PIN, HIGH);  // Включили выход генератора
  Serial.println("Программа ZEPPER 1");
  Serial.println("Генератор включен");

  // Работа с излучателем "ОЧКИ"
  // Шаг первый, частота 473кГц время 2 минуты Uвых - 5 вольт
  float freq_zepper = 473000;  // Частота 473 Кгц
  uint8_t tau_freq = 2;
  uint8_t v_zepper = 5;  // 5 вольт на выход для очков

  // Установим параметры генерации ШАГ 1
  setFreqExposite(freq_zepper, tau_freq, v_zepper);
  Serial.println("Первый шаг пройден");

  // Установим параметры генерации ШАГ 2
  freq_zepper = 395000;  // Частота 395 Кгц
  tau_freq = 2;
  v_zepper = 5;  // 5 вольт на выход для очков
  setFreqExposite(freq_zepper, tau_freq, v_zepper);
  Serial.println("Второй шаг пройден");

  // Установим параметры генерации ШАГ 3
  freq_zepper = 403850;  // Частота 403.85 Кгц
  tau_freq = 2;
  v_zepper = 5;  // 5 вольт на выход для очков
  setFreqExposite(freq_zepper, tau_freq, v_zepper);
  Serial.println("Третий шаг пройден");

  // Установим параметры генерации ШАГ 4
  freq_zepper = 397600;  // Частота 397.6 Кгц
  tau_freq = 2;
  v_zepper = 5;  // 5 вольт на выход для очков
  setFreqExposite(freq_zepper, tau_freq, v_zepper);
  Serial.println("Четвёртый шаг пройден");

  // Работа с излучателем "ЭЛЕКТРОДЫ"
  Serial.println("Возьмите в руки электроды ");
  // Установим параметры генерации ШАГ 1
  freq_zepper = 30000;  // Частота 30.0 Кгц
  tau_freq = 7;
  v_zepper = 12;  // 12 вольт на выход для электродов
  setFreqExposite(freq_zepper, tau_freq, v_zepper);
  Serial.println("Первый шаг пройден");

  Serial.println("Отдыхаем 20 минут");
  digitalWrite(ON_OFF_CASCADE_PIN, LOW);  // Выключили выход генератора

  freq_zepper = 30000;  // Частота 30.0 Кгц
  tau_freq = 20;
  v_zepper = 0;  // 0 вольт на выход для электродов
  setFreqExposite(freq_zepper, tau_freq, v_zepper);

  Serial.println("Шаг 2 - возьмите в руки электроды");
  delay(5000);
  digitalWrite(ON_OFF_CASCADE_PIN, HIGH);
  freq_zepper = 30000;  // Частота 30.0 Кгц
  tau_freq = 7;
  v_zepper = 12;  // 12 вольт на выход для электродов
  setFreqExposite(freq_zepper, tau_freq, v_zepper);
  Serial.println("Второй шаг пройден");
  zepper_flag = false;

  digitalWrite(ON_OFF_CASCADE_PIN, LOW);  // Выключили выход генератора
  Serial.println("Программа ZEPPER 1");
  Serial.println("Генератор выключен");
  ifreq = FREQ_MIN;

}  // *** END ZEPPER ***


void setFreqExposite(uint32_t freq, uint8_t minute, uint8_t power) {
  uint32_t t_exposite = MINUTES(1);
  uint32_t t_zepper;
  uint32_t t_exR = minute * t_exposite;  // Рассчётное время экспозиции
  memTimers = t_exR;                     // Остаточное время экспозиции
  ifreq = freq;

  t_zepper = SetFreqToZepper(ifreq, power);
  Serial.print("Частота ");
  Serial.print(ifreq / 1000);
  Serial.println(" kHz");

  while ((millis() - t_zepper) <= t_exR) {
    if ((millis() - t_zepper) % 10000) {
      memTimers = t_exR - ((millis() - t_zepper));
      zepDisplay();
    }
  }
}

/*
   A0 - MCP41x1_CS       CS
   A1 - AD9833_MOSI      AD9833_SDATA
   A2 - AD9833_SCK       AD9833_SCLK
   A3 - AD9833_CS        AD9833_FSYNC
   A4 - LCD1602/LCD2004  SDA
   A5 - LCD1602/LCD2004  SCL
   A6 -
   A7 - SENS_IMPLOSION
   D0 - RX
   D1 - TX
   D2 -
   D3 - PWM, tic.encoder() (virtual, not connected)
   D4 -
   D5 - LT1206_SHUTDOWN
   D6 - ENC_DT
   D7 - ENC_CLK
   D8 - ENC_SW
   D9 - BUZZER
   D10 -
   D11 - MCP41x1_MOSI MOSI  SDI/SDO
   D12 - MCP41x1_MISO MISO  only shematic diagram to SDI/SDO
   D13 - MCP41x1_SCK  SCK   SCK

   D14 - A0
   D15 - A1
   D16 - A2
   D17 - A3
   D18 - A4
   D19 - A5
*/
