//ідейний надихач AlexGyver

// -------НАСТРОЙКИ-------

// сонар
#define ECHO 6
#define TRIG 7

// дисплей
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

//time
#include <TimerOne.h>

// крутая библиотека сонара
#include <NewPing.h>
NewPing sonar(TRIG, ECHO, 400);

float dist_3[3] = {0.0, 0.0, 0.0};   // массив для хранения трёх последних измерений
float middle, dist, dist_filtered, len;
float k;
byte i, delta;
unsigned long last_press, sensTimer;
float _err_measure = 0.3;  // примерный шум измерений
float _q = 0.01;           // скорость изменения значений 0.001-1, варьировать самому

boolean butt_flag = 0;  //змінні для кнопок
boolean butt_knop = 0;
boolean buttf, buttk;
int iter_fun = 0;

void setup() {
  Serial.begin(9600);
  //Serial.flush();
  //Serial.println("sonar_dist");
  pinMode(8,INPUT_PULLUP);
  pinMode(9,INPUT_PULLUP);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("length:");
  lcd.setCursor(0, 1);
  lcd.print("num_fun:");
  lcd.setCursor(9, 1);
  lcd.print(iter_fun);
}

void loop() {
  if (millis() - sensTimer > 50) {                          // измерение и вывод каждые 50 мс
    // счётчик от 0 до 2
    // каждую итерацию таймера i последовательно принимает значения 0, 1, 2, и так по кругу
    len = (float)sonar.ping() / 57.5;

    buttf = !digitalRead(8);  // логіка двух кнопок
    buttk = !digitalRead(9);
    if (buttf == 1 && butt_flag == 0 && millis() - last_press > 1000){
      iter_fun ++;
      lcd.setCursor(9, 1);
      lcd.print(iter_fun);
      last_press = millis();
      }
    if (buttk == 1 && butt_knop == 0 && millis() - last_press > 1000){
      iter_fun = 0;
      lcd.setCursor(9, 1);
      lcd.print(iter_fun);
      last_press = millis();
      }
      
    creation_arr(dist_3);    
    Serial.print(dist_3[1]);//0
    Serial.print(",");

    switch (iter_fun){
      case 0:
        dist_filtered = medium_arithmetic(dist_3);
        Serial.println(dist_filtered);//0 
        break;
      case 1:
        dist_filtered = middle_of_3(dist_3[0], dist_3[1], dist_3[2]);
        Serial.println(dist_filtered);//1
        break;
      case 2:
        dist_filtered = middle_of_dist(dist_3[0], dist_3[1], dist_3[2]);
        Serial.println(dist_filtered);//2
        break;
      case 3:
        dist_filtered = simpleKalman(dist_3[1]);
        Serial.println(dist_filtered);//3
        break;
      default:
        Serial.print(medium_arithmetic(dist_3));//0
        Serial.print(",");
    
        Serial.print(middle_of_3(dist_3[0], dist_3[1], dist_3[2]));//1    
        Serial.print(",");
    
        Serial.print(middle_of_dist(dist_3[0], dist_3[1], dist_3[2]));//2
        Serial.print(",");

        dist_filtered = simpleKalman(dist_3[1]);
        Serial.println(dist_filtered);//3
      }

    lcd.setCursor(8, 0);
    lcd.print(dist_filtered);
    //Serial.println(dist_filtered);
    
    sensTimer = millis();                                   // сбросить таймер
  }

}

void creation_arr(float arr[]){
  if (i > 1) i = 0;
  else i++;
  //delay(100); 
  arr[i] = (float)sonar.ping() / 57.5;                  // получить расстояние в текущую ячейку массива
  }

// медианный фильтр из 3ёх значений
float middle_of_3(float a, float b, float c) {
  if ((a <= b) && (a <= c)) {
    middle = (b <= c) ? b : c;
  }
  else {
    if ((b <= a) && (b <= c)) {
      middle = (a <= c) ? a : c;
    }
    else {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}

// середнє арифметичне
float medium_arithmetic(float *dist_3 ){
  float sum_dist;
  for(int i=0;i<sizeof(dist_3);i++){
    sum_dist = dist_3[i] + sum_dist;
    }
    return (sum_dist/sizeof(dist_3));
  }

float middle_of_dist(float a, float b, float c){
  dist = middle_of_3(dist_3[0], dist_3[1], dist_3[2]);    // фильтровать медианным фильтром из 3ёх последних измерений
  delta = abs(dist_filtered - dist);                     // расчёт изменения с предыдущим
  if (delta > 1) k = 0.7;                               // если большое - резкий коэффициент
  else k = 0.1;                                        // если маленькое - плавный коэффициент
  dist_filtered = dist * k + dist_filtered * (1 - k); // фильтр "бегущее среднее"
  return dist_filtered;
  }

//Простой “Калман”
float simpleKalman(float newVal) {
  float _kalman_gain, _current_estimate;
  static float _err_estimate = _err_measure;
  static float _last_estimate;
  _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}
