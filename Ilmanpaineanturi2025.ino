/*
ISP ohjelmointirajapinta / programming pins
Arduino UNO     Atmega328P 3.3V 8MHz ulkoinen kide / external crystal
10              1         RESET
11              17        MOSI 5V -> 3.3V Voltage division
12              18        MISO
13              19        SCK  5V -> 3.3V 

*/
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "LiquidCrystal.h"

// BMP280 I2C
Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// LCD-pins and LCD setup
const int rs = 7, en = 6, d4 = 14, d5 = 15, d6 = 16, d7 = 17;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

uint8_t deviceMode = 0;
unsigned long deBounceDelay = 100;
unsigned long deBounceLast = 0;

float measurement1, measurement2 = 0;

void setup() {
  deviceMode = 0;
//Pins PD2 and PD3 (INT0 and INT1) set to input-pullup
  DDRD &= ~(1 << PD2) & ~(1 << PD3);
  PORTD |= (1 << PD2) | (1 << PD3);

//Enable interrupts on descending edge for INT0 and INT1
  EICRA |= (1 << ISC01) | (1 << ISC11);
  EICRA &= ~((1 << ISC00) | (1 << ISC10));

//Enable microcontroller interrupts
  EIMSK |= (1 << INT0) | (1 << INT1);

  lcd.begin(16, 2);

  unsigned status;
  status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!status) { //Check address: 0x76 or 0x77
    lcd.print("BMP280 fail");
    while (1) delay(10);
  }
//BMP280 settings
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  lcd.print("BMP280 ready");
  sei();
  delay(1000);
}

//BMP280 normal mode
void normalMode() {
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
}

//BMP280 sleep tilaan
void sleepMode(){
  bmp.setSampling(Adafruit_BMP280::MODE_SLEEP,
                  Adafruit_BMP280::SAMPLING_X1,
                  Adafruit_BMP280::SAMPLING_X1,
                  Adafruit_BMP280::FILTER_OFF,
                  Adafruit_BMP280::STANDBY_MS_1);
}

//Atmega328 sleep mode
void sleepMode328() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
  sei();
  sleep_cpu();
  sleep_disable();
}

void loop() {


  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  if (deviceMode == 1) {
    normalMode();
    float tempC = bmp.readTemperature();
    char buffer[10];

    lcd.clear();
    lcd.print("Tempearature:");
    lcd.setCursor(0, 1);
    dtostrf(tempC, 5, 1, buffer);
    lcd.print(buffer);
    lcd.write(223); //Symbol for degree
    lcd.print("C");
  }

  if (deviceMode == 2) {
    normalMode();
    float pHa = bmp.readPressure();
    char buffer2[10];

    lcd.clear();
    lcd.print("Pressure:");
    lcd.setCursor(0, 1);
    dtostrf(pHa, 5, 1, buffer2);
    lcd.print(buffer2);
    lcd.print(" pHa");
  }
  //Activate sleep mode on microcontroller and sensor
  if (deviceMode == 3){
    lcd.clear();
    sleepMode();
    sleepMode328();
  }

  if (deviceMode >= 4 && deviceMode != 6){
    normalMode();
    if (deviceMode == 5){
      //Take final measurement after button press and return the result in altitude differnece
      float height = 0;
      char buffer3[10];
      measurement2 = bmp.readPressure();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Height: ");

      //Calculate heigth from measured air pressure
      //Divide by 100 to get hPa
      measurement1 /= 100;
      measurement2 /= 100;
      //Calculate height in meters from measurements
      measurement1 = 44330 * (1.0 - pow(measurement1/1013.25, 0.1903));
      measurement2 = 44330 * (1.0 - pow(measurement2/1013.25, 0.1903));
      height = measurement1 - measurement2;
      height = fabs(height);
      dtostrf(height, 5, 1, buffer3);
      lcd.setCursor(0, 1);
      lcd.print(buffer3);
      lcd.print("m");

      deviceMode = 6;
      measurement1 = NULL;
      measurement2 = NULL;

    } else if(measurement1 >= 1){
      //If first measurement has been taken and measure button has not been pressed
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Measuring");
    } else {
      //Read first measurement
      measurement1 = bmp.readPressure();
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Measuring");
    }



  }
  delay(100);
}

//Button interrupt routine with debounce
ISR(INT0_vect) {
  unsigned long now = millis();
  if ((now - deBounceLast) > deBounceDelay) {
    deBounceLast = now;
    if (deviceMode >= 3) {
      deviceMode = 0;
    }
    deviceMode = deviceMode + 1;
  }
}

ISR(INT1_vect) {
  
  unsigned long now = millis();
  if ((now - deBounceLast) > deBounceDelay) {
    deBounceLast = now;
    if (deviceMode == 4){
      deviceMode = 5;
    } else {
    deviceMode = 4;
    }
    
  }
}

/*
Funktiot:
Nappi interruptit -> flag -> main kutsuu jotain
-mittaustilan vaihto
--lämpötila, ilmanpaine, korkeus(käyttää toista nappia kalibrointiin)
Mittaus
-mittaus funktio jokaiselle mittaustilalle
-tulos printataan LCD näytölle
Korkeus
-näyttää nykyisen korkeuden merenpintaan suhteutettuna 1 rivillä
-2 nappia painamalla asetetaan ensimmäinen arvo, nappia uudelleen painamalla asetetaan
toinen arvo, jonka jälkeen lasketaan korkeuden muutos arvojen mukaan.
-Mittauksen aikana LCD->"Measuring"
-Mittaustulos tulostetaan LCD näytölle, 2 nappia painamalla näytetään nykyinen korkeus



*/

















