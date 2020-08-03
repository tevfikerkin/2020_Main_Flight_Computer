#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;

Adafruit_BMP280 bmp;

File R38;

Servo noseCone; Servo payloadLock; Servo payloadRod; Servo mainChute;

byte error, bmp280 = 119, mpu6050 = 104, bmpErr, mpuErr;
char messageBox[2];
int mainState = 0;
int chipSelect = 10;
int refAlt, preAlt, Alt, apogee = 0 , i = 1, button = 2, Kled = A2, Mled = A3, buzzer = 8, higher, counter = 1,b=1;


void setup() {
  Serial.begin(9600);

  // Confirm all sensor is running !!
  for (int k = 0; k < 3; k++) {

    if (!bmp.begin())
      counter++;

    else if (!mpu.begin())
      counter++;

    else if (!SD.begin(chipSelect))
      counter++;

  }
  if (counter >= 2) // if counter == 1 , main computer sensors are running and main flight computer will controll the flight !!
    mainState = 1;

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X4, // Pressure Oversampling, this value (X4) obtain from datasheet and using for outdoor dynamic applications. Standard val. is X16
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  pinMode(2, OUTPUT); pinMode(Mled, OUTPUT); pinMode(Kled, OUTPUT); pinMode(buzzer, OUTPUT); // indicator to confirm the computer is started.
  noseCone.attach(3); payloadLock.attach(4); payloadRod.attach(5); mainChute.attach(6); // Servos
  digitalWrite(Kled, HIGH); digitalWrite(Mled, HIGH);
  delay(100);
  digitalWrite(Kled, LOW); digitalWrite(Mled, LOW);

}

void loop() {

while (b < 3) {
      digitalWrite(buzzer, HIGH);
      delay(100);
      digitalWrite(buzzer, LOW);
      delay(100);
      b++;
    }
  bmpErr = sensorState(bmp280);
  mpuErr = sensorState(mpu6050);

  if (bmpErr == 0 && mpuErr == 0 ) {
    digitalWrite(Mled, HIGH);
    sendState();

    Serial.println(F(" Computer is Running"));
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    if (i==10) {
      refAlt = bmp.readAltitude(1013.25);
    }

    Alt = bmp.readAltitude(1013.25) - refAlt;
    
    if (Alt > preAlt && i > 10) {

      higher = Alt;
    }
    
    decisionLoop(refAlt, Alt, preAlt, a.acceleration.x, a.acceleration.y, a.acceleration.z, digitalRead(button));
    
    if (i == 11) { //başlıkları bir kere yazdırmak için kullandım.
      R38 = SD.open("R38-1.txt", FILE_WRITE);
      R38.print("İrtifa(m) \t"); R38.print("X ekseni ivme(m/s^2) \t"); R38.print("Y ekseni ivme(m/s^2) \t"); R38.print("Sıcaklık \t"); R38.println("Faydalı Yük Durum \n");
      R38.close();
    }
    R38 = SD.open("R38-1.txt", FILE_WRITE);
    R38.print(Alt); R38.print("\t\t"); R38.print(abs(a.acceleration.x)); R38.print("\t\t\t"); R38.print(abs(a.acceleration.y)); R38.print("\t\t\t"); R38.print(bmp.readTemperature());R38.print("\t\t\t");R38.println(digitalRead(button));
    R38.close();

    i++;
    preAlt = Alt;
    delay(50);
  }
  else
  {
    mainState = 1; // 0 main flight computer is running, 1 means the auxiliary computer is activated.
    sendState();
    Serial.println(F("Auxiliary Computer is Activated !!"));
    digitalWrite(Mled, LOW);
  }
}

void decisionLoop(int refAlt, int Alt, int preAlt, int accX,  int accY, int accZ, int button) {

  if (Alt > 1 && higher - Alt >= 2 && accX < 20) {

    apogee = 1; // Tepe noktasına ulaşıldığını ve artık verilerin inişe ait veriler olduğunu anlamak için tanımladım.
    /*noseCone.write(180);
      payloadLock.write(180);
      payloadRod.write(180);*/
    Serial.println(F("Tepe Noktasına Ulaşıldı."));

  }
  else if (Alt > 1 && higher - Alt >= 2 && accY < 20) {

    apogee = 1; // Tepe noktasına ulaşıldığını ve artık verilerin inişe ait veriler olduğunu anlamak için tanımladım.
    /*noseCone.write(180);
      payloadLock.write(180);
      payloadRod.write(180);*/
    Serial.println(F("Tepe Noktasına Ulaşıldı."));
  }
  if (apogee != 0 && button == 0) {
    Serial.println(F("Faydalı Yük Ayrıldı"));
    Serial.println(F("Sürükleme Paraşütü Açıldı"));

  }

  if (apogee != 0 && Alt <= 2.5) {
    //mainChute.write(180);
    Serial.println(F("ANA PARAŞÜT AÇILDI"));
  }

}

void sendState() {
  sprintf(messageBox, "%d", mainState);
  Serial.write(messageBox);
}

byte sensorState(byte address) {
  Wire.beginTransmission(address);
  error = Wire.endTransmission();
  return error; // if error is 1, there is a problem and auxialary computer activated else there is no problem and error is equal to 0 and main computer remains its works.
}

