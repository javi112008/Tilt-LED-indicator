#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

int tiltVal;
int latchPin = 11;      // (11) ST_CP [RCK] on 74HC595
int clockPin = 9;      // (9) SH_CP [SCK] on 74HC595
int ledDataPin = 12;     // (12) DS [S1] on 74HC595
int j;
byte ledData;
Adafruit_MPU6050 mpu;

void updateShiftRegister(int latchPin, int ledDataPin, int clockPin, int bitOrder, byte ledData){
   digitalWrite(latchPin, LOW);
   shiftOut(ledDataPin, clockPin, bitOrder, ledData);
   digitalWrite(latchPin, HIGH);
}





void setup() {
 


  Serial.begin(115200);
  while (!Serial) delay(10);

  // Try to initialize MPU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  delay(100);


 pinMode(latchPin, OUTPUT);
  pinMode(ledDataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
}

void loop() {
 

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Convert acceleration to tilt (pitch/roll)
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // Calculate roll (side-to-side) and pitch (front-back)
  float pitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float roll  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

//you should set the following values to 0, so you first see how much you need to compensate for. ( you may change based on your work surface  :) )

pitch+=4.4; // compensate for uneven surface 
roll+=0.3; // compensate for uneven surface 

  // Output
  Serial.print("Pitch: ");
  Serial.print(pitch, 0);
  Serial.print("°,  Roll: ");
  Serial.print(roll, 0);
  Serial.println("°");

  delay(100);

  tiltVal= map(roll,-80,80,8,-8); //maps pitch values to a vlaue that can be turned into ledData 
  tiltVal = constrain(tiltVal, -3, 3); // constrained to only 3 lights turning on 


ledData = 0;

// Always light center LEDs (3 and 4)
bitSet(ledData, 3);
bitSet(ledData, 4);


  if (tiltVal > 0) {
    for (int i = 5; i < 5 + tiltVal && i < 8; i++) bitSet(ledData, i);
  } else if (tiltVal < 0) {
    for (int i = 2; i >= 3 + tiltVal && i >= 0; i--) bitSet(ledData, i);
  }

  updateShiftRegister(latchPin, ledDataPin, clockPin, LSBFIRST, ledData);
  delay(50);
}
