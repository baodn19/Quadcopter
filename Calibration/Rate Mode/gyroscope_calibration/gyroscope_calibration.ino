#include <Wire.h>

float rate_roll, rate_pitch, rate_yaw; // For one measurement
float rate_calibration_roll = 0; 
float rate_calibration_pitch = 0; 
float rate_calibration_yaw = 0; 

void TakeMeasurement(void) {
  // Low pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Sensitivity level
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Allocate memory
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  // Measure
  int16_t X = Wire.read() << 8 | Wire.read();
  int16_t Y = Wire.read() << 8 | Wire.read();
  int16_t Z = Wire.read() << 8 | Wire.read();
  rate_roll = (float)X / 65.5;
  rate_pitch = (float)Y / 65.5;
  rate_yaw = (float)Z / 65.5;
}

void setup() {
  Serial.begin(57600);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // Access to the gyroscope power
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Calibration value
  for (int i = 0; i < 2000; i++) {
    TakeMeasurement();

    rate_calibration_roll += rate_roll;
    rate_calibration_pitch += rate_pitch;
    rate_calibration_yaw += rate_yaw;

    delay(1);
  }

  rate_calibration_roll /= 2000;
  rate_calibration_pitch /= 2000;
  rate_calibration_yaw /= 2000;
}

void loop() {
  TakeMeasurement();

  rate_roll -= rate_calibration_roll;
  rate_pitch -= rate_calibration_pitch;
  rate_yaw -= rate_calibration_yaw;

  Serial.println("Roll: " + (String)rate_roll + " °/s; Pitch: " + (String)rate_pitch + " °/s; Yaw: " + (String)rate_yaw + " °/s");
  delay(50);
}
