#include <Wire.h>

float rate_roll, rate_pitch, rate_yaw; // For one measurement
float rate_calibration_roll = 0; 
float rate_calibration_pitch = 0; 
float rate_calibration_yaw = 0; 

float acceleration_X = 0;
float acceleration_Y = 0;
float acceleration_Z = 0;
float angle_roll, angle_pitch;

float kalman_angle_roll = 0;
float uncertainty_kalman_angle_roll = 2*2;
float kalman_angle_pitch = 0;
float uncertainty_kalman_angle_pitch = 2*2;
float kalman_1d_output[] = {0, 0}; // Kalman filter state vector

uint32_t loop_timer;

void kalman_1d(float kalman_state, float kalman_uncertainty, float measured_angle, float measured_rate) {
  kalman_state = kalman_state + measured_rate * 0.004; // 0.004 is the time step in seconds
  kalman_uncertainty = kalman_uncertainty + 0.004 * 0.004 * 4 * 4; // 4 is the rate standard deviation
  float kalman_gain = kalman_uncertainty / (kalman_uncertainty + 3 * 3);

  kalman_state = kalman_state + kalman_gain * (measured_angle - kalman_state);
  kalman_uncertainty = (1 - kalman_gain) * kalman_uncertainty;

  kalman_1d_output[0] = kalman_state;
  kalman_1d_output[1] = kalman_uncertainty;
}

void TakeMeasurement(void) {
  // Configure accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10); // Set full scale range to ±8g
  Wire.endTransmission();

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

  // Allocate memory for rate
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  // Measure rate
  int16_t rate_X = Wire.read() << 8 | Wire.read();
  int16_t rate_Y = Wire.read() << 8 | Wire.read();
  int16_t rate_Z = Wire.read() << 8 | Wire.read();
  rate_roll = (float)rate_X / 65.5;
  rate_pitch = (float)rate_Y / 65.5;
  rate_yaw = (float)rate_Z / 65.5;

  // Allocate memory for acceleration
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  // Measure acceleration
  int16_t measured_acceleration_X = Wire.read() << 8 | Wire.read();
  int16_t measured_acceleration_Y = Wire.read() << 8 | Wire.read();
  int16_t measured_acceleration_Z = Wire.read() << 8 | Wire.read();
  acceleration_X = ((float)measured_acceleration_X / 4096.0) - 0.03; 
  acceleration_Y = ((float)measured_acceleration_Y / 4096.0) + 0.01;
  acceleration_Z = ((float)measured_acceleration_Z / 4096.0) + 0.03;
  angle_roll = atan(acceleration_Y / sqrt(acceleration_X * acceleration_X + acceleration_Z * acceleration_Z)) * 180.0 / 3.14159265358979323846;
  angle_pitch = atan(-acceleration_X / sqrt(acceleration_Y * acceleration_Y + acceleration_Z * acceleration_Z)) * 180.0 / 3.14159265358979323846;
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

  loop_timer = micros();
}

void loop() {
  TakeMeasurement();

  rate_roll -= rate_calibration_roll;
  rate_pitch -= rate_calibration_pitch;
  rate_yaw -= rate_calibration_yaw;

  // Serial.println("Acceleration X [g]: " + (String)acceleration_X + "; Acceleration Y [g]: " + (String)acceleration_Y + "; Acceleration Z [g]: " + (String)acceleration_Z);

  kalman_1d(kalman_angle_roll, uncertainty_kalman_angle_roll, angle_roll, rate_roll);
  kalman_angle_roll = kalman_1d_output[0];
  uncertainty_kalman_angle_roll = kalman_1d_output[1];
  kalman_1d(kalman_angle_pitch, uncertainty_kalman_angle_pitch, angle_pitch, rate_pitch);
  kalman_angle_pitch = kalman_1d_output[0];
  uncertainty_kalman_angle_pitch = kalman_1d_output[1];

  Serial.print(kalman_angle_roll);
  Serial.print("\t");
  Serial.println(kalman_angle_pitch);

  while(micros() - loop_timer < 4000);
  loop_timer = micros();
}