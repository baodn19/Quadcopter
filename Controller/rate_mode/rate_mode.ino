#include <Wire.h>
#include <PulsePosition.h>

// Gyroscope
float yaw_rate, pitch_rate, roll_rate;
float yaw_calibration_rate, pitch_calibration_rate, roll_calibration_rate;
int rate_calibration_number;

// Receiver
PulsePositionInput receiver_input(RISING);
float receiver_value[] = {0, 0, 0, 0, 0, 0, 0, 0}; // {Roll, pitch, throttle, yaw, aux1, aux2, aux3, aux4}
int channel_number = 0;

// Battery
float voltage, current, remaining_battery_percentage, initial_battery_percentage;
float current_consumed = 0;
float default_battery_percentage = 1100;

uint32_t loop_timer;

// PID
float desired_yaw_rate, desired_pitch_rate, desired_roll_rate;
float yaw_error, pitch_error, roll_error;
float previous_yaw_error, previous_pitch_error, previous_roll_error;
float previous_yaw_I, previous_pitch_I, previous_roll_I;
float PID_return[] = {0, 0, 0}; // {Input, previous error, previous I}
float throttle_input, yaw_input, pitch_input, roll_input;
float motor_1_input, motor_2_input, motor_3_input, motor_4_input;
float yaw_P = 2, pitch_P = 0.6, roll_P = pitch_P;
float yaw_I = 12, pitch_I = 3.5, roll_I = pitch_I;
float yaw_D = 0, pitch_D = 0.03, roll_D = pitch_D;

/*
Description: This function reads the battery voltage and current from the analog pins.
*/
void BatteryVoltage(void) {
    voltage = (float)analogRead(15) * (251 / 15810);
    current = (float)analogRead(21) * (140 / 1581);    
}

/*
Description: This function reads the receiver input values and stores them in the receiver_value array.
*/
void ReadReceiver(void) {
    channel_number = receiver_input.available();
    if (channel_number > 0) {
        for (int i = 1; i <= channel_number; i++) {
            receiver_value[i-1] = receiver_input.read(i);
        }
    }
}

/*
Description: This function reads the gyroscope values and calculates the yaw, pitch, and roll rates.
*/
void GyroSignal(void) {
    // DLPF (Register 26)
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05); // 10Hz
    Wire.endTransmission();

    // Gyro LSB (Register 27)
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x08); // 500 degrees/s; LSB = 65.5 LSB/degree/s
    Wire.endTransmission();

    // Measurements (Register 67 - 72)
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);

    // Calculate gyro values
    int16_t gyro_x = (Wire.read() << 8) | Wire.read();
    int16_t gyro_y = (Wire.read() << 8) | Wire.read();
    int16_t gyro_z = (Wire.read() << 8) | Wire.read();
    yaw_rate = (float)gyro_z / 65.5;
    pitch_rate = (float)gyro_y / 65.5;
    roll_rate = (float)gyro_x / 65.5;
}

/*
Description: This function calculates the PID output based on the error, proportional, integral, and derivative gains.
*/
void PIDEquation(float error, float P, float I, float D, float previous_error, float previous_I) {
    // Calculate PID terms
    float P_term = error * P;
    float I_term = previous_I + I * (error + previous_error) * 0.004 / 2;
    float D_term = (error - previous_error) * D / 0.004;
    float PID_output = P_term + I_term + D_term;

    // Prevent windup
    if (I_term < -400) {
        I_term = -400;
    } else if (I_term > 400) {
        I_term = 400;
    }
    if (PID_output < -400) {
        PID_output = -400;
    } else if (PID_output > 400) {
        PID_output = 400;
    }

    // Return the PID output and update previous values
    PID_return[0] = PID_output;
    PID_return[1] = error;
    PID_return[2] = I_term;
}

/*
Description: This function resets the PID controller's previous error and integral values to zero.
*/
void ResetPID(void) {
    previous_yaw_error = 0;
    previous_pitch_error = 0;
    previous_roll_error = 0;
    previous_yaw_I = 0;
    previous_pitch_I = 0;
    previous_roll_I = 0;
}

void setup() {
    pinMode(5, OUTPUT); // Red LED
    digitalWrite(5, HIGH);
    pinMode(13, OUTPUT); // Teensy LED
    digitalWrite(13, HIGH);

    Wire.setClock(400000); // Set I2C clock to 400kHz
    Wire.begin(); // Initialize I2C communication
    delay(250);

    Wire.beginTransmission(0x68); // Start communication with MPU6050
    Wire.write(0x6B); // Power management register
    Wire.write(0x00); // Wake up the MPU6050
    Wire.endTransmission();

    // Gyro calibration
    for (rate_calibration_number = 0; rate_calibration_number < 2000; i++) {
        GyroSignal();
        yaw_calibration_rate += yaw_rate;
        pitch_calibration_rate += pitch_rate;
        roll_calibration_rate += roll_rate;
        delay(1);
    }
    yaw_calibration_rate /= 2000;
    pitch_calibration_rate /= 2000;
    roll_calibration_rate /= 2000;

    // Motor refresh rate
    analogWriteFrequency(1, 250);
    analogWriteFrequency(2, 250);
    analogWriteFrequency(3, 250);
    analogWriteFrequency(4, 250);
    analogWriteResolution(12); // Set resolution to 12 bits

    // Mark that setup is complete
    pinMode(6, OUTPUT); // Green LED
    digitalWrite(6, HIGH);

    // Check battery percentage at start, turn off Red LED if battery is sufficient
    BatteryVoltage();
    if (voltage > 8.3) {
        digitalWrite(5, LOW);
        initial_battery_percentage = default_battery_percentage;
    } else if (voltage < 7.5) {
        initial_battery_percentage = default_battery_percentage * 35 / 100;
    } else {
        digitalWrite(5, LOW);
        initial_battery_percentage = default_battery_percentage * (82 * voltage - 580) / 100;
    }

    // Check if throttle is at minimum
    receiver_input.begin(14);
    while(receiver_value[2] > 1020 && receiver_value[2] < 1050) {
        ReadReceiver();
        delay(10);
    }

    loop_timer = micros();
}

void loop() {
    // Actual yaw, pitch, and roll rates
    GyroSignal();
    yaw_rate -= yaw_calibration_rate;
    pitch_rate -= pitch_calibration_rate;
    roll_rate -= roll_calibration_rate;

    // Desired yaw, pitch, and roll rates
    ReadReceiver();
    desired_pitch_rate = 0.15 * receiver_value[1] - 225;
    desired_yaw_rate = 0.15 * receiver_value[3] - 225;
    desired_roll_rate = 0.15 * receiver_value[0] - 225;
    throttle_input = receiver_value[2];

    // PID calculations
    yaw_error = desired_yaw_rate - yaw_rate;
    pitch_error = desired_pitch_rate - pitch_rate;
    roll_error = desired_roll_rate - roll_rate;
    PIDEquation(yaw_error, yaw_P, yaw_I, yaw_D, previous_yaw_error, previous_yaw_I);
    yaw_input = PID_return[0];
    previous_yaw_error = PID_return[1];
    previous_yaw_I = PID_return[2];
    PIDEquation(pitch_error, pitch_P, pitch_I, pitch_D, previous_pitch_error, previous_pitch_I);
    pitch_input = PID_return[0];
    previous_pitch_error = PID_return[1];
    previous_pitch_I = PID_return[2];
    PIDEquation(roll_error, roll_P, roll_I, roll_D, previous_roll_error, previous_roll_I);
    roll_input = PID_return[0];
    previous_roll_error = PID_return[1];
    previous_roll_I = PID_return[2];

    // Limit throttle input to 80%
    if (throttle_input > 1800) throttle_input = 1800;

    // Calculate motor inputs
    motor_1_input = 1.024 * (throttle_input - roll_input - pitch_input - yaw_input);
    motor_2_input = 1.024 * (throttle_input - roll_input + pitch_input + yaw_input);
    motor_3_input = 1.024 * (throttle_input + roll_input + pitch_input - yaw_input);
    motor_4_input = 1.024 * (throttle_input + roll_input - pitch_input + yaw_input);

    // Limit motor inputs to 1999
    if (motor_1_input > 1999) motor_1_input = 1999;
    if (motor_2_input > 1999) motor_2_input = 1999;
    if (motor_3_input > 1999) motor_3_input = 1999;
    if (motor_4_input > 1999) motor_4_input = 1999;

    // Keep the motors running to prevent motor stopping mid-flight
    int throttle_idle = 1180;
    if (motor_1_input < throttle_idle) motor_1_input = throttle_idle;
    if (motor_2_input < throttle_idle) motor_2_input = throttle_idle;
    if (motor_3_input < throttle_idle) motor_3_input = throttle_idle;
    if (motor_4_input < throttle_idle) motor_4_input = throttle_idle;

    // Stop all motors if controller is at minimum throttle
    int throttle_stop = 1000;
    if (throttle_input < 1050) {
        motor_1_input = throttle_stop;
        motor_2_input = throttle_stop;
        motor_3_input = throttle_stop;
        motor_4_input = throttle_stop;

        ResetPID();
    }
    
    // Send commands to motors
    analogWrite(1, motor_1_input);
    analogWrite(2, motor_2_input);
    analogWrite(3, motor_3_input);
    analogWrite(4, motor_4_input);

    // Track battery percentage
    BatteryVoltage();
    current_consumed += current * (5 / 18) * 0.004;
    remaining_battery_percentage = (initial_battery_percentage - current_consumed) / default_battery_percentage * 100;
    if (remaining_battery_percentage <= 30) {
        digitalWrite(5, HIGH); // Turn on Red LED if battery is low
    } else {
        digitalWrite(5, LOW); // Turn off Red LED if battery is sufficient
    }

    while (micros() - loop_timer < 4000);
    loop_timer = micros(); // Update loop timer
}