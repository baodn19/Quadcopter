#include <Wire.h>
// Trimming values for altitude measurement
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4;
int16_t dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

// Altitude measurement variables
float altitude, initial_altitude;
int rate_calibration_number;

void BarometerSignal() {
  // Measure raw data from the barometer
  Wire.beginTransmission(0x76);
  Wire.write(0xF7); // Register address for pressure and temperature data
  Wire.endTransmission();
  Wire.requestFrom(0x76, 6); // Request 6 bytes of data

  // Read the pressure and temperature data
  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();

  // Construct raw data values
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);
  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);

  // Calculate t_fine
  signed long int var1, var2, t_fine;
  var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
  t_fine = var1 + var2;

  // Calculate calibrated pressure
  unsigned long int p;
  var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * ((signed long int)dig_P6);
  var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + ((((signed long int)dig_P2) * var1) >>1 )) >>18;
  var1 =((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
  if (var1 == 0)
  {
  p = 0; // avoid exception caused by division by zero
  }
  p = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (p < 0x80000000)
  {
  p = (p << 1) / ((unsigned long int)var1);
  }
  else
  {
  p = (p / (unsigned long int)var1) * 2;
  }
  var1 = (((signed long int)dig_P9) * ((signed long int)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((signed long int)(p >> 2)) * ((signed long int)dig_P8)) >> 13;
  p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7) >> 4));

  // Calculate altitude based on pressure
  double pressure = (double)p / 100; // Convert to hPa
  altitude = (1.0 - pow(pressure / 1013.25, 1 / 5.255)) * 44330 * 100; // Altitude in cm
}

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000); // Set I2C clock speed to 400kHz
  Wire.begin();

  // Mode and oversampling settings
  Wire.beginTransmission(0x76);
  Wire.write(0xF4); // Control register address
  Wire.write(0x57); // Set mode to normal, pressure oversampling x16, temperature oversampling x2
  Wire.endTransmission();

  // IIR filter and standby time settings
  Wire.beginTransmission(0x76);
  Wire.write(0xF5); // Configuration register address
  Wire.write(0x14); // Set IIR filter to 16, standby time to 0.5 ms
  Wire.endTransmission();

  // Read trimming values from the barometer
  int8_t data[24];
  int i = 0;
  Wire.beginTransmission(0x76);
  Wire.write(0x88); // Register address for trimming values
  Wire.endTransmission();
  Wire.requestFrom(0x76, 24); // Request 24 bytes of trimming
  while(Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  
  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];

  // Calculate initial altitude
  for (rate_calibration_number = 0; rate_calibration_number < 2000; rate_calibration_number++) {
    BarometerSignal();
    initial_altitude += altitude;
    delay(1);
  }
  initial_altitude /= 2000; // Average the altitude over 2000 readings
}

void loop() {
  BarometerSignal();
  altitude -= initial_altitude; // Adjust altitude based on initial measurement
  Serial.print("Altitude: ");
  Serial.print(altitude);
  Serial.println(" cm");
  delay(50); // Delay for readability
}
