#include <PulsePosition.h>
PulsePositionInput receiver_input(RISING);

float receiver_value[] = {0, 0, 0, 0, 0, 0, 0, 0}; // {Roll, Pitch, Throttle, Yaw, VRA, VRB, idk, idk}
int channel_number = 0;

void read_receiver() {
  channel_number = receiver_input.available();
  if (channel_number > 0) {
    for (int i = 1; i <= channel_number; i++) {
      receiver_value[i-1] = receiver_input.read(i);
    }
  }
}

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT); 
  digitalWrite(13, HIGH);
  receiver_input.begin(14);

  analogWriteFrequency(1, 250); // Each period is 4000 microseconds
  analogWriteResolution(12); // 12 bit resolution for finer control compare to 8 bit of Teensy
  delay(250);

  // Check for accidental start (i.e if the throttle is already high). Throttle's lowest position is between 1020 and 1050.
  if(receiver_value[2] < 1020 || receiver_value[2] > 1050) { 
    read_receiver();
    delay(4);
  }
}

void loop() {
  read_receiver();
  analogWrite(1, 1.042 * receiver_value[2]); // Write the throttle value to pin 1
}
