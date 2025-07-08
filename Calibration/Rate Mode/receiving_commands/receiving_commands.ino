#include <PulsePosition.h>
PulsePositionInput receiver_input(RISING);

float receiver_value[] = {0, 0, 0, 0, 0, 0, 0, 0};
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
  receiver_input.begin(14); // Pin 14 for receiver input
}

void loop() {
  read_receiver();
  Serial.println("Channel number: " + String(channel_number)
                 + "; Throttle [μs]: " + String(receiver_value[0])
                 + "; Pitch [μs]: " + String(receiver_value[1])
                 + "; Yaw [μs]: " + String(receiver_value[2])
                 + "; Roll [μs]: " + String(receiver_value[3]));
  delay(50); // Adjust the delay as needed
}
