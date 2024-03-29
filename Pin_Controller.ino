int DigitalControlPins[8] = {22, 24, 26, 28, 30, 32, 34, 36};
int PWMControlPins[8] = {4, 5, 6, 7, 8, 9, 10, 11};

void setup() {
  // Define MUX control pins
  for (int i = 0; i < 8; i++) {
    pinMode(DigitalControlPins[i], OUTPUT);
  }
  Serial.begin(9600); // Baud rate
}

void loop() {
  if (Serial.available() > 0) {

    String receivedData = Serial.readStringUntil('\n');

    int command_type, data;
    sscanf(receivedData.c_str(), "%d %d", &command_type, &data);

    // Establish MUX control
    if (command_type < 8) { // Check if command_type is within MUX pin range
      for (int j = 0; j < 8; j++) {
        if (command_type == j) {
          if (data == 1) {
            digitalWrite(DigitalControlPins[j], HIGH);
          }
          else {
            digitalWrite(DigitalControlPins[j], LOW);
          }
        }
      }
    }

    // Establish PWM control
    for (int k = 8; k < 16; k++) {
      if (command_type == k) {
        analogWrite(PWMControlPins[k - 8], data);
      }
    }
  }
}
