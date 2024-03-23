// Define PWM pins that are allocated for the phase shifters
const int pwmPin_P1 = 4; // Phase shifter #1
const int pwmPin_P2 = 5; // Phase shifter #2
const int pwmPin_P3 = 6; // Phase shifter #3
const int pwmPin_P4 = 7; // Phase shifter #4
const int pwmPin_P5 = 8; // Phase shifter #5
const int pwmPin_P6 = 9; // Phase shifter #6
const int pwmPin_P7 = 10; // Phase shifter #7
const int pwmPin_P8 = 11; // Phase shifter #8

// Define information about TARGET 1
const float desired_voltage_P1_T1 = 5; // Voltage for Phase shifter #1
const float desired_voltage_P2_T1 = 5; // Voltage for Phase shifter #2
const float desired_voltage_P3_T1 = 5; // Voltage for Phase shifter #3
const float desired_voltage_P4_T1 = 5; // Voltage for Phase shifter #4
const float desired_voltage_P5_T1 = 5; // Voltage for Phase shifter #5
const float desired_voltage_P6_T1 = 5; // Voltage for Phase shifter #6
const float desired_voltage_P7_T1 = 5; // Voltage for Phase shifter #7
const float desired_voltage_P8_T1 = 5; // Voltage for Phase shifter #8

// Define information about TARGET 2
const float desired_voltage_P1_T2 = 5; // Voltage for Phase shifter #1
const float desired_voltage_P2_T2 = 5; // Voltage for Phase shifter #2
const float desired_voltage_P3_T2 = 5; // Voltage for Phase shifter #3
const float desired_voltage_P4_T2 = 5; // Voltage for Phase shifter #4
const float desired_voltage_P5_T2 = 5; // Voltage for Phase shifter #5
const float desired_voltage_P6_T2 = 5; // Voltage for Phase shifter #6
const float desired_voltage_P7_T2 = 5; // Voltage for Phase shifter #7
const float desired_voltage_P8_T2 = 5; // Voltage for Phase shifter #8

// Returns analogwrite value for desired voltages in range: 1V - 15V
int compute_analog_level(float desired_voltage) {
  float analog_level = (desired_voltage - 15) / -0.0547;
  return int(analog_level);
}

// Voltage control for given desired voltages
void voltage_control(float voltage_1, float voltage_2, float voltage_3, float voltage_4, float voltage_5, float voltage_6, float voltage_7, float voltage_8) {

  // Organize desired voltages and their corresponding PWM pins
  float voltages[8] = {voltage_1, voltage_2, voltage_3, voltage_4, voltage_5, voltage_6, voltage_7, voltage_8};
  int PWM_pins[8] = {pwmPin_P1, pwmPin_P2, pwmPin_P3, pwmPin_P4, pwmPin_P5, pwmPin_P6, pwmPin_P7, pwmPin_P8};

  for (int i = 0; i < 8; i++) {

    // Multiplexing
    if (voltages[i] >= 1) {
      analogWrite(PWM_pins[i], compute_analog_level(voltages[i])); 
    } else if ((voltages[i] < 1) && (voltages[i] >= 0)) {
      analogWrite(PWM_pins[i], int(voltages[i] * 51));
    } else {
      analogWrite(PWM_pins[i], 0);
    }

  }
}

void setup() {
  // PWM pins for phase shifter control
  pinMode(pwmPin_P1, OUTPUT);
  pinMode(pwmPin_P2, OUTPUT);
  pinMode(pwmPin_P3, OUTPUT);
  pinMode(pwmPin_P4, OUTPUT);
  pinMode(pwmPin_P5, OUTPUT);
  pinMode(pwmPin_P6, OUTPUT);
  pinMode(pwmPin_P7, OUTPUT);
  pinMode(pwmPin_P8, OUTPUT);
}

void loop() {
  // Switch between two targets, monitoring time = 10 seconds

  //Target 1
  voltage_control(desired_voltage_P1_T1, desired_voltage_P2_T1, desired_voltage_P3_T1, desired_voltage_P4_T1, desired_voltage_P5_T1, desired_voltage_P6_T1, desired_voltage_P7_T1, desired_voltage_P8_T1);
  delay(10000);

  // Target 2
  voltage_control(desired_voltage_P1_T2, desired_voltage_P2_T2, desired_voltage_P3_T2, desired_voltage_P4_T2, desired_voltage_P5_T2, desired_voltage_P6_T2, desired_voltage_P7_T2, desired_voltage_P8_T2);
  delay(10000);
}
