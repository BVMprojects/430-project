#include <Wire.h>
#include <SimpleFOC.h>

// MPU6050 Registers and Constants
#define MPU6050_ADDR      0x68
#define ACCEL_CONFIG      0x1C
#define GYRO_CONFIG       0x1B
#define ACCEL_YOUT_H      0x3D
#define ACCEL_ZOUT_H      0x3F
#define GYRO_XOUT_H       0x43
#define PWR_MGMT_1        0x6B
#define Gyro_amount       0.996  // Complementary filter factor

// Global variables
int16_t AcY, AcZ, GyX;
float robot_angle = 0, Acc_angle = 0;
float GyX_offset = 0;
bool vertical = false;

// Motor setup for SimpleFOC
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(3, 5, 6);  // Adjust pins for motor driver

// Complementary filter constants
const float Gyro_conversion = 6.07968E-5;
const float Degree_conversion = 57.2958;

// Function to write to MPU6050 registers
void writeToMPU(byte address, byte value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
}

// MPU6050 setup
void setupMPU6050() {
  Wire.begin();
  writeToMPU(PWR_MGMT_1, 0);         // Wake up MPU6050
  writeToMPU(ACCEL_CONFIG, 0);       // Set accelerometer range
  writeToMPU(GYRO_CONFIG, 0);        // Set gyro range
  delay(100);
  
  // Gyro calibration (offset calculation)
  for (int i = 0; i < 1024; i++) {
    readMPU6050Data();
    GyX_offset += GyX;
    delay(5);
  }
  GyX_offset /= 1024;
}

// Function to read MPU6050 data
void readMPU6050Data() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_YOUT_H);                  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);  
  
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  
  // Subtract gyro offset
  GyX -= GyX_offset;
}

// Balance calculation using complementary filter
void calculateBalance() {
  // Convert gyro X value to angle and filter
  robot_angle += -GyX * Gyro_conversion;
  Acc_angle = atan2(AcY, -AcZ) * Degree_conversion;
  robot_angle = robot_angle * Gyro_amount + Acc_angle * (1 - Gyro_amount);
}

// PID controller for motor
float controlMotor(float angle_error) {
  static float previous_error = 0;
  float kp = 0.6, kd = 0.05;  // Adjust PID constants as needed
  float control_signal = kp * angle_error + kd * (angle_error - previous_error);
  previous_error = angle_error;
  
  return constrain(control_signal, -12, 12); // Limit motor voltage
}

void setup() {
  Serial.begin(115200);
  
  setupMPU6050();
  
  // Motor initialization
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.controller = ControlType::voltage;
  motor.init();
  motor.initFOC();
}

void loop() {
  readMPU6050Data();
  calculateBalance();
  
  // If tilt angle exceeds threshold, stop balancing
  if (abs(robot_angle) > 10) {
    motor.move(0);
    vertical = false;
    return;
  }
  
  vertical = true;
  float motor_voltage = controlMotor(robot_angle);
  motor.move(motor_voltage);
}
