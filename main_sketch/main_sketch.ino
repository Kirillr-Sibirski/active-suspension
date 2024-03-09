#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"   //Must include for DMP holds firmware hex to push to MPU on init
#include <EEPROM.h>
#include <Servo.h>

// MPU Address for I2C
byte devAddr = 0x68;
MPU6050 mpu(devAddr);

// MPU control/status vars from JRowberg
bool dmpReady = false;  // set true if DMP init was successful
byte mpuIntStatus;   	// holds actual interrupt status byte from MPU
byte devStatus;      	// return status after each device operation (0 = success, !0 = error)
int packetSize;    		// expected DMP packet size (default is 42 bytes)
int fifoCount;     		// count of all bytes currently in FIFO
byte fifoBuffer[64]; 	// FIFO storage buffer
Quaternion q;           // [w, x, y, z] quaternion container

Servo front_right;
Servo front_left;
Servo rear_right;
Servo rear_left;

int front_right_pos;
int front_left_pos;
int rear_right_pos;
int rear_left_pos;

int receiver = 11; // Receiver channel connected
boolean receiverStateWasUp = false;

float roll_angle;  // Tilt angle around the Z-axis
float pitch_angle; // Tilt angle around the Y-axis

// PID Constants
float KP_roll = 2.0; // 1.5
float KI_roll = 0.01;
float KD_roll = 0; // 0.4, 0 because the data from the sensor is noisy
float KP_pitch = 2.0;
float KI_pitch = 0.01;
float KD_pitch = 0; // 0.4
float DT = 0.5;

float prev_error_roll = 0;
float integral_roll = 0;
float prev_error_pitch = 0;
float integral_pitch = 0;

// Modified version of Adafruit BN0555 library to convert Quaternion to world angles the way we need
// The math is a little different here compared to Adafruit's version to work the way I needed for this project
VectorFloat QtoEulerAngle(Quaternion qt) {
  VectorFloat ret;
  double sqw = qt.w * qt.w;
  double sqx = qt.x * qt.x;
  double sqy = qt.y * qt.y;
  double sqz = qt.z * qt.z;

  ret.x = atan2(2.0 * (qt.x * qt.y + qt.z * qt.w), (sqx - sqy - sqz + sqw));
  ret.y = asin(2.0 * (qt.x * qt.z - qt.y * qt.w) / (sqx + sqy + sqz + sqw));  //Adafruit uses -2.0 *..
  ret.z = atan2(2.0 * (qt.y * qt.z + qt.x * qt.w), (-sqx - sqy + sqz + sqw));

  // Added to convert Radian to Degrees
  ret.x = ret.x * 180 / PI;
  ret.y = ret.y * 180 / PI;
  ret.z = ret.z * 180 / PI;
  return ret;
}

// Write a 2 byte word starting at the startAddr provided
void epromWriteWord(int startAddr, int value) {
  EEPROM.update(startAddr, value);
  EEPROM.update(startAddr+1, value >> 8);
}

// Return a 2 byte word read from a starting EEPROM address
int epromReadWord(int startAddr) {
  int value = EEPROM.read(startAddr);
  value = value | (EEPROM.read(startAddr+1) << 8);
  return value;
}

void getCalibration() {
  // Get the saved calibration values from EEPROM and update MPU
  // Address Always starts at 0 ends at 11, 2 bytes per axis

  // Future Check if we have anything saved (look for all FF)
  // if not assume calibration not run and skip setting just MPU default
  // Eventually prompt user to calibrate! if there's enough space left in memory of this sketch!!!
    mpu.setXAccelOffset(epromReadWord(0));
    mpu.setYAccelOffset(epromReadWord(2));
    mpu.setZAccelOffset(epromReadWord(4));
    mpu.setXGyroOffset(epromReadWord(6));
    mpu.setYGyroOffset(epromReadWord(8));
    mpu.setZGyroOffset(epromReadWord(10)); // last address read would be 11 decimal in eeprom    
}

void setCalibration() {
  // Run DMP auto calibration and then get those values and save to EEprom
  // This should only be called when Auto Calibrate option is selected
  // to preserve EEPROM life use update instead of write

  // Run autocalibration 6 times
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  // Get the final values that were saved and move them to EEPROM
  int Data[3];
  // Accel Offsets
  I2Cdev::readWords(devAddr, 0x06, 3, (int *)Data);
  epromWriteWord(0,Data[0]);
  epromWriteWord(2,Data[1]);
  epromWriteWord(4,Data[2]);
  // Gyro Offsets
  I2Cdev::readWords(devAddr, 0x13, 3, (int *)Data);
  epromWriteWord(6,Data[0]);
  epromWriteWord(8,Data[1]);
  epromWriteWord(10,Data[2]); // Last byte written is eeprom address 11 decimal 
}

void defaultPositions() {
  front_right_pos = 90;
  front_left_pos = 90;
  rear_right_pos = 90;
  rear_left_pos = 90;
  front_right.write(front_right_pos);
  front_left.write(front_left_pos);
  rear_right.write(rear_right_pos);
  rear_left.write(rear_left_pos);
}

void setup() {
  // From JRowber sample to setup MPU6050 connection
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);
  // Init MPU6050
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  Serial.print("Init MPU");

  pinMode(receiver, INPUT_PULLUP);

  // Get stored EEPROM Calibration values and send to MPU
  // Otherwise default to predefined and display Calibration needed!

  front_right.attach(3); 
  front_left.attach(2); 
  rear_right.attach(4);
  rear_left.attach(5);

  defaultPositions();

  delay(1000); // make sure that servos are centered

  // setCalibration();
  // delay(100);
  getCalibration();

  // make sure it worked - Because we are pushing firmware on startup of DMP
  // we need to check that everything actually went as planned devStatus 0 is success.
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

  } else {
    Serial.print("IMU fail");
  }
}

void loop() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    boolean receiverStateUp = digitalRead(receiver);
    if (receiverStateUp && !receiverStateWasUp) { // Check if transmitter button is clicked
      delay(100); // You have to hold button on the receiver for some time before it is read as clicked
      receiverStateUp = digitalRead(receiver);
      if(!receiverStateUp) {
        defaultPositions();
        delay(1000); // make sure that servos are centered
        setCalibration();
        delay(100);
      }
    }
    receiverStateWasUp = receiverStateUp;
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    // Convert Quaternion to Euler angles
    VectorFloat euler_angles = QtoEulerAngle(q);
    roll_angle = euler_angles.z;
    pitch_angle = euler_angles.y;

    // Calculate PID terms for roll stabilization
    float error_roll = 0.0 - roll_angle;  // Desired angle is 0 degrees
    integral_roll = integral_roll + error_roll * DT;
    float derivative_roll = (error_roll - prev_error_roll) / DT;
    prev_error_roll = error_roll; // Save current errors for the next iteration

    // Calculate PID terms for pitch stabilization
    float error_pitch = 0.0 - pitch_angle;  // Desired angle is 0 degrees
    integral_pitch = integral_pitch + error_pitch * DT;
    float derivative_pitch = (error_pitch - prev_error_pitch) / DT;
    prev_error_pitch = error_pitch; // Save current errors for the next iteration

    if (abs(roll_angle) < 1.0 && abs(pitch_angle) < 1.0) {
      // Reset integral terms when the system is close to the horizontal position
      integral_roll = 0;
      integral_pitch = 0;
    }

    // Calculate control output for roll
    float control_output_roll = KP_roll * error_roll + KI_roll * integral_roll + KD_roll * derivative_roll;
    // Calculate control output for pitch
    float control_output_pitch = KP_pitch * error_pitch + KI_pitch * integral_pitch + KD_pitch * derivative_pitch;

    // Update servo positions based on control outputs
    adjustServoPositions(control_output_roll, control_output_pitch);

  }
}
void adjustServoPositions(float control_output_roll, float control_output_pitch) {
//  float deadband_size = 1.0;
//
//  if(abs(control_output_roll) < deadband_size && abs(control_output_pitch) < deadband_size) {
//    return;
//  }
  
  // Modify servo positions based on the control outputs for roll and pitch
  // Adjust the following conditions based on your servo orientation and desired behavior
  Serial.println(control_output_roll);
  Serial.println(control_output_pitch);
  delay(10); // So the system doesn't go to insane
  front_right_pos = 90 + control_output_roll + control_output_pitch;
  front_left_pos = 90 - control_output_roll + control_output_pitch;
  rear_right_pos = 90 - control_output_roll + control_output_pitch;
  rear_left_pos = 90 + control_output_roll + control_output_pitch;
//  Serial.println("Front right position: ");
//  Serial.println(front_right_pos);
  
  // Clip servo positions to a reasonable range
  front_right_pos = constrain(front_right_pos, 40, 140);
  front_left_pos = constrain(front_left_pos, 40, 140);
  rear_right_pos = constrain(rear_right_pos, 40, 140);
  rear_left_pos = constrain(rear_left_pos, 40, 140);

  // Update servo positions
  front_right.write(front_right_pos);
  front_left.write(front_left_pos);
  rear_right.write(rear_right_pos);
  rear_left.write(rear_left_pos);
}
