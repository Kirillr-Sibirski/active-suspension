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

int front_right_pos = 90;
int front_left_pos = 90;
int rear_right_pos = 90;
int rear_left_pos = 90;

int ch3;
int ch4;

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
  ret.z = atan2( 2.0 * (qt.y * qt.z + qt.x * qt.w), (-sqx - sqy + sqz + sqw));

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

  // Get stored EEPROM Calibration values and send to MPU
  // Otherwise default to predefined and display Calibration needed!

  front_right.attach(3); 
  front_left.attach(2); 
  rear_right.attach(4);
  rear_left.attach(5);

  front_right.write(front_right_pos);
  front_left.write(front_left_pos);
  rear_right.write(rear_right_pos);
  rear_left.write(rear_left_pos);

  delay(1000); // make sure that servos are centered

  setCalibration();
  delay(100);
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

  pinMode(10, INPUT); // Set our input pins as such
  pinMode(11, INPUT); // Set our input pins as such
}

// MAIN PROGRAM LOOP!!
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // Get the Quaternion values from DMP buffer
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    // Calc angles converting Quaternion to Euler this was giving more stable acurate results compared to
    // getting Euler directly from DMP. I think Quaternion conversion takes care of gimble lock.
    VectorFloat ea = QtoEulerAngle(q);

    //DEBUG ONLY COMMENT OUT UNLESS NEEDED
    // Serial.print("quat\t");
    // Serial.print(ea.x);
    // Serial.print("\t");
    // Serial.print(ea.y);
    // Serial.print("\t");
    // Serial.print(ea.z);
    // Serial.println("\t"); 
//    ch3 = pulseIn(11, HIGH, 25000);
//    ch4 = pulseIn(10, HIGH, 25000);
//    Serial.println("Ch3: ");
//    Serial.println(ch3);
//    Serial.println(ch4);
//    if(ch4 >= 2000) {
//      front_right_pos = 90;
//      front_left_pos = 90;
//      rear_right_pos = 90;
//      rear_left_pos = 90;
//      front_right.write(front_right_pos);
//      front_left.write(front_left_pos);
//      rear_right.write(rear_right_pos);
//      rear_left.write(rear_left_pos);
//      delay(1000);
//      setCalibration();
//      delay(100);
//      getCalibration();
//    }


    if(ea.z < -0.5){ // car is leaning forward
      if(front_right_pos <= 140) {
        front_right_pos = front_right_pos+1;
        front_right.write(front_right_pos);
      } else if(rear_right_pos >= 40){
        rear_right_pos = rear_right_pos-1;
        rear_right.write(rear_right_pos);
      }
      if(front_left_pos >= 40) {
        front_left_pos=front_left_pos-1;
        front_left.write(front_left_pos);
      } else if(rear_left_pos <= 140) {
        rear_left_pos=rear_left_pos+1;
        rear_left.write(rear_left_pos);
      }
    } else if(ea.z > 0.5){ // car is leaning backward
      if(front_right_pos >= 40) {
        front_right_pos = front_right_pos-1;
        front_right.write(front_right_pos);
      } else if(rear_right_pos <= 140) {
        rear_right_pos = rear_right_pos+1;
        rear_right.write(rear_right_pos);
      }
      if(front_left_pos <= 140) {
        front_left_pos=front_left_pos+1;
        front_left.write(front_left_pos);
      } else if(rear_left_pos >= 40) {
        rear_left_pos=rear_left_pos-1;
        rear_left.write(rear_left_pos);
      }
    }

//    if(ea.y < -1){ // car is leaning right
//      if(front_right_pos <= 140) { // We raise the suspension as much as possible
//        front_right_pos = front_right_pos+1;
//        front_right.write(front_right_pos);
//      } else if(front_left_pos <= 140){ // If it is still not horizontal -> we lower the other shock
//        front_left_pos = front_left_pos+1;
//        front_left.write(front_left_pos);
//      }
//      if(rear_right_pos <= 140) { // We raise the suspension (equally), although, this may poise a problem e.g. it may just get into a loop of vibration when ea.y asks the servo to be raised and the ea.z to lower it. Potential solution could be that we track both y and z coordinates and adjust individual shocks. 
//        rear_right_pos=rear_right_pos+1;
//        rear_right.write(rear_right_pos);
//      } else if(rear_left_pos <= 140) {
//        rear_left_pos=rear_left_pos+1;
//        rear_left.write(rear_left_pos);
//      }
//    } else if(ea.y > 1){ // car is leaning left
//      if(front_right_pos >= 40) {
//        front_right_pos = front_right_pos-1;
//        front_right.write(front_right_pos);
//      } else if(front_left_pos >= 40){
//        front_left_pos = front_left_pos-1;
//        front_left.write(front_left_pos);
//      }
//      if(rear_right_pos >= 40) {
//        rear_right_pos=rear_right_pos-1;
//        rear_right.write(rear_right_pos);
//      } else if(rear_left_pos >= 40) {
//        rear_left_pos=rear_left_pos-1;
//        rear_left.write(rear_left_pos);
//      }
//    } 

  }
}
