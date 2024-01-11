/* DesignBuildDestroy.com - Digital Level 1.0
   Written by DesignBuildDestroy (DBD) 2020

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Required Libraries
   * Jeff Rowberg's MPU6050 library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
   * Jeff Rowberg's I2Cdev library: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev
   * Adafruit SSD1306 OLED library: https://github.com/adafruit/Adafruit_SSD1306
   * Adafruit GFX library: https://github.com/adafruit/Adafruit-GFX-Library
   * Adafruit font from GFX library: https://github.com/adafruit/Adafruit-GFX-Library/blob/master/Fonts/FreeMono9pt7b.h
   
 */

#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"   //Must include for DMP holds firmware hex to push to MPU on init
#include <EEPROM.h>

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

  Serial.begin(115200); // Only needed for Debug otherwise can comment out

  // Init MPU6050
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // Get stored EEPROM Calibration values and send to MPU
  // Otherwise default to predefined and display Calibration needed!
  getCalibration();
  // Might need to setCalibration(); // This runs the actual calibration

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
    Serial.print("INU failed");
  }
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
    /*  Serial.print("quat\t");
      Serial.print(ea.x);
      Serial.print("\t");
      Serial.print(ea.y);
      Serial.print("\t");
      Serial.print(ea.z);
      Serial.println("\t"); 
    */
    
    float angVal = 0;

    // Figure out how to display the data on OLED at the right rotation
    // Like flipping a phone around rotating the display - trial and error...took a while
    // TOP IS TOP means TOP of OLED is the TOP of the screen, RIGHT IS TOP Right side of OLED is now the top, etc.

    // TOP IS TOP angling Right side (LEVEL)
    if (ea.x > 0 && ea.y > 35 && ea.y <= 90) {
      angVal = (90 - ea.y);
    }
    // Angling right side up RIGHT is TOP (toward PLUMB)
    if (ea.x > 0 && ea.y <= 35 && ea.y > -35) {
      angVal = ea.y;
    }
    // LEFT IS TOP (PLUMB)
    if (ea.x > 0 && ea.y <= -35 && ea.y > -90) {
      angVal = ea.y + 90;
    }
    // TOP IS TOP angling Left side (LEVEL)
    if (ea.x < 0 && ea.y > 35 && ea.y <= 90) {
      angVal = (90 - ea.y);
    }
    // Upside down - BOTTOM IS TOP (LEVEL)
    if (ea.x < 0 && ea.y <= 35 && ea.y > -35) {
      angVal = ea.y;
    }
    // Upside down - BOTTOM IS TOP
    if (ea.x < 0 && ea.y <= -35 && ea.y > -90) {
      angVal = ea.y + 90;
    }
    // laying down face up - this is also Calibration position
    // need to also check Z here or it can get confused with another position
    if (ea.x < 5 && ea.x > -20 && ea.y <= 35 && ea.y > -35 && ea.z < 5) {
      angVal = 90 - ea.y;
      if (angVal > 50) {
        angVal -= 90; // bandaid fix from being laid flat...
      }
    }

    //angVal is what we need
    Serial.print("Angle value: ",angVal);
  }
}