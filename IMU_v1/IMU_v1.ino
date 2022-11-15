// Skeleton from: https://learn.adafruit.com/st-9-dof-combo/arduino
// Modified by Cozette Dyer 11/14/2022

// SPDX-FileCopyrightText: 2020 Kattni Rembor for Adafruit Industries
//
// SPDX-License-Identifier: MIT

// Basic demo for accelerometer, gyro, and magnetometer readings
// from the following Adafruit ST Sensor combo boards:
// * LSM6DSOX + LIS3MDL FeatherWing : https://www.adafruit.com/product/4565
// * ISM330DHCX + LIS3MDL FeatherWing https://www.adafruit.com/product/4569
// * LSM6DSOX + LIS3MDL Breakout : https://www.adafruit.com/product/4517
// * LSM6DS33 + LIS3MDL Breakout Lhttps://www.adafruit.com/product/4485


// By default, the I2C address is 0x6A.  
// If you add a jumper from DO to 3.3V, the address will change to 0x6B

#include <Adafruit_LSM6DS33.h> // 6-DoF Accelerometer and Gyroscope Sensor
Adafruit_LSM6DS33 lsm6ds;

#include <Adafruit_LIS3MDL.h> // magnetometer
Adafruit_LIS3MDL lis3mdl;

void setup(void) {
  Serial.begin(115200); // baud rate of I2C??
  while (!Serial) // gives time for serial port to connect
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
 
  bool lsm6ds_success, lis3mdl_success;

  // hardware I2C mode, can pass in address & alt Wire

  lsm6ds_success = lsm6ds.begin_I2C();
  lis3mdl_success = lis3mdl.begin_I2C();

  if (!lsm6ds_success){
    Serial.println("Failed to find LSM6DS chip");
  }
  if (!lis3mdl_success){
    Serial.println("Failed to find LIS3MDL chip");
  }
  if (!(lsm6ds_success && lis3mdl_success)) {
    while (1) {
      delay(1); // 10
    }
  }

  Serial.println("LSM6DS and LIS3MDL Found!");

// Double check these values!!                                         *FINISH*
  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  lsm6ds.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE); // should we use low power?
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  
  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!

} // end of SETUP() loop

void loop() {

  sensors_event_t accel, gyro, mag, temp;

  lsm6ds.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x, 4);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y, 4);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z, 4);
  Serial.println(" \tm/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro  X: ");
  Serial.print(gyro.gyro.x, 4);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y, 4);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z, 4);
  Serial.println(" \tradians/s ");

  /* Display the results (magnetic field is measured in uTesla) */
  Serial.print(" \t\tMag   X: ");
  Serial.print(mag.magnetic.x, 4);
  Serial.print(" \tY: ");
  Serial.print(mag.magnetic.y, 4);
  Serial.print(" \tZ: ");
  Serial.print(mag.magnetic.z, 4);
  Serial.println(" \tuTesla ");

  Serial.print("\t\tTemp   :\t\t\t\t\t");
  Serial.print(temp.temperature);
  Serial.println(" \tdeg C");
  Serial.println();
  delay(1000);

} // end of LOOP() loop (main loop, repeats)
