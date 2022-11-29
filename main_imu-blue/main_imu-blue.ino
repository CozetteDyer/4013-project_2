#include <SoftwareSerial.h>
#include <Wire.h> // for I2C for IMU
#include <Adafruit_LSM6DS33.h> // 6-DoF Accelerometer and Gyroscope Sensor
#include <Adafruit_LIS3MDL.h> // magnetometer

Adafruit_LSM6DS33 accelerometer; // accel and gyro 
Adafruit_LIS3MDL magnetometer; // magnetometer

const int rx = 0;
const int tx =  1;


SoftwareSerial bluetooth(rx, tx);
//SoftwareSerial gps(dummy1, dummy2);
//SoftwareSerial imu(dummy3, dummy4);

// void IMU ()
// {
//   // By default, the I2C address is 0x6A.  
//   // If you add a jumper from DO to 3.3V, the address will change to 0x6B

//     // finish
  
// }

// void GPS()
// {
//     // finish
// }

void setup() {
  // put your setup code here, to run once:

  //gps.begin(9600);
    Serial.begin(11520);
  //imu.begin(115200);
 // bluetooth.begin(9600);

//   delay(100);
//   bluetooth.println("Let's Start!");

//   delay(100);

  //                                                                                IMU Set-up

    Serial.println("CODE: main_imu-blue!");
    bool accelerometer_success, magnetometer_success;

  // hardware I2C mode, can pass in address & alt Wire

    accelerometer_success = accelerometer.begin_I2C();
    magnetometer_success = magnetometer.begin_I2C();

    if (!accelerometer_success){
    Serial.println("Failed to find accelerometer chip");
    }

    /* FOR some reson this doesnt work. Teensy cant seem to find the chip, but we still get data. Not sure if data is correct 
     *  Need to revisit!!
     */
//    if (!magnetometer_success){
//    Serial.println("Failed to find magnetometer chip");
//    }
//    if (!(accelerometer_success && magnetometer_success)) {
//    while (1) {
//        delay(1); // 10
//        }
//    }
    Serial.println("LSM6DS and LIS3MDL Found!");
    // Double check these values!!                                                    *FINISH*
    accelerometer.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    accelerometer.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
    accelerometer.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
    accelerometer.setGyroDataRate(LSM6DS_RATE_12_5_HZ);

    magnetometer.setDataRate(LIS3MDL_DATARATE_155_HZ);
    magnetometer.setRange(LIS3MDL_RANGE_4_GAUSS);
    magnetometer.setPerformanceMode(LIS3MDL_MEDIUMMODE); // should we use low power?
    magnetometer.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    magnetometer.setIntThreshold(500);
    magnetometer.configInterrupt(false, false, true, // enable z axis
                                true, // polarity
                                false, // don't latch
                                true); // enabled!

} // end of void setup()



void loop() {
  // put your main code here, to run repeatedly:
//   if (bluetooth.available())
//   {
//     char c = (char)bluetooth.read();
//     Serial.write(c);

//   }


    // IMU 
    sensors_event_t accel, gyro, mag, temp;
    accelerometer.getEvent(&accel, &gyro, &temp);
    magnetometer.getEvent(&mag);
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float tempV;
    String accelString, gyroString, magString;


    /* I2C is 8-bit data bus, where address can be 7 bit or 10 bit, 
       you can't send more than 8 bit at a time */

    /* Display the results (acceleration is measured in m/s^2) */
    ax = accel.acceleration.x; // Accel X
    ay = accel.acceleration.y; // Accel Y
    az = accel.acceleration.z; // Accel Z
    accelString = "AX = " + ax + "\tAY = " + ay + "\tAZ = " + az; 
    Serial.println(accelString);

    /* Display the results (rotation is measured in rad/s) */
    gx = gyro.gyro.x;
    gy = gyro.gyro.y;
    gz = gyro.gyro.z;
    gyroString = "GX = " + gx + "\tGY = " + gy + "\tGZ = " + gz; 
    Serial.println(gyroString);

    /* Display the results (magnetic field is measured in uTesla) */
    mx = mag.magnetic.x;
    my = mag.magnetic.y;
    mz = mag.magnetic.z;
    magString = "MX = " + mx + "\tMY = " + my + "\tMZ = " + mz; 
    Serial.println(magString);

    /* Display the results (magnetic field is measured in uTesla) */
    tempV = temp.temperature;
    Serial.println("temp " + tempV);
    delay(1000);

 
} // end of void loop()
