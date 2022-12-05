#include <SoftwareSerial.h>
#include <Wire.h> // for I2C for IMU
#include <Adafruit_LSM6DS33.h> // 6-DoF Accelerometer and Gyroscope Sensor
#include <Adafruit_LIS3MDL.h> // magnetometer
#include <SD.h> // SD card lib
#include <SPI.h> // for SD card

File sdFile;
const int chipSelect = BUILTIN_SDCARD;
Adafruit_LSM6DS33 accelerometer; // accel and gyro 
Adafruit_LIS3MDL magnetometer; // magnetometer

const int rx = 0;
const int tx =  1;


SoftwareSerial bluetooth(rx, tx);
//SoftwareSerial gps(dummy1, dummy2);
//SoftwareSerial imu(dummy3, dummy4);

void IMU () {
  // By default, the I2C address is 0x6A.  
  // If you add a jumper from DO to 3.3V, the address will change to 0x6B

  // *********************************************************************     IMU 
    sensors_event_t accel, gyro, mag, temp;
    accelerometer.getEvent(&accel, &gyro, &temp);
    magnetometer.getEvent(&mag);
    String ax, ay, az;
    String gx, gy, gz;
    String mx, my, mz;
    String tempV;
    String accelString, gyroString, magString;

    /* 
        I2C is 8-bit data bus, where address can be 7 bit or 10 bit, 
        you can't send more than 8 bit at a time 
        // String( val, decimal places)
    */
        
    /* Display the results (acceleration is measured in m/s^2) */
    ax = String(accel.acceleration.x, 4); // Accel X
    ay = String(accel.acceleration.y, 4); // Accel Y
    az = String(accel.acceleration.z, 4); // Accel Z
    accelString = "AX = " + ax + "\tAY = " + ay + "\tAZ = " + az; 
    //Serial.println(accelString);
    writeSD(ax, ay, az);
    
    /* Display the results (rotation is measured in rad/s) */
    gx = String(gyro.gyro.x, 4);
    gy = String(gyro.gyro.y, 4);
    gz = String(gyro.gyro.z, 4);
    gyroString = "GX = " + gx + "\tGY = " + gy + "\tGZ = " + gz;
    writeSD(gx, gy, gz);
    //Serial.println(gyroString);

    /* Display the results (magnetic field is measured in uTesla) */
    mx = String(mag.magnetic.x, 4);
    my = String(mag.magnetic.y, 4);
    mz = String(mag.magnetic.z, 4);
    magString = "MX = " + mx + "\tMY = " + my + "\tMZ = " + mz;     
    writeSD(mx, my, mz);
    //Serial.println(magString);

    /* Display the results (magnetic field is measured in uTesla) */
    tempV = String(temp.temperature, 2); // 2 decimal places
    Serial.println("temp " + tempV);
    delay(1000);
    // *********************************************************************     end of IMU
}   // end of imu()

void GPS() {
    // finish
}

void writeSD(String d1, String d2, String d3) {
    sdFile = SD.open("data_test.txt", FILE_WRITE); // change file name ***
    //Serial.print("Writing to data_test.txt. . .");

    // if the file opened okay, write to it:
    if (sdFile) {
        sdFile.print(d1 + "," + d2 + "," + d3 + ",");
        // close the file:
        sdFile.close();
    } 
    
    else { // if the file didn't open, print an error:
        Serial.println("error opening test.txt");
    }
    //Serial.println("writing complete.");
} // end of write to SD card function

void readSD() {   
    sdFile = SD.open("data_test.txt"); // re-open the file for reading
    //Serial.println("data_test.txt:");
    if (sdFile) {
        // read from the file until there's nothing else in it:
        while (sdFile.available()) {
            Serial.write(sdFile.read());
        }
        
        sdFile.close(); // close the file
        } 
    
    else {
        // if the file didn't open, print an error:
        Serial.println("\nerror opening test.txt\n");
    }

} // read from SD card function 

void setup() {
    // put your setup code here, to run once:
    Serial.println("CODE: main_sd-imu!\n");

    //gps.begin(9600);
    Serial.begin(11520); // imu.begin(115200);
  
    // bluetooth.begin(9600);
    //   delay(100);
    //   bluetooth.println("Let's Start!");
    //   delay(100);

    // *********************************************************************     IMU-setup
    bool accelerometer_success; //, magnetometer_success;

    // hardware I2C mode, can pass in address & alt Wire
    accelerometer_success = accelerometer.begin_I2C();
    // magnetometer_success = magnetometer.begin_I2C();

    if (!accelerometer_success){
    Serial.println("Failed to find accelerometer chip");
    }

    //Serial.println("IMU found!");
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
    // *********************************************************************     end of IMU - setup
    
    
    // *********************************************************************     SD card setup
    SPI.setMOSI(7);  // Audio shield has MOSI on pin 7
    SPI.setSCK(14);  // Audio shield has SCK on pin 14

    // Open serial communications and wait for port to open:
    Serial.begin(9600);
    while (!Serial) {
        ; // wait for serial port to connect.
    }

    Serial.print("Initializing SD card...");

    if (!SD.begin(chipSelect)) {
        Serial.println("SD initialization failed");
        return;
    }
    //Serial.println("SD initialization done.");

// *********************************************************************     SD card setup

} // end of void setup()



void loop() {
  // put your main code here, to run repeatedly:
    //   if (bluetooth.available())
    //   {
    //     char c = (char)bluetooth.read();
    //     Serial.write(c);
    //   }


    IMU(); // get data from IMU
    readSD();


 
} // end of void loop()
