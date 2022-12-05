#include <SoftwareSerial.h> // serial monitor
#include <Adafruit_LSM6DS33.h> // 6-DoF Accelerometer and Gyroscope Sensor
#include <Adafruit_LIS3MDL.h> // magnetometer
#include <SD.h> // SD card lib
#include <SPI.h> // for SD card

File sdFile; // create SD card instance
const int chipSelect = BUILTIN_SDCARD; // using built in SD card in Teensy

Adafruit_LSM6DS33 accelerometer; // accel and gyro 
Adafruit_LIS3MDL magnetometer; // magnetometer


void IMU () {
  // By default, the I2C address is 0x6A.  
  // If you add a jumper from DO to 3.3V, the address will change to 0x6B

  // *********************************************************************     IMU 
    sensors_event_t accel, gyro, magn, temp;
    accelerometer.getEvent(&accel, &gyro, &temp);
    magnetometer.getEvent(&magn);
    String ax, ay, az;
    String gx, gy, gz;
    String mx, my, mz;
    String tempV;
    String accelString, gyroString, magnString;

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
    mx = String(magn.magnetic.x, 4);
    my = String(magn.magnetic.y, 4);
    mz = String(magn.magnetic.z, 4);
    magnString = "MX = " + mx + "\tMY = " + my + "\tMZ = " + mz;     
    writeSD(mx, my, mz);
    //Serial.println(magnString);

    /* Display the results (magnetic field is measured in uTesla) */
    //tempV = String(temp.temperature, 2); // 2 decimal places
    //writeTempSD(tempV);
    //Serial.println("\ntemp " + tempV);
    //delay(1000);
    // *********************************************************************     end of IMU
}   // end of imu()


void writeSD(String d1, String d2, String d3) {
    sdFile = SD.open("IMU_test.txt", FILE_WRITE); // change file name ***
    //Serial.print("Writing to IMU_test.txt. . .");

    // if the file opened okay, write to it:
    if (sdFile) {
        sdFile.print(d1 + "," + d2 + "," + d3 + ",");
        // close the file:
        sdFile.close();
    } 
    
    else { // if the file didn't open, print an error:
        Serial.println("error opening IMU.txt");
    }
    //Serial.println("writing complete.");
} // end of write to SD card function

//void writeTempSD(String temp) {
//    sdFile = SD.open("IMU_test.txt", FILE_WRITE); // change file name ***
//
//    if (sdFile) { //checking if file is open
//        sdFile.print(temp + ",");
//        sdFile.close(); // close the file:
//    } 
//    
//    else { // if the file didn't open, print an error:
//        Serial.println("error opening IMU.txt (TEMP!!!!)");
//    }
//    //Serial.println("writing complete.");
//} // end of write to SD card function


void readSD() {   
    sdFile = SD.open("IMU_test.txt"); // re-open the file for reading
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
    Serial.println("CODE: imu-SD card unit test!\n\n");

    Serial.begin(115200); 

    // ****** IMU ********************************************************     IMU-setup
    bool accelerometer_success, magnetometer_success;

    // hardware I2C mode, can pass in address & alt Wire
    accelerometer_success = accelerometer.begin_I2C();
    magnetometer_success = magnetometer.begin_I2C();

    if (!accelerometer_success){
    Serial.println("Failed to find accelerometer chip");
    }

    if (!magnetometer_success){
    Serial.println("Failed to find accelerometer chip");
    }

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


    // ****** SD Card ******************************************************     SD card setup

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

    IMU(); // get data from IMU

    readSD();

} // end of void loop()
