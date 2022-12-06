#include <SoftwareSerial.h> // serial monitor
#include <Adafruit_LSM6DS33.h> // 6-DoF Accelerometer and Gyroscope Sensor
#include <Adafruit_LIS3MDL.h> // magnetometer
#include <SD.h> // SD card lib
#include <SPI.h> // for SD card
#include <Adafruit_GPS.h> // GPS baby

File sdFile; // create SD card instance
const int chipSelect = BUILTIN_SDCARD; // using built in SD card in Teensy

Adafruit_LSM6DS33 accelerometer; // accel and gyro 
Adafruit_LIS3MDL magnetometer; // magnetometer

#define GPSSerial Serial5 // using serial port 5
Adafruit_GPS GPS(&GPSSerial); // Connect to the GPS on the hardware port
#define GPSECHO false // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
                      // Set to 'true' if you want to debug and listen to the raw GPS sentences
String gps_date;
String gps_time;
String gps_longitude;
String gps_latitude;
String gps_satellites;
String gps_altitude;
uint32_t  timer = millis();

const int rx = 0;
const int tx =  1;
SoftwareSerial bluetooth(rx, tx);

void getIMU () {
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
    writeIMU_SD(ax, ay, az);
    //Serial.println(accelString);
    
    
    /* Display the results (rotation is measured in rad/s) */
    gx = String(gyro.gyro.x, 4);
    gy = String(gyro.gyro.y, 4);
    gz = String(gyro.gyro.z, 4);
    gyroString = "GX = " + gx + "\tGY = " + gy + "\tGZ = " + gz;
    writeIMU_SD(gx, gy, gz);
    //Serial.println(gyroString);

    /* Display the results (magnetic field is measured in uTesla) */
    mx = String(magn.magnetic.x, 4);
    my = String(magn.magnetic.y, 4);
    mz = String(magn.magnetic.z, 4);
    magnString = "MX = " + mx + "\tMY = " + my + "\tMZ = " + mz;     
    writeIMU_SD(mx, my, mz);
    //Serial.println(magnString);

    /* Display the results (magnetic field is measured in uTesla) */
    tempV = String(temp.temperature, 2); // 2 decimal places
    //Serial.println("temp " + tempV);
    delay(1000);
}   // end of imu()

void getGPS() {
    char c = GPS.read(); // read data from the GPS in the 'main loop'

    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            return; // we can fail to parse a sentence in which case we should just wait for anotherchar c = GPS.read();
    } 

    if (millis() - timer > 2000) {   // approximately every 2 seconds or so, print out the current stats
        timer = millis(); // reset the gps_timer
        Serial.print("Fix:\t"); Serial.print((int)GPS.fix);
        Serial.print("\t Quality:\t"); Serial.println((int)GPS.fixquality);
        if (GPS.fix) {
            gps_time = String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds);
            gps_date = String(GPS.month) + "/" + String(GPS.day) + "/" + String(GPS.year);
            Serial.println("TIME:\t" + gps_date + "\tDATE:\t" + gps_date);

            gps_satellites = String((int)GPS.satellites);
            gps_latitude = String(GPS.latitude) + String(GPS.lat);
            gps_longitude = String(GPS.longitude) + String(GPS.lon);
            gps_altitude = String(GPS.altitude);
            //Serial.println("SAT:\t" + gps_satellites + "\tLAT:\t" + gps_latitude +"\tLON:\t" + gps_longitude +"\tALT:\t" + gps_altitude );

            writeGPS_SD(gps_time, gps_date, gps_satellites); 
            writeGPS_SD(gps_latitude, gps_longitude, gps_altitude); 
        } // end of (GPS.fix) loop
    } // end of (millis() - gps_timer > 2000)  loop
} // end of GPS functions

void writeIMU_SD(String d1, String d2, String d3) {
    sdFile = SD.open("IMU_test.txt", FILE_WRITE); // change file name ***
    //Serial.print("Writing to IMU_test.txt. . .");

    // if the file opened okay, write to it:
    if (sdFile) {
        sdFile.print(d1 + "," + d2 + "," + d3 + ",");
        // close the file:
        sdFile.close();
    } 
    
    else { // if the file didn't open, print an error:
        Serial.println("\n\nerror opening IMU.txt\n\n");
    }
    //Serial.println("writing complete.");
} // end of write IMU --> SD card function

void readIMU_SD() {   
    sdFile = SD.open("IMU_test.txt"); // re-open the file for reading
    Serial.println("\n\nIMU_test!");
    if (sdFile) {
        // read from the file until there's nothing else in it:
        while (sdFile.available()) {
            Serial.write(sdFile.read());
        }
        
        sdFile.close(); // close the file
        } 
    
    else {
        // if the file didn't open, print an error:
        Serial.println("\n\nerror opening IMU.txt\n\n");
    }

} // read from IMU -> SD card function 

void writeGPS_SD(String d1, String d2, String d3) {
   sdFile = SD.open("GPS_test.txt", FILE_WRITE); 

    Serial.println("\n\nGPS_test!");

   if (sdFile) { // if the file opened okay, write to it:
       sdFile.print(d1 + "," + d2 + "," + d3 + ",");
       // close the file:
       sdFile.close();
   } 
   
   else { // if the file didn't open, print an error:
       Serial.println("\n\nerror opening GPS.txt\n\n");
   }
   //Serial.println("writing complete.");
} // end of write GPS -> SD card function

void readGPS_SD() {   
   sdFile = SD.open("GPS_test.txt"); // re-open the file for reading
   if (sdFile) {
    // read from the file until there's nothing else in it:
        while (sdFile.available()) {
            Serial.write(sdFile.read());
        } // end of while
    sdFile.close(); // close the file
   } 
   else { // if the file didn't open, print an error:
        Serial.println("\n\nerror opening GPS.txt\n\n");
   }
} // read from GPS -> SD card function 


void writeSD_headers() {

    SD.remove("IMU_test.txt"); // revomes previous IMU data file
    SD.remove("GPS_test.txt"); // remove previous GPS data file
    
    sdFile = SD.open("IMU_test.txt", FILE_WRITE); // change file name ***
    //Serial.print("Writing to IMU_test.txt. . .");

    // if the file opened okay, write to it:
    if (sdFile) {
        sdFile.print("Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z, Magn X, Magn Y, Magn Z");
        sdFile.close();
    } 
    
    else { // if the file didn't open, print an error:
        Serial.println("\n\nerror opening IMU.txt for Header\n\n");
    }

 // ------------------------------------------------------ GPS
    sdFile = SD.open("GPS_test.txt", FILE_WRITE); // change file name ***
    //Serial.print("Writing to GPS_test.txt. . .");

    // if the file opened okay, write to it:
    if (sdFile) {
        sdFile.print("Time, Date, Satellites, Latitude, Longitude, Elevation");
        sdFile.close();
    } 
    
    else { // if the file didn't open, print an error:
        Serial.println("\n\nerror opening GPS.txt for Header\n\n");
    }

} // end of write to SD card function --- HEADERS!!

void setup() {
    Serial.println("CODE: main\n");
    Serial.begin(115200); // imu.begin(115200);
  
    // bluetooth.begin(9600);
    //   delay(100);
    //   bluetooth.println("Let's Start!");
    //   delay(100);

    // ****** IMU ********************************************************     IMU-setup
    bool accelerometer_success, magnetometer_success;

    // hardware I2C mode, can pass in address & alt Wire
    accelerometer_success = accelerometer.begin_I2C();
    magnetometer_success = magnetometer.begin_I2C();

    if (!accelerometer_success){
      Serial.println("\n\nFailed to find accelerometer chip\n\n");
    }

    if (!magnetometer_success){
      Serial.println("\n\nFailed to find accelerometer chip\n\n");
    }

    // Double check these values!!                                                    
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
    


    // ****** GPS *****************************************************     GPS - setup
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status
    delay(1000);
    GPSSerial.println(PMTK_Q_RELEASE);  // Ask for firmware version
    // ********************************************************************     end of GPS - setup
    

    
    // ****** SD Card ******************************************************     SD card setup
    // Open serial communications and wait for port to open:
    Serial.begin(9600);
    while (!Serial) {
        ; // wait for serial port to connect.
    }
    Serial.print("\n\nInitializing SD card...\n\n");
    if (!SD.begin(chipSelect)) {
        Serial.println("\n\nSD initialization failed\n\n");
        return;
    }
    writeSD_headers(); //writing headers to SD card
    //Serial.println("SD initialization done.");
// *********************************************************************     SD card setup
} // end of void setup()

void loop() {
    //   if (bluetooth.available())
    //   {
    //     char c = (char)bluetooth.read();
    //     Serial.write(c);
    //   }

    getIMU(); // get data from IMU
    getGPS(); // get dat from GPS

    readIMU_SD();
    readGPS_SD();

} // end of void loop()
