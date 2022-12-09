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

String ax, ay, az;
String gx, gy, gz;
String mx, my, mz;
String tempV;
String accelString, gyroString, magnString;


#define GPSSerial Serial5 // using serial port 5
Adafruit_GPS GPS(&GPSSerial); // Connect to the GPS on the hardware port //GPSSerial
#define GPSECHO false // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
                      // Set to 'true' if you want to debug and listen to the raw GPS sentences
String gps_date;
String gps_time;
String gps_longitude;
String gps_latitude;
String gps_satellites;
String gps_altitude;
uint32_t  timer = millis();

const int rx = 34;
const int tx =  35;
SoftwareSerial bluetooth(rx, tx);

void getData () {
    // ------------------------------------------------------------------------------ IMU
    sensors_event_t accel, gyro, magn, temp;
    accelerometer.getEvent(&accel, &gyro, &temp);
    magnetometer.getEvent(&magn);
    /* Display the results (acceleration is measured in m/s^2) */
    ax = String(accel.acceleration.x, 4); // Accel X
    ay = String(accel.acceleration.y, 4); // Accel Y
    az = String(accel.acceleration.z, 4); // Accel Z
    accelString = "\tAX = " + ax + "\t|\tAY = " + ay + "\t|\tAZ = " + az + "\tm/s^2"; 
    
    
    /* Display the results (rotation is measured in rad/s) */
    gx = String(gyro.gyro.x, 4);
    gy = String(gyro.gyro.y, 4);
    gz = String(gyro.gyro.z, 4);
    gyroString = "\tGX = " + gx + "\t|\tGY = " + gy + "\t|\tGZ = " + gz + "\trad/s";

    /* Display the results (magnetic field is measured in uTesla) */
    mx = String(magn.magnetic.x, 4);
    my = String(magn.magnetic.y, 4);
    mz = String(magn.magnetic.z, 4);
    magnString = "\tMX = " + mx + "\t|\tMY = " + my + "\t|\tMZ = " + mz + "\tuTelsa";     

    /* Display the results in Celcius) */
    tempV = String(temp.temperature, 2); // 2 decimal places
    delay(1000);
    // --------------------------------------------------------------------------- GPS
    Serial.print("\nFix:\t"); Serial.print((int)GPS.fix);
    Serial.print("\tQuality:\t"); Serial.println((int)GPS.fixquality);
    if (GPS.satellites) { // .fix
        gps_time = String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds);
        gps_date = String(GPS.month) + "/" + String(GPS.day) + "/" + String(GPS.year);
        Serial.println("------------------------------------------------------------------------");
        Serial.println("\tTIME:" + gps_time + "\t|\tDATE:" + gps_date);

        gps_satellites = String((int)GPS.satellites);
        gps_latitude = String(GPS.latitude) + String(GPS.lat);
        gps_longitude = String(GPS.longitude) + String(GPS.lon);
        gps_altitude = String(GPS.altitude);
        Serial.println("\tSAT: " + gps_satellites + +"\t\t|\tALT: " + gps_altitude + "\n\tLAT: " + gps_latitude +"\t|\tLON:\t" + gps_longitude);

    } // end of (GPS.fix) loop

    Serial.println("\n" + accelString + "\n" + gyroString + "\n" + magnString + "\n" + "\ttemp = " + tempV + " °C");
    Serial.println("------------------------------------------------------------------------");

    // ------------------------------------------------------------------------------------------- write to SD 
    sdFile = SD.open("data.txt", FILE_WRITE); // change file name ***
    //Serial.print("Writing to data.txt. . .");

    // if the file opened okay, write to it:
    if (sdFile) {
        sdFile.print(gps_time + ',' +  gps_date + ',' + gps_satellites+ "," );
        sdFile.print(gps_latitude  + ',' + gps_longitude + ',' +  gps_altitude + "," ); 

        sdFile.print(ax + "," + ay + "," + az + "," );
        sdFile.print(gx + "," + gy + "," + gz + "," );
        sdFile.print(mx + "," + my + "," + mz + "\n" );
        
        sdFile.close(); // close the file:
    } 
    
    else { // if the file didn't open, print an error:
        Serial.println("\n\nerror opening data.txt\n\n");
    }
    delay(1000);
}   // end of imu()


void readSD() {   
   sdFile = SD.open("data.txt"); // re-open the file for reading
   if (sdFile) {
    // read from the file until there's nothing else in it:
        while (sdFile.available()) {
            Serial.write(sdFile.read());
        } // end of while
    sdFile.close(); // close the file
   } 
   else { // if the file didn't open, print an error:
        Serial.println("\n\nerror opening SD to READ\n\n");
   }
} // read from GPS -> SD card function 


void writeSD_headers() {

    SD.remove("data.txt"); // revomes previous data file
    
    sdFile = SD.open("data.txt", FILE_WRITE);
    //Serial.print("Writing to data.txt. . .");

    // if the file opened okay, write to it:
    if (sdFile) {
        sdFile.print("Time, Date, Satellites, Latitude, Longitude, Elevation, Accel X, Accel Y, Accel Z, Gyro X, Gyro Y, Gyro Z, Magn X, Magn Y, Magn Z, \n " );
        sdFile.close();
    } 
    
    else { // if the file didn't open, print an error:
        Serial.println("\n\nerror with Headers\n\n");
    }

} // end of write to SD card function --- HEADERS!!

void setup() {
    Serial.println("CODE: main\n");
    Serial.begin(9600); 
    bluetooth.begin(9600);
    delay(100);

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
    GPS.begin(9600); // 9600
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status
    delay(1000);
    GPSSerial.println(PMTK_Q_RELEASE);  // Ask for firmware version
    // ********************************************************************     end of GPS - setup
    

    
    // ****** SD Card ******************************************************     SD card setup
    // Open serial communications and wait for port to open:
    Serial.begin(9600); // 9600
    while (!Serial) {
        ; // wait for serial port to connect.
    }
    Serial.print("Initializing SD card...");
    if (!SD.begin(chipSelect)) {
        Serial.println("\n\nSD initialization failed\n\n");
        return;
    }
    writeSD_headers(); //writing headers to SD card
    Serial.println("SD initialization done.\n");
// *********************************************************************     SD card setup
} // end of void setup()

void loop() {
    //   if (bluetooth.available())
    //   {
    //     char c = (char)bluetooth.read();
    //     Serial.write(c);
    //   }

    char c = GPS.read(); // read data from the GPS in the 'main loop'

    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            return; // we can fail to parse a sentence in which case we should just wait for anotherchar c = GPS.read();
    } 

    if (millis() - timer > 4000) {  //2000 // approximately every 2 seconds or so, print out the current stats
        timer = millis(); // reset the gps_timer

        getData(); // get dat from GPS
        delay(1000); // wait babes
    }

    //readSD();

} // end of void loop()
