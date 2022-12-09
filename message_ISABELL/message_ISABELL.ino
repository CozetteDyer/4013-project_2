//Copy created specifically for the purpose of testing the ReadWrite function for the built in SD.

#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <SD.h>
#include <SPI.h>

// For Radio
SoftwareSerial mySerial(21, 20); //Rx,Tx

// For GPS
// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire);
#define GPSECHO false
uint32_t timer = millis();

String Date = "N/A";
String Time = "N/A";
String Longitude = "N/A";
String Latitude = "N/A";
String Elevation = "N/A"; // Altitude in meters above MSL
String Satellites = "N/A";

// For IMU
Adafruit_LSM6DSOX lsm6ds;
Adafruit_LIS3MDL lis3mdl;

String accelX = "N/A";
String accelY = "N/A";
String accelZ = "N/A";
String gyroX = "N/A";
String gyroY = "N/A";
String gyroZ = "N/A";
String magX = "N/A";
String magY = "N/A";
String magZ = "N/A";

//For SD card write
File dataFile;
const int chipSelect = BUILTIN_SDCARD;

void setup() {
  // Serial Setup
  mySerial.begin(9600);
  pinMode(2, OUTPUT); // set LED PIN to output

  //GPS
  Serial.begin(115200); //?
  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPS.println(PMTK_Q_RELEASE);

  //IMU Setup
  bool lsm6ds_success = lsm6ds.begin_I2C();
  bool lis3mdl_success = lis3mdl.begin_I2C();
  if(!lsm6ds_success || !lis3mdl_success) { Serial.println("Failed to find IMU"); }
  else { Serial.println("IMU Connected!");}

  // SD CARD
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");  
  dataFile = SD.open("data.csv", FILE_WRITE);
  while (!dataFile){
    dataFile = SD.open("data.csv", FILE_WRITE);
  }
  String header = "Date,Time,Satellites,Latitude,Longitude,Elevation MSL(M),X Accel(m/s^2),Y Accel(m/s^2),Z Accel(m/s^2),X Mag(uT), Y Mag(uT), Z Mag(uT), X Gyro(rps), Y Gyro(rps), Z Gyro(rps)";
  dataFile.println(header);
  Serial.println(header);
  dataFile.close();
}

void loop() {
  // GPS SETUP
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  
//  if(GPS.newNMEAreceived()){ //if GPS is connected LED stays on
//    digitalWrite(2, HIGH);
//  } else {
//      digitalWrite(2, HIGH); // sets the digital pin on
//      delay(1000);            // waits for a second
//      digitalWrite(2, LOW);  // sets the digital pin off
//      delay(1000);            // waits for a second
//  }
  
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    GpsData();
    delay(10000);
    ImuData();
    WriteData();
  }
}

void GpsData(){
  Serial.print("Fix: "); Serial.println((int)GPS.fix);
  if (GPS.fix){
    Latitude = String(GPS.latitude) + String(GPS.lat);
    Longitude = String(GPS.longitude) + String(GPS.lon);
    Elevation = String(GPS.altitude);
    Satellites = String((int)GPS.satellites);
    Time = String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + "." + String(GPS.milliseconds);
    Date = String(GPS.month) + "/" + String(GPS.day) + "/" + String(GPS.year);
  }
}

void ImuData(){
  sensors_event_t accel, gyro, mag, temp;
  // /* Get new normalized sensor events */
  lsm6ds.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);
  accelX = String(accel.acceleration.x, 4);
  accelY = String(accel.acceleration.y, 4);
  accelZ = String(accel.acceleration.z, 4);
  gyroX = String(gyro.gyro.x, 4);
  gyroY = String(gyro.gyro.y, 4);
  gyroZ = String(gyro.gyro.z, 4);
  magX = String(mag.magnetic.x, 4);
  magY = String(mag.magnetic.y, 4);
  magZ = String(mag.magnetic.z, 4);
}

//void GpsSetup(){
//  // put your main code here, to run repeatedly:
//   // read data from the GPS in the 'main loop'
//  char c = GPS.read();
//  // if you want to debug, this is a good time to do it!
//  if (GPSECHO)
//    if (c) Serial.print(c);
//  // if a sentence is received, we can check the checksum, parse it...
//  if (GPS.newNMEAreceived()) {
//    // a tricky thing here is if we print the NMEA sentence, or data
//    // we end up not listening and catching other sentences!
//    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
//    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
//    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
//      return; // we can fail to parse a sentence in which case we should just wait for another
//  }
//}

void WriteData() {
  dataFile = SD.open("data.csv", FILE_WRITE);
  String CsvData = Date + "," + Time + "," + Satellites + "," + Latitude + "," + Longitude + "," + Elevation + " MSL," +
  accelX + "," + accelY + "," + accelZ + "," + magX + "," + magY + "," + magZ + "," + gyroX + "," + gyroY + "," + gyroZ;
  
  dataFile.println(CsvData);
  dataFile.close();
  Serial.println(CsvData);
  mySerial.print(CsvData);
}