

#include <Adafruit_GPS.h>
#define GPSSerial Serial5
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false

String GPS_Date;
String GPS_Time;
String GPS_Longitude;
String GPS_Lattitude;
String GPS_Satallites;
String GPS_Elevation;

void setup() {
  // put your setup code here, to run once:

  // GPS
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic parsing test!");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
}

void loop() {

//GPS
   // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for anotherchar c = GPS.read();

  }

   if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      GPS_Time = String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds);
      GPS_Date = String(GPS.month) + "/" + String(GPS.day) + "/" + String(GPS.year);
      GPS_Satallites = String((int)GPS.satellites);
      GPS_Lattitude = String(GPS.lattitude) + String(GPS.lat);
      GPS_Longitude = String(GPS.longitude) + String(GPS.lon);
      GPS_Altitude = String(GPS.altitude);
    }

    void exportInfo {
      //for the CSV file
    }
  }



