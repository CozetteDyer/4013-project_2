#include <Adafruit_GPS.h>

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

// --------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200); // GPS
  Serial.println("Adafruit GPS library basic parsing test!");

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status, comment out to keep quiet
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);  // Ask for firmware version
} // end of set up 

void loop() {
  char c = GPS.read(); // read data from the GPS in the 'main loop'
 
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
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
      Serial.println("SAT:\t" + gps_satellites + "\tLAT:\t" + gps_latitude +"\tLON:\t" + gps_longitude +"\tALT:\t" + gps_altitude );

    } // end of (GPS.fix) loop
  } // end of (millis() - gps_timer > 2000)  loop
} // end of loop
