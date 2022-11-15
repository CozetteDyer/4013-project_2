#include <SoftwareSerial.h>

const int rx = 0;
const int tx =  1;

SoftwareSerial bluetooth(rx, tx);
//SoftwareSerial gps(dummy1, dummy2);
//SoftwareSerial imu(dummy3, dummy4);

//void IMU ()
{



}

//void GPS()
{



  
}

void setup() {
  // put your setup code here, to run once:

  
  //gps.begin(9600);
  //imu.begin(115200);
  bluetooth.begin(9600);

  delay(100);
  bluetooth.println("Let's Start!");

  delay(100);

  


}







void loop() {
  // put your main code here, to run repeatedly:
  if (bluetooth.available())
  {
    char c = (char)bluetooth.read();
    Serial.write(c);


  }
 
}
