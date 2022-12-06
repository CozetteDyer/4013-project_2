#include <SoftwareSerial.h>

#define USBserial Serial4

const int rx = 16;
const int tx = 17;

void setup() {
  // put your setup code here, to run once:
    USBserial.begin(9600);
    delay(100);
    USBserial.println("Let's go!");
    delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:

    if (USBserial.available())
    {
        char c =(char)USBserial.read();
        USBserial.write(c);
    }
}