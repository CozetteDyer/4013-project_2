void loop() {
  // put your main code here, to run repeatedly:
  if (bluetooth.available())
  {
    char c = (char)bluetooth.read();
    Serial.write("gaming");
    Serial.write(c);



  }
  if (Serial.available())
  {

     char c = (char)Serial.read();
     bluetooth.write("goober");
     bluetooth.write(c);