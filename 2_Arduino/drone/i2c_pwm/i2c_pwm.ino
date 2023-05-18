#include <Servo.h>
#include <Wire.h>

Servo FR;
Servo FL;
Servo BR;
Servo BL;

// {FL, FR, BL, BR}
int cmd[4] = {900, 900, 900, 900};

void setup() {
  // put your setup code here, to run once:
//  Serial.begin(115200);
  FL.attach(5);
  FR.attach(6);
  BL.attach(9);
  BR.attach(10);
  Wire.begin(0x40);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvents);
  pinMode(LED_BUILTIN, OUTPUT);
//  Serial.println("Begin");


}

void loop() {
  // put your main code here, to run repeatedly:

}
union ArrayToInteger {
  byte array[4];
  uint32_t integer;
};
void receiveEvent(int howMany)
{
  ArrayToInteger converter;
  digitalWrite(LED_BUILTIN, HIGH);
  if (Wire.available()) {
    int header = Wire.read();
    int i = 0;
    while (Wire.available()) { // loop through all but the last
      byte c = Wire.read();
      converter.array[i] = c ;
      i++;
    }
    int idx;
    switch(header){
      case 0x10:
        idx = 0;
        break;
      case 0x11:
        idx = 1;
        break;
      case 0x12:
        idx = 2;
        break;
      case 0x13:
        idx = 3;
        break;
    }
    cmd[idx] = converter.integer;
  }
}

void requestEvents() {
  Wire.write(byte(24));
  write_motor(cmd);
  digitalWrite(LED_BUILTIN, LOW);
}

void write_motor(int cmd_new[4]) {
  FL.writeMicroseconds(cmd_new[0]);
  FR.writeMicroseconds(cmd_new[1]);
  BL.writeMicroseconds(cmd_new[2]);
  BR.writeMicroseconds(cmd_new[3]);
//  Serial.print(cmd[0]);
//  Serial.print(" ");
//  Serial.print(cmd[1]);
//  Serial.print(" ");
//  Serial.print(cmd[2]);
//  Serial.print(" ");
//  Serial.println(cmd[3]);
}
