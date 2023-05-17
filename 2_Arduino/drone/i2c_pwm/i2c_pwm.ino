#include <Servo.h>
#include <Wire.h>

Servo FR;  
Servo FL;  
Servo BR;  
Servo BL;  
// {FR, FL, BR, BL}
int cmd[4] = {900, 900, 900, 900};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  FR.attach(5); 
  FL.attach(6); 
  BR.attach(9); 
  BL.attach(10);  
  Wire.begin(0x40); 
  Wire.onReceive(receiveEvent); 
  Wire.onRequest(requestEvents);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Begin");
  
  
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
  if (Wire.available()){
    int header = Wire.read();
    Serial.println(header);
    int i = 0;
    while (Wire.available()) { // loop through all but the last
      byte c = Wire.read();
      converter.array[i] = c ; 
      Serial.print(c, HEX);
      Serial.print(" ");
      i++;
  }
    
  }
  Serial.print("OUT: ");
  Serial.println(converter.integer); 
  

  
  
}

void requestEvents(){
  Wire.write(byte(24));
  Serial.println(" Sending: 0x01");
  digitalWrite(LED_BUILTIN, LOW); 
}

void write_motor(int cmd_new[4]){
  FR.writeMicroseconds(cmd_new[0]);
  FL.writeMicroseconds(cmd_new[1]);
  BR.writeMicroseconds(cmd_new[2]);
  BL.writeMicroseconds(cmd_new[3]);
}
