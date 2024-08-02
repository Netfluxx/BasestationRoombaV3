//Master Arduino Code

#include <Wire.h>

byte slave_addr = 0x00;
byte command = 0xff;

//commands
const byte CMD_FORWARD = 0x01;
const byte CMD_BACKWARD = 0x02;
const byte CMD_RIGHT = 0x03;
const byte CMD_LEFT = 0x04;
const byte CMD_STOP = 0x05;
//slave addresses
const byte SLAVE_1 = 0x10;
const byte SLAVE_2 = 0x20;
const byte SLAVE_3 = 0x30;
const byte SLAVE_4 = 0x40;

const byte CMD_ERR = 0xff;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(3000);
}

void loop() {
  if (Serial.available() > 0) {
    String received_str = Serial.readStringUntil('\n');
    delay(10);
    command = CMD_ERR;

    if(received_str == "forward"){
      Serial.println("f");
      command = CMD_FORWARD;
    }else if(received_str == "backwards"){
      Serial.println("b");
      command = CMD_BACKWARD;
    }else if(received_str == "right"){
      Serial.println("r");
      command = CMD_RIGHT;
    }else if(received_str == "left"){
      Serial.println("l");
      command = CMD_LEFT;
    }else if(received_str == "stop"){
      Serial.println("s");
      command = CMD_STOP;
    }else{
      command = CMD_ERR;
      Serial.println("#");
    }
  
  
    for(int slave_index=1; slave_index<5; slave_index++){
      if(slave_index!=1){  //for debugging with only one slave
        continue;
      }
      switch(slave_index){
        case 1:
          slave_addr = SLAVE_1;
          break;
        case 2:
          slave_addr = SLAVE_2;
          break;
        case 3:
          slave_addr = SLAVE_3:
          break;
        case 4:
          slave_addr = SLAVE_4;
          break//Master Arduino Code

#include <Wire.h>

byte slave_addr = 0x00;
byte command = 0xff;

//commands
const byte CMD_FORWARD = 0x01;
const byte CMD_BACKWARD = 0x02;
const byte CMD_RIGHT = 0x03;
const byte CMD_LEFT = 0x04;
const byte CMD_STOP = 0x05;
//slave addresses
const byte SLAVE_1 = 0x10;
const byte SLAVE_2 = 0x20;
const byte SLAVE_3 = 0x30;
const byte SLAVE_4 = 0x40;

const byte CMD_ERR = 0xff;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(3000);
}

void loop() {
  if (Serial.available() > 0) {
    String received_str = Serial.readStringUntil('\n');
    delay(10);
    command = CMD_ERR;

    if(received_str == "forward"){
      Serial.println("f");
      command = CMD_FORWARD;
    }else if(received_str == "backwards"){
      Serial.println("b");
      command = CMD_BACKWARD;
    }else if(received_str == "right"){
      Serial.println("r");
      command = CMD_RIGHT;
    }else if(received_str == "left"){
      Serial.println("l");
      command = CMD_LEFT;
    }else if(received_str == "stop"){
      Serial.println("s");
      command = CMD_STOP;
    }else{
      command = CMD_ERR;
      Serial.println("#");
    }
  
  
    for(int slave_index=1; slave_index<5; slave_index++){
      if(slave_index!=1){  //for debugging with only one slave
        continue;
      }
      switch(slave_index){
        case 1:
          slave_addr = SLAVE_1;
          break;
        case 2:
          slave_addr = SLAVE_2;
          break;
        case 3:
          slave_addr = SLAVE_3:
          break;
        case 4:
          slave_addr = SLAVE_4;
          break
        default:
          break;
      }
      Wire.beginTransmission(slave_addr);
      Wire.write(command);
      Wire.endTransmission();

      Wire.requestFrom(slave_addr, 1);
      if (Wire.available()) {
        byte state = Wire.read();
        //feed it back to the raspberry pi
      }else{
        ;//error
      }
    }
    command = CMD_ERR;
  }

  delay(10);
  command = CMD_ERR;
}

        default:
          break;
      }
      Wire.beginTransmission(slave_addr);
      Wire.write(command);
      Wire.endTransmission();

      Wire.requestFrom(slave_addr, 1);
      if (Wire.available()) {
        byte state = Wire.read();
        //feed it back to the raspberry pi
      }else{
        ;//error
      }
    }
    command = CMD_ERR;
  }

  delay(10);
  command = CMD_ERR;
}
