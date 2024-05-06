//Slave 1 Arduino Code

#include <Wire.h>

const int i2c_address = 0x10;

const byte CMD_FORWARD = 0x01;
const byte CMD_BACKWARD = 0x02;
const byte CMD_RIGHT = 0x03;
const byte CMD_LEFT = 0x04;
const byte CMD_STOP = 0x05;
const byte CMD_ERR = 0xff;

const int IN1 = 7;
const int IN2 = 8;
const int PWMA = 10;
byte prev_command = 0xff;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  Wire.begin(i2c_address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop(){
  delay(10);
}


void receiveEvent(int howMany){
  while (Wire.available()){
    byte command = Wire.read();
    if(command == CMD_ERR){
      executeCommand(prev_command);
    }else{
      executeCommand(command);
      prev_command = command;
    }
  }
}

void executeCommand(byte command){
  switch (command){
    case CMD_FORWARD:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(PWMA, 255);
      break;
    case CMD_BACKWARD:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(PWMA, 255);
      break;
    case CMD_RIGHT:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      break;
    case CMD_LEFT:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      break;
    case CMD_STOP:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      break;
    default:
      break;
  }
}

void requestEvent(){
  Wire.write("1"); //return encoder value
}
