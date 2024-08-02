#include <Wire.h>

const int i2c_address = 0x30;

const byte CMD_FORWARD = 0x01;
const byte CMD_BACKWARD = 0x02;
const byte CMD_RIGHT = 0x03;
const byte CMD_LEFT = 0x04;
const byte CMD_STOP = 0x05;
const byte CMD_ERR = 0xff;

const int IN1 = 9;
const int IN2 = 10;
byte prev_command = 0xff;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  Serial.begin(9600); // Initialize serial communication for debugging
  Wire.begin(i2c_address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop(){
  delay(10); // Short delay to ensure the loop runs smoothly
}

void receiveEvent(int howMany){
  while (Wire.available()){
    byte command = Wire.read();
    //Serial.print("Received command: ");
    //Serial.println(command); // Print received command for debugging
    if(command == CMD_ERR){
      executeCommand(prev_command);
    } else {
      executeCommand(command);
      prev_command = command;
    }
  }
}

void executeCommand(byte command){
  switch (command){
    case CMD_FORWARD:
      digitalWrite(IN2, LOW);
      analogWrite(IN1, 168); // Full speed forward
      break;
    case CMD_BACKWARD:
      digitalWrite(IN1, LOW);
      analogWrite(IN2, 168); // Full speed backward
      break;
    case CMD_RIGHT:
      digitalWrite(IN1, LOW);
      analogWrite(IN2, 255); // Half speed forward (for turning right)
      break;
    case CMD_LEFT:
      digitalWrite(IN2, LOW);
      analogWrite(IN1, 255); // Half speed backward (for turning left)
      break;
    case CMD_STOP:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW); // Stop
      break;
    default:
      break;
  }
  //Serial.print("Executed command: ");
  //Serial.println(command); // Print executed command for debugging
}

void requestEvent(){
  Wire.write("1"); //return encoder value
}
