#include <Wire.h>

// Commands
const byte CMD_FORWARD = 0x01;
const byte CMD_BACKWARD = 0x02;
const byte CMD_RIGHT = 0x03;
const byte CMD_LEFT = 0x04;
const byte CMD_STOP = 0x05;
const byte CMD_ERR = 0x0f;

// Slave addresses
const byte SLAVE_ADDRESSES[] = {0x10, 0x20, 0x30, 0x40};
const int NUM_SLAVES = sizeof(SLAVE_ADDRESSES) / sizeof(SLAVE_ADDRESSES[0]);

void setup() {
  Serial.begin(9600); // Start serial communication
  while (!Serial); // Wait for serial port to connect
  Wire.begin(); // Start I2C as master
  delay(3000); // Initial delay for startup
}

void loop() {
  String state_package = "";
  if (Serial.available() > 0) {
    String received_str = Serial.readStringUntil('\n');
    delay(10); // ensure receptio  // while (Wire.available()){
  //   byte command = Wire.read();
  //   if(command == CMD_ERR){
  //     executeCommand(prev_command);
  //   }else{
  //     executeCommand(command);
  //     prev_command = command;
  //   }
  // }
  //     }
  //     // Send command to each slave
  //     Wire.beginTransmission(SLAVE_ADDRESSES[i]);
  //     Wire.write(command);
  //     Wire.endTransmission();
  //     //no receiving feedback for now
  //     // Request state from each slave
  //     //Wire.requestFrom((int)SLAVE_ADDRESSES[i], sizeof(long));
  //     //while(Wire.available() < sizeof(long)) {
  //     //  ; // Wait until all bytes are received
  //     //}

  //     //long receivedValue = 0;
  //     //for (int j = 0; j < sizeof(long); j++) {
  //     //  receivedValue |= ((long)Wire.read()) << (j * 8); // Reconstruct the long from bytes
  //     //}  // while (Wire.available()){
  //   byte command = Wire.read();
  //   if(command == CMD_ERR){
  //     executeCommand(prev_command);
  //   }else{
  //     executeCommand(command);
  //     prev_command = command;
  //   }
  // }
  //   }

  //   // Send the collected states to Raspberry Pi
  //   //Serial.println(state_package);  //No feedback for now
  }
  delay(10);
}

byte determineCommand(String input) {
  if(input == "forward") {
    Serial.println("f");
    return CMD_FORWARD;
  } else if(input == "backwards") {
    Serial.println("b");
    return CMD_BACKWARD;
  } else if(input == "right") {
    Serial.println("r");
    return CMD_RIGHT;
  } else if(input == "left") {
    Serial.println("l");
    return CMD_LEFT;
  } else if(input == "stop") {
    Serial.println("s");
    return CMD_STOP;
  } else {
    Serial.println("#");
    return CMD_ERR;
  }
}
