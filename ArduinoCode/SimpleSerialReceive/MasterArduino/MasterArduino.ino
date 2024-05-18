#include <Wire.h>
#include <vector>

// command identifiers
const byte CMD_FORWARD  = 0x01;
const byte CMD_BACKWARD = 0x02;
const byte CMD_RIGHT    = 0x03;
const byte CMD_LEFT     = 0x04;
const byte CMD_STOP     = 0x05;

// I2C slave addresses
const byte SLAVE_1 = 0x10;  // Front Right Arduino
const byte SLAVE_2 = 0x20;  // Front Left  Arduino
const byte SLAVE_3 = 0x30;  // Back  Right Arduino
const byte SLAVE_4 = 0x40;  // Back  Left  Arduino

const byte CMD_ERR = 0xff;
const byte SLV_ERR = 0xfe;

byte slave_addr = SLV_ERR;
byte command = CMD_ERR;
const String spd_err = "SPEED_ERR"; // couldn't send data to slave
const String fb_err = "SLV_FB_ERR"; // slave didn't respond
const String distrib_err = "DIST_ERR"; // error when distributing the command to the slaves

// message format to send back to the RPi (comma separated): 
// received_cmd (from master arduino), f_r_send, f_r_fb, f_r_speed, f_l_send, f_l_fb, f_l_speed, b_r_send, b_r_fb, b_r_speed, b_l_send, b_l_fb, b_l_speed
// where:
// received_cmd = "f" or "b" or "r" or "l" or "s" or "#"
// x_y_send = "OK" or "DIST_ERR"
// x_y_fb   = "OK" or "SLV_FB_ERR"
// x_y_speed = String(float value) or "SPEED_ERR" 

std::vector<String> received_speeds = {spd_err, spd_err, spd_err, spd_err}; // speed values received from the slaves (0-255)
std::vector<String> SlaveSendError = {distrib_err, distrib_err, distrib_err, distrib_err}; // error when sending the command to the slaves (OK or DIST_ERR)
std::vector<String> SlaveReceiveError = {fb_err, fb_err, fb_err, fb_err}; // error when receiving the feedback from the slaves (OK or SLV_FB_ERR)

String state_feedback = fb_err; // custom feedback message to send to the RPi according to the format above

void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(1500);
}

void loop() {
  if (Serial.available() > 0) {
    String received_str = Serial.readStringUntil('\n');
    delay(6); // 6ms
    
    if (received_str == "forward") {
      state_feedback = "f,";
      command = CMD_FORWARD;
    } else if (received_str == "backwards") {
      state_feedback = "b,";
      command = CMD_BACKWARD;
    } else if (received_str == "right") {
      state_feedback = "r,";
      command = CMD_RIGHT;
    } else if (received_str == "left") {
      state_feedback = "l,";
      command = CMD_LEFT;
    } else if (received_str == "stop") {
      state_feedback = "s,";
      command = CMD_STOP;
    } else {
      command = CMD_ERR;
      state_feedback = "#,";
    }
  
    for (unsigned int slave_index = 1; slave_index <= 4; slave_index++) {
      switch (slave_index) {
        case 1:
          slave_addr = SLAVE_1;
          break;
        case 2:
          slave_addr = SLAVE_2;
          break;
        case 3:
          slave_addr = SLAVE_3;
          break;
        case 4:
          slave_addr = SLAVE_4;
          break;
        default:
          slave_addr = SLV_ERR;
          break;
      }

      if (slave_addr != SLV_ERR) {
        Wire.beginTransmission(slave_addr);
        Wire.write(command);
        byte error = Wire.endTransmission();

        if (error) {
          received_speeds[slave_index - 1] = spd_err;
          SlaveSendError[slave_index - 1] = distrib_err;
          SlaveReceiveError[slave_index - 1] = fb_err;
          continue;
        } else {
          SlaveSendError[slave_index - 1] = "OK";
        }

        Wire.requestFrom(slave_addr, 1);  // request 1 byte from the slave Arduino
        if (Wire.available() == 0) { // no response from slave
          received_speeds[slave_index - 1] = spd_err;
          SlaveReceiveError[slave_index - 1] = fb_err;
        } else if (Wire.available() > 0) {
          byte state = Wire.read();
          received_speeds[slave_index - 1] = String(state);
          SlaveReceiveError[slave_index - 1] = "OK";
        }
      }
    }

    for (unsigned int i = 0; i < 4; i++) {
      state_feedback += SlaveSendError[i] + ",";
      state_feedback += SlaveReceiveError[i] + ",";
      state_feedback += received_speeds[i] + ",";
    }
    Serial.println(state_feedback);

    // reset for next command
    state_feedback = fb_err;
    slave_addr = SLV_ERR;
    command = CMD_ERR;
    received_speeds = {spd_err, spd_err, spd_err, spd_err};
    SlaveSendError = {distrib_err, distrib_err, distrib_err, distrib_err};
    SlaveReceiveError = {fb_err, fb_err, fb_err, fb_err};
  }

  delay(6); // 6ms

}
