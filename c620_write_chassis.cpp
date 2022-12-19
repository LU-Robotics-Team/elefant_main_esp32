#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>

/*

  Please check your CAN termination resistance should be around
  60 ohm, measure the resisitance at the CAN H and L terminal
  of the MCP2515, toggle the 120 ohm switches on C620.

  Reference: https://youtu.be/JPE42HSJxAk?t=244
  

*/

struct can_frame canMsg, canCmd;
MCP2515 mcp2515(5); // CS Pin -> 5 for ESP32

short motorStatusBuffer[4][2];

class C620 {
  public:
    int id;
    short position;
    short velocity;

    C620(int c620_id) {
      id = c620_id;
    }

    void read() {

      position = motorStatusBuffer[id - 0x201][0];
      velocity = motorStatusBuffer[id - 0x201][1];

      Serial.print(id, HEX); Serial.print('\t');
      Serial.print(position); Serial.print('\t');
      Serial.println(velocity);

    }

    void write(short accl) {

      short current = constrain(accl, -16384, 16384);
      
      byte currentHB = (current >> 8) & 0xFF;
      byte currentLB = current & 0xFF;

      if (id <= 0x204) {
        canCmd.can_id = 0x200;
      }
      else {
        canCmd.can_id = 0x1FF;
      }

      if (id == 0x201 || id == 0x205) {
        canCmd.data[0] = currentHB;
        canCmd.data[1] = currentLB;
      }
      else if (id == 0x202 || id == 0x206) {
        canCmd.data[2] = currentHB;
        canCmd.data[3] = currentLB;        
      }
      else if (id == 0x203 || id == 0x207) {
        canCmd.data[4] = currentHB;
        canCmd.data[5] = currentLB;        
      }
      else if (id == 0x204 || id == 0x208) {
        canCmd.data[6] = currentHB;
        canCmd.data[7] = currentLB;        
      }

      mcp2515.sendMessage(&canCmd);

    }
};


void parseFeedback() {

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {

    short posHB = canMsg.data[0] << 8;
    short posLB = canMsg.data[1];
    short pos = posHB | posLB;
    pos = map(pos, 0, 8191, 0, 360);

    short rpmHB = canMsg.data[2] << 8;
    short rpmLB = canMsg.data[3];
    short rpm = rpmHB | rpmLB;

    motorStatusBuffer[canMsg.can_id - 0x201][0] = pos;
    motorStatusBuffer[canMsg.can_id - 0x201][1] = rpm;
  
  }

}

C620 frontLeftWheel(0x201);
C620 backLeftWheel(0x202);
C620 frontRightWheel(0x203);
C620 backRightWheel(0x204);

void setup() {

  Serial.begin(115200);

  canCmd.can_id  = 0x00;
  canCmd.can_dlc = 8;
  canCmd.data[0] = 0x00;
  canCmd.data[1] = 0x00;
  canCmd.data[2] = 0x00;
  canCmd.data[3] = 0x00;
  canCmd.data[4] = 0x00;
  canCmd.data[5] = 0x00;
  canCmd.data[6] = 0x00;
  canCmd.data[7] = 0x00;
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

}

void loop() {
  // parseFeedback();
  // frontLeftWheel.read();
  // frontRightWheel.read();

  frontLeftWheel.write(800);
  backLeftWheel.write(800);
  frontRightWheel.write(-800);
  backRightWheel.write(-800);

}
