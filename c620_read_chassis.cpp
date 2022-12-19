#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>

struct can_frame canMsg;
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

      // Serial.print(id, HEX); Serial.print('\t');
      // Serial.print(position); Serial.print('\t');
      // Serial.println(velocity);

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
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

}

void loop() {
  parseFeedback();
  frontLeftWheel.read();
  backLeftWheel.read();
  frontRightWheel.read();
  backRightWheel.read();

  for (int i=0; i<4; i++) {
    for (int j=0; j<2; j++) {
      Serial.print(motorStatusBuffer[i][j]); Serial.print('\t');
    }
  }

  Serial.println();

  delay(10);
}
