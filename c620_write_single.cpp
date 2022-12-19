#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg1;
MCP2515 mcp2515(5);

const short currentLowerLimit = -1024;
const short currentUpperLimit = 1024;

void setup() {

  Serial.begin(115200);
  while (!Serial) {};
  
  canMsg1.can_id  = 0x200;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x00;
  canMsg1.data[1] = 0x00;
  canMsg1.data[2] = 0x32;
  canMsg1.data[3] = 0xFA;
  canMsg1.data[4] = 0x26;
  canMsg1.data[5] = 0x8E;
  canMsg1.data[6] = 0xBE;
  canMsg1.data[7] = 0x86;

  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("Ready");
}

void loop() {

  short current = 500;
  current = constrain(current, currentLowerLimit, currentUpperLimit);
  
  byte currentHB = (current >> 8) & 0xFF;
  byte currentLB = current & 0xFF;

  canMsg1.data[0] = currentHB;
  canMsg1.data[1] = currentLB;

  Serial.print(current); Serial.print(" ");
  Serial.print(current, BIN); Serial.print(" ");
  Serial.print(currentHB, BIN); Serial.print(" ");
  Serial.print(currentLB, BIN); Serial.println(" ");

  mcp2515.sendMessage(&canMsg1);

//  Serial.println("Messages sent");
  
  delay(100);
}
