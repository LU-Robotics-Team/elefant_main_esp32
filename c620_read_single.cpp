#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>

struct can_frame canMsg;
MCP2515 mcp2515(5); // CS Pin -> 5 for ESP32


void setup() {
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");
    
//    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
//      Serial.print(canMsg.data[i],HEX);
//      Serial.print(" ");
//    }

    short posHB = canMsg.data[0] << 8;
    short posLB = canMsg.data[1];
    short pos = posHB | posLB;
    Serial.print(map(pos, 0, 8191, 0, 360));
    Serial.print(" ");

    short rpmHB = canMsg.data[2] << 8;
    short rpmLB = canMsg.data[3];
    short rpm = rpmHB | rpmLB;
    Serial.println(rpm);     
  }
}
