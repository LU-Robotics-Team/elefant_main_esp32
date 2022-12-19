#include <Arduino.h>
#include <SerialTransfer.h>

SerialTransfer cmdPacket; // DOUT -> 16, take 3.3 V power

struct COMMAND {
  int joyX;
  int joyY;
  int joyButton;
  int up;
  int down;
  int left;
  int right;
  int leftTrig;
  int rightTrig;
} cmd;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600);
  cmdPacket.begin(Serial2);

}

void loop() {

  if (cmdPacket.available())
  {
    uint16_t recSize = 0;

    recSize = cmdPacket.rxObj(cmd, recSize);
    Serial.print(cmd.joyX); Serial.print('\t');
    Serial.print(cmd.joyY); Serial.print('\t');
    Serial.print(cmd.joyButton); Serial.print('\t');
    Serial.print(cmd.up); Serial.print('\t');
    Serial.print(cmd.down); Serial.print('\t');
    Serial.print(cmd.left); Serial.print('\t');
    Serial.print(cmd.right); Serial.print('\t');
    Serial.print(cmd.leftTrig); Serial.print('\t');
    Serial.print(cmd.rightTrig);
    Serial.println();
  }
}