/*

Define PID object in a class, then use pointer does not work,
seems like the object is destroyed (out of scope) after the constructor

Solution
declare using dynamic memory (i.e. using new operator)
Ref: https://stackoverflow.com/questions/10080935/when-is-an-object-out-of-scope


25/11/2022
When I removed the serial prints, system went out of control... WHY??

*/

#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>
#include <PID_v1.h>

struct can_frame canMsg, canCmd;
MCP2515 mcp2515(5); // CS Pin -> 5 for ESP32

short motorStatusBuffer[4][2];

const short motorAcclLimit = 2000;
const short nominalAccl = 1000;

// double setpoint, feedback, control;
// const double Kp = 1;
// const double Ki = 0.1;
// const double Kd = 0;
// PID motor1PID(&feedback, &control, &setpoint, Kp, Ki, Kd, DIRECT);

class C620 {

  private:
    double setpoint, feedback, control;
    const double Kp = 1;
    const double Ki = 0.1;
    const double Kd = 0;
    PID* pidPtr = nullptr;

  public:
    int id;
    short position;
    short velocity;

    C620(int c620_id) {
      id = c620_id;

      pidPtr = new PID(&feedback, &control, &setpoint, Kp, Ki, Kd, DIRECT);
      pidPtr->SetOutputLimits(-16384, 16384);
      pidPtr->SetMode(AUTOMATIC);

      // motor1PID.SetOutputLimits(-16384, 16384);
      // motor1PID.SetMode(AUTOMATIC);
    }

    void read() {
      position = motorStatusBuffer[id - 0x201][0];
      velocity = motorStatusBuffer[id - 0x201][1];
    }

    void write(short rpm) {

      // PID
      setpoint = rpm;
      feedback = velocity;
      pidPtr->Compute();
      // motor1PID.Compute();
      
      // Serial.print(setpoint); Serial.print('\t');
      // Serial.print(feedback); Serial.print('\t');
      // Serial.println(control);

      short current = control;
      
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
  parseFeedback();
  frontLeftWheel.read();
  backLeftWheel.read();
  frontRightWheel.read();
  backRightWheel.read();

  frontLeftWheel.write(1000);
  backLeftWheel.write(1000);
  frontRightWheel.write(1000);
  backRightWheel.write(1000);

  Serial.print(frontLeftWheel.velocity); Serial.print('\t');
  Serial.print(backLeftWheel.velocity); Serial.print('\t');
  Serial.print(frontRightWheel.velocity); Serial.print('\t');
  Serial.print(backRightWheel.velocity); Serial.print('\t');

  Serial.println();
}
