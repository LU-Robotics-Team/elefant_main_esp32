#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>
#include <SerialTransfer.h>


/*

  Please check your CAN termination resistance should be around
  60 ohm, measure the resisitance at the CAN H and L terminal
  of the MCP2515, toggle the 120 ohm switches on C620.

  Reference: https://youtu.be/JPE42HSJxAk?t=244
  

*/

#define FORWARD 0
#define BACKWARD 1
#define LEFT 2
#define RIGHT 3
#define FORWARD_LEFT 4
#define FORWARD_RIGHT 5
#define BACKWARD_LEFT 6
#define BACKWARD_RIGHT 7
#define ROTATE_CW 8
#define ROTATE_CCW 9
#define STOP 10

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

struct can_frame canMsg, canCmd;
MCP2515 mcp2515(5); // CS Pin -> 5 for ESP32

short motorStatusBuffer[4][2];

class C620 {

  private:
    double feedback, control, setpoint;
    double Kp, Ki, Kd;
    int T;
    unsigned long lastTime;
    double totalErr, lastErr;
    int controlLimit;

  public:
    int id;
    short position;
    short velocity;

    C620(int c620_id) {
      id = c620_id;

      Kp = 1;
      Ki = 0.01;
      Kd = 0;

      T = 10;

      lastTime = 0;
      totalErr = 0;
      lastErr = 0;
      controlLimit = 10000;
    }

    void read() {

      position = motorStatusBuffer[id - 0x201][0];
      velocity = motorStatusBuffer[id - 0x201][1];

    }

    void write(short rpm) {

      // PID
      setpoint = rpm;
      feedback = velocity;

      unsigned long now = millis();

      if (now - lastTime >= T) {

        double err = setpoint - feedback;

        totalErr += err;
        totalErr = constrain(totalErr, -controlLimit, controlLimit);

        double deltaErr = err - lastErr;

        control = Kp*err + (Ki*T)*totalErr + (Kd/T)*deltaErr;

        control = constrain(control, -controlLimit, controlLimit);

        lastErr = err;
        lastTime = millis();
      }

      // Set motor
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

  Serial2.begin(9600);
  cmdPacket.begin(Serial2);

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

  // read joystick
  if (cmdPacket.available())
  {
    uint16_t recSize = 0;
    recSize = cmdPacket.rxObj(cmd, recSize);
  }

  // calculate drive speed
  const int deadband = 30;
  int driveMode;

  driveMode = STOP;

  if (cmd.leftTrig == 1 && cmd.rightTrig == 1) {
    if (abs(cmd.joyX) <= deadband && cmd.joyY >= deadband) driveMode = FORWARD;
    else if (abs(cmd.joyX) <= deadband && cmd.joyY <= -deadband) driveMode = BACKWARD;
    else if (abs(cmd.joyY) <= deadband && cmd.joyX >= deadband) driveMode = RIGHT;
    else if (abs(cmd.joyY) <= deadband && cmd.joyX <= -deadband) driveMode = LEFT;
    else if (cmd.joyX <= -deadband && cmd.joyY >= deadband) driveMode = FORWARD_LEFT;
    else if (cmd.joyX <= -deadband && cmd.joyY <= -deadband) driveMode = BACKWARD_LEFT;
    else if (cmd.joyX >= deadband && cmd.joyY >= deadband) driveMode = FORWARD_RIGHT;
    else if (cmd.joyX >= deadband && cmd.joyY <= -deadband) driveMode = BACKWARD_RIGHT;
  }
  else if (cmd.leftTrig == 1 && cmd.rightTrig == 0 && 
          abs(cmd.joyX) <= deadband && abs(cmd.joyY) <= deadband) driveMode = ROTATE_CW;
  else if (cmd.leftTrig == 0 && cmd.rightTrig == 1 && 
          abs(cmd.joyX) <= deadband && abs(cmd.joyY) <= deadband) driveMode = ROTATE_CCW;

  // Serial.print(cmd.joyX); Serial.print('\t');
  // Serial.print(cmd.joyY); Serial.print('\t');
  // Serial.print(cmd.left); Serial.print('\t');
  // Serial.print(cmd.leftTrig); Serial.print('\t');
  // Serial.print(cmd.rightTrig); Serial.print('\t');
  // Serial.println(driveMode);


  int nominalVel = 1000;
  int velMultiplier = 1;

  if (cmd.left == 0) velMultiplier = 2;
  else velMultiplier = 1;

  int targetWheelVel[4];

  switch (driveMode)
  {
    case FORWARD:
      targetWheelVel[0] = nominalVel * velMultiplier;
      targetWheelVel[1] = nominalVel * velMultiplier;
      targetWheelVel[2] = nominalVel * velMultiplier;
      targetWheelVel[3] = nominalVel * velMultiplier;
      break;

    case BACKWARD:
      targetWheelVel[0] = -nominalVel * velMultiplier;
      targetWheelVel[1] = -nominalVel * velMultiplier;
      targetWheelVel[2] = -nominalVel * velMultiplier;
      targetWheelVel[3] = -nominalVel * velMultiplier;
      break;

    default:
      targetWheelVel[0] = 0;    
      targetWheelVel[1] = 0;    
      targetWheelVel[2] = 0;    
      targetWheelVel[3] = 0;    
      break;
  }

  parseFeedback();
  frontLeftWheel.read();
  frontRightWheel.read();
  backLeftWheel.read();
  backRightWheel.read();

  frontLeftWheel.write(targetWheelVel[0]);
  backLeftWheel.write(targetWheelVel[1]);
  frontRightWheel.write(targetWheelVel[2]);
  backRightWheel.write(targetWheelVel[3]);

  // If I just run the above, PID will not work
  // until I add some printing or a delay. It works. WHY????

  // Serial.print(frontLeftWheel.velocity); Serial.print('\t');
  // Serial.print(backLeftWheel.velocity); Serial.print('\t');
  // Serial.print(frontRightWheel.velocity); Serial.print('\t');
  // Serial.print(backRightWheel.velocity); Serial.print('\t');

  // Serial.println();
  // delay(1);

}
