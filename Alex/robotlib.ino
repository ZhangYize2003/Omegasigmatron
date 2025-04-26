#include <AFMotor.h>

// Motor control
#define FRONT_LEFT   4 // M4 on the driver shield
#define FRONT_RIGHT  1 // M1 on the driver shield
#define BACK_LEFT    3 // M3 on the driver shield
#define BACK_RIGHT   2 // M2 on the driver shield

AF_DCMotor motorFL(FRONT_LEFT);
AF_DCMotor motorFR(FRONT_RIGHT);
AF_DCMotor motorBL(BACK_LEFT);
AF_DCMotor motorBR(BACK_RIGHT);

void move(int direction)
{
  int speed = 255;
  motorFL.setSpeed(speed);
  motorFR.setSpeed(speed);
  motorBL.setSpeed(speed);
  motorBR.setSpeed(speed);

  switch(direction)
    {
      case BACK:
        motorFL.run(BACKWARD);
        motorFR.run(BACKWARD);
        motorBL.run(FORWARD);
        motorBR.run(FORWARD); 
      break;
      case GO:
        motorFL.run(FORWARD);
        motorFR.run(FORWARD);
        motorBL.run(BACKWARD);
        motorBR.run(BACKWARD); 
      break;
      case CW:
        motorFL.run(FORWARD);
        motorFR.run(FORWARD);
        motorBL.run(FORWARD);
        motorBR.run(FORWARD); 
      break;
      case CCW:
        motorFL.run(BACKWARD);
        motorFR.run(BACKWARD);
        motorBL.run(BACKWARD);
        motorBR.run(BACKWARD); 
      break;
      case STOP:
      default:
        motorFL.setSpeed(0);
        motorFR.setSpeed(0);
        motorBL.setSpeed(0);
        motorBR.setSpeed(0);
    }
}

/*
 * Alex's movement commands. The start time of the movement is stored
 * using the millis() function.
 */
void forward()
{
  moveStartTime = millis();
  dir = (TDirection) FORWARD;
  move(FORWARD);
}

void backward()
{
  moveStartTime = millis();
  dir = (TDirection) BACKWARD;
  move(BACKWARD);
}

void ccw()
{
  moveStartTime = millis();
  move(CCW);
}

void cw()
{
  moveStartTime = millis();
  move(CW);
}

void stop()
{
  move(STOP);
}
