#include "PropServo.h"


uint8_t PropServo::PropServoStatus = 0;
double_t PropServo::delta = (maxrpm-minrpm)/1000.0;
uint16_t PropServo::rpm[4] = { stoprpm };

uint8_t  PropServo::prop1pin = DEFAULT_1_PIN;
uint8_t  PropServo::prop2pin = DEFAULT_2_PIN;
uint8_t  PropServo::prop3pin = DEFAULT_3_PIN;
uint8_t  PropServo::prop4pin = DEFAULT_4_PIN;

/**
 *           CW
 *          ( 0 )
 *            |
 *  CCW       |        CCW
 * ( 3 )--------------( 1 )
 *            |
 *            |
 *          ( 2 )
 *           CW
 **/

PropServo::PropServo(uint8_t motor1pin, uint8_t motor2pin, uint8_t motor3pin, uint8_t motor4pin){
  prop1pin = motor1pin;
  prop2pin = motor2pin;
  prop3pin = motor3pin;
  prop4pin = motor4pin;

  calibrate();
}
 
PropServo::PropServo(){
  calibrate();
}

/**
 *  Control method to set thrust to each motor
 *  motor: int, 0-3, selects the motor to set thrust
 *  thrust int, 0-1000, sets the thrust level of the motor.
 *  1 provides thrust, 0 turns motor off.
 **/
void PropServo::setThrust(uint8_t motor, uint16_t thrust){
  //770 is lowest thrust level while still spinning
  //2000 is max

  if(motor < 4 && motor >= 0){
    int thrustLevel = 0;
    if(thrust > 999) thrustLevel = maxrpm;
    else if(thrust <= 0) thrustLevel = stoprpm;
    else {
      thrustLevel = (int)(delta*thrust)+770;
    }

    rpm[motor] = thrustLevel;
  }
}

/**
 *  Internal method used to actually update the servo control all at once
 *  A new cog is automatically started to manage these PWM pins (it can run like 24 at once)
 **/
void PropServo::updateServoRPM(){
  servo_set(prop1pin, rpm[0]);
  servo_set(prop2pin, rpm[1]);
  servo_set(prop3pin, rpm[2]);
  servo_set(prop4pin, rpm[3]);
}

/**
 *  Calibrates the ESC to understand our control range.
 *  Only needs to be run once. But I don't know how volatile
 *  the ESCs are so we should run this on startup.
 *  
 *  The protocol for this was found in the documentation bundled
 *  with the ESCs
 **/
void PropServo::calibrate(){
  print("Calibrating ESCs...");
  servo_set(prop1pin, maxrpm);
  servo_set(prop2pin, maxrpm);
  servo_set(prop3pin, maxrpm);
  servo_set(prop4pin, maxrpm);
  pause(3000);
  servo_set(prop1pin, stoprpm);
  servo_set(prop2pin, stoprpm);
  servo_set(prop3pin, stoprpm);
  servo_set(prop4pin, stoprpm);
  pause(3000);
  print("done\n");
  //other classes are free to manage the props now
  PropServoStatus = 1;
}