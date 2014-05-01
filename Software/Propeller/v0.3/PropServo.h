#ifndef _COSEQ_PROP_CONTROL_H_
#define _COSEQ_PROP_CONTROL_H_

#include "simpletools.h"
#include "servo.h"

#define DEFAULT_1_PIN 9
#define DEFAULT_2_PIN 8
#define DEFAULT_3_PIN 7
#define DEFAULT_4_PIN 6

#define stoprpm 700
#define minrpm 770
#define maxrpm 2000

#define MOTOR_FRONT 0
#define MOTOR_RIGHT 1
#define MOTOR_REAR 2
#define MOTOR_LEFT 3

class PropServo {
  public:
    PropServo();
    PropServo(uint8_t motor1pin, uint8_t motor2pin, uint8_t motor3pin, uint8_t motor4pin);
    
    //This is used to determine when the class is done initializing the ESCs
    //It becomes 1 once that is complete (so the calls of setThrust / updateServoRPM
    //will only occur from another class - to help with concurrency issues)
    //this takes at least 6 seconds
    static uint8_t PropServoStatus;
    
    static void setThrust(uint8_t motor, uint16_t thrust);
    static void updateServoRPM();
    
  private:
    static uint8_t prop1pin;
    static uint8_t prop2pin;
    static uint8_t prop3pin;
    static uint8_t prop4pin;
    static double_t delta;
    static uint16_t rpm[4];
    
    void calibrate();
};
#endif /* _COSEQ_PROP_CONTROL_H_ */