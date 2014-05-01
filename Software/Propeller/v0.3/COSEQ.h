/**

  COSEQ.h - This combines all the libraries for a single controller class
  
  25APR14
  v0.1
  
**/

#ifndef _COSEQ_H_
#define _COSEQ_H_

//includes
#include "MPU6050.H"
#include "simpletools.h"
#include "PropServo.h"
#include "quadcmd.h"
#include "fdserial.h" //for debugging - printing to serial from other cogs
#include <semaphore.h>

//definitions:
#define MaxEepromAddress  0x7FFF // 32,767
#define MPU9150_SDA_PIN   13
#define MPU9150_SCL_PIN   12

//Calibration values determined empirically
//they are always added to the read value to get to 0
//As in gyro x sample + 147 = ~0
//The Accelerometer values mainly account for assembly not being perfectly level
//Accel z sample + XXX = ~-65536 (value of gravity in the negative Z direction)
#define MPU9150_CAL_GYRO_X  147
#define MPU9150_CAL_GYRO_Y  -62
#define MPU9150_CAL_GYRO_Z  237

#define MPU9150_CAL_ACCEL_X   -126
#define MPU9150_CAL_ACCEL_Y   -439
#define MPU9150_CAL_ACCEL_Z   -354

//'Cushion' calibrations
//These sensors are +/- 2G (as I set them up in the MPU6050::initialize()
//so these numbers are the range of values that represent '0' to reduce jitter
//when the device is at a stand-still
//The accelerometer has much more jitter than the gyro, but the gyro drifts over time
//These are always positive
#define MPU9150_CUSH_GYRO_X   222
#define MPU9150_CUSH_GYRO_Y   166
#define MPU9150_CUSH_GYRO_Z   69

#define MPU9150_CUSH_ACCEL_X  756
#define MPU9150_CUSH_ACCEL_Y  476
#define MPU9150_CUSH_ACCEL_Z  1048

//Math definitions - included already
//#define M_PI 3.14159265359
#define dt 0.04 

#define COG_FLIGHT_STACKSIZE 340
#define COG_MPU9150_STACKSIZE 340
#define COG_SERIAL_STACKSIZE 400

#define TIMEOUT_US 10000000 //timeout for losing serial controls in us (1000us = 1ms) (80ms = 80000us or two missed periods in serial)

#define DEBUG_MODE_LEVEL 0 //0 == no debug, 1+ == debugging - this probably screws up the xbee commo
//all serial printing with debugging requires a pause if it is in a loop - if the serial bus gets full, the i2c bus to the 
// mpu9150 dies

class COSEQ {
  public:
    COSEQ();
    
    void start();
    
    //avionics
    int16_t getPitch(uint16_t gyroX, uint16_t gyroY, uint16_t gyroZ, uint16_t accelX, uint16_t accelY, uint16_t accelZ);
    
    static void adjustPitch(float_t sampledPitch, float_t desiredPitch, uint16_t* motor1level, uint16_t* motor3level);
    static void adjustRoll(float_t sampledRoll, float_t desiredRoll, uint16_t* motor2level_in, uint16_t* motor4level_in);
    static void adjustYaw(float_t sampledHeading, float_t desiredHeading, uint16_t* motor1level_in, uint16_t* motor2level_in, 
      	    uint16_t* motor3level_in, uint16_t* motor4level_in);
      	
    static void adjustThrust(uint16_t* motor1level_in, uint16_t* motor2level_in, uint16_t* motor3level_in, 
    		uint16_t* motor4level_in, int16_t grav, float_t altitude);
    
    uint16_t findNeutralThrust();
    
//    void listEEPROM();

	static volatile uint8_t serialLock;
	static volatile uint8_t mpu9150Lock;
	
	static volatile uint32_t serialSample;
	static volatile int16_t vForceApprox;
	
	static int16_t min(int16_t a, int16_t b);
	static int16_t max(int16_t a, int16_t b);
	
	static uint16_t min(uint16_t a, uint16_t b);
	static uint16_t max(uint16_t a, uint16_t b);
	
	static float_t min(float_t a, float_t b);
	static float_t max(float_t a, float_t b);
	
	volatile static uint16_t motor1debug;
	volatile static uint16_t motor2debug;
	volatile static uint16_t motor3debug;
	volatile static uint16_t motor4debug;
	
	static volatile float_t spitch, sroll;
	
	static volatile float dyn_DT;
	
	static float getSeconds(unsigned int cycles);

    static Quadcmd_t xbcmd;

  private:
    //locks
    
  
    //cog 1
    //static volatile int flight_t, flight_n;                     // Global vars for cogs to share
    //stack array with 40 longs (required) + extra for local variables and calculations. 
    //Be liberal with the extra memory, and use testing to pare it down after your prototyping is done
    //COGs have 2KB memory space and they all share the 32K hub mem
     //avionics   
    static unsigned int flight_stack[COG_FLIGHT_STACKSIZE];                  // Stack vars for cog 1
    static void cog_flight(void *);
    static int16_t ComplementaryFilter(short accX, short accY, short accZ, short gyrX, short gyrY, short gyrZ, 
        float_t *pitch, float_t *roll);

    
    //cog 2
    //sensor readings -- only let the mpu9150 COG change them!! Reading them won't create a concurrency
    //issue
    //static volatile int16_t vax, vay, vaz, vgx, vgy, vgz, vmx, vmy, vmz;
    
    //static unsigned int mpu9150_stack[COG_MPU9150_STACKSIZE]; 
    //static void cog_mpu9150(void *);
    //cog 3
    //Serial communication - updates the command structure
    //static Quadcmd_t xbcmd; //can't compile with them volatile
    //static sem_t serialW; //, serialMutex;
    static volatile uint8_t serialHold;
    //static volatile int8_t serialReadCount;
    static unsigned int comms_stack[COG_SERIAL_STACKSIZE]; 
    static void cog_comms(void *);
    
    //avionics


    //mpu reading
};


#endif /* _COSEQ_H_ */