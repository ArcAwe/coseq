#include "COSEQ.h"

volatile uint8_t COSEQ::mpu9150Lock = 1; //start as 1 to make flight cog wait for initialization
volatile uint8_t COSEQ::serialLock = 0;

// volatile int16_t COSEQ::vax = 0;
// volatile int16_t COSEQ::vay = 0;
// volatile int16_t COSEQ::vaz = 0;
// volatile int16_t COSEQ::vgx = 0;
// volatile int16_t COSEQ::vgy = 0;
// volatile int16_t COSEQ::vgz = 0;
// volatile int16_t COSEQ::vmx = 0;
// volatile int16_t COSEQ::vmy = 0;
// volatile int16_t COSEQ::vmz = 0;

volatile float_t COSEQ::spitch = 0.0;
volatile float_t COSEQ::sroll = 0.0;

unsigned int COSEQ::flight_stack[COG_FLIGHT_STACKSIZE] = { 0 };
//unsigned int COSEQ::mpu9150_stack[COG_MPU9150_STACKSIZE] = { 0 };
unsigned int COSEQ::comms_stack[COG_SERIAL_STACKSIZE] = { 0 };

volatile uint32_t COSEQ::serialSample = 0;
volatile int16_t COSEQ::vForceApprox = -16384; //default to gravity

Quadcmd_t COSEQ::xbcmd;
//sem_t COSEQ::serialW;
volatile uint8_t COSEQ::serialHold = 0;

volatile uint16_t COSEQ::motor1debug = 0;
volatile uint16_t COSEQ::motor2debug = 0;
volatile uint16_t COSEQ::motor3debug = 0;
volatile uint16_t COSEQ::motor4debug = 0;

volatile float COSEQ::dyn_DT = 0.04;

//volatile sem_t COSEQ::serialMutex;
//volatile int8_t COSEQ::serialReadCount = 0;


COSEQ::COSEQ(){
     //print("Putting byte %c @ 0x804\n", 'A');
     //ee_putByte('A',0x804);
}


/*

    Starts the control threads on different cogs.
    
    This is the general scoping theme:
    
    First COG:  the initial, controller cog
    Second COG: Flight cog - reads accelerometer/gyro and sends commands to ESCs
    Third COG:  Reads MPU9150 in a consistent way - probably could be combined into second COG, 
                but we have the COGS available - started in flight cog
    Fourth COG: COMMO cog 
    Fifth COG:  i2c cog started automatically
    Sixth COG:  servo cog started automatically
    Seventh COG:SPI cog started automatically
    
*/
void COSEQ::start(){
    
//   void (*flightcog)(void*);
//   flightcog = &COSEQ::cog_flight;
    if(DEBUG_MODE_LEVEL > 0){
        print("Starting flight cog...\n");
    }
    
    //sem_init(&serialW, 1, 1);
    //sem_init(&serialMutex, 1, 1);
    
    uint8_t cog1s = cogstart(&cog_flight, NULL, flight_stack, sizeof(flight_stack));
    
    uint8_t cog3s = cogstart(&cog_comms, NULL, comms_stack, sizeof(comms_stack));
}

/*
    COG Function for flight
*/
void COSEQ::cog_flight(void *){


    //start the sensor
    if(DEBUG_MODE_LEVEL > 0){
        print("Starting MPU9150 Cog...\n");
    }
    //cogstart(&cog_mpu9150, NULL, mpu9150_stack, sizeof(mpu9150_stack));
    

    int16_t forceapp_m;
    
    float_t sampledPitch = 0.0;
    float_t sampledRoll = 0.0;
    
    PropServo motorControl = PropServo(); //use default pins
    //this takes a while - at least 6 seconds - you can tell by the "beep-beep-beep... beeeeeeep" when it is complete
    uint16_t motor1level = 0;
    uint16_t motor2level = 0;
    uint16_t motor3level = 0;
    uint16_t motor4level = 0;
    
    uint16_t thrustLevel = 0;
    int8_t pitchSet = 0;
    int8_t rollSet = 0;
    int8_t yawSet = 0;
    
    uint8_t commandNum = 0;
    
    while(true){
        if(timeout(TIMEOUT_US) > 0){
            //we've lost the signal. 
            //kill the motors to 10%
            //TODO: autoland - we could just use CMD_TYPE 2 and set thrust to a lower level...
            PropServo::setThrust(MOTOR_FRONT, 10);
            PropServo::setThrust(MOTOR_RIGHT, 10);
            PropServo::setThrust(MOTOR_REAR, 10);
            PropServo::setThrust(MOTOR_LEFT, 10);
            PropServo::updateServoRPM();
            
            //for debug purposes
            motor1debug = 10;
            motor2debug = 10;
            motor3debug = 10;
            motor4debug = 10;
            while(timeout(TIMEOUT_US) > 0){
                //wait for the signal to come back :(
            }
        }
        
        //readers/writers problem! - see wikipedia. I ignore the readcount and mutex because this is the only reading thread
        //sem_wait(&serialMutex);
        //readcount += 1;
        //if(readcount == 1){
        //sem_wait(&serialW);
        while(serialHold > 0){
            //block - wait for writing to be done
        }
        //Critical section
        commandNum = xbcmd.CMD_TYPE;
        if(commandNum == 1){
            motor1level = xbcmd.data_bytes[0]*4;
            motor2level = xbcmd.data_bytes[1]*4;
            motor3level = xbcmd.data_bytes[2]*4;
            motor4level = xbcmd.data_bytes[3]*4;
        } else if(commandNum == 2){
            thrustLevel = xbcmd.data_bytes[0]*4;
            pitchSet = xbcmd.data_bytes[1]-127;
            rollSet = xbcmd.data_bytes[2]-127;
            yawSet = xbcmd.data_bytes[3]-127;
        }
        
        sampledPitch = spitch;
        sampledRoll = sroll;
        
        //done
        //sem_wait(&serialMutex);
        //sem_post(&serialW);
        
        if(commandNum == 1){
            //we are good already
        } else if(commandNum == 2) {
            motor1level = thrustLevel;
            motor2level = thrustLevel;
            motor3level = thrustLevel;
            motor4level = thrustLevel;
            
            //+ pitch = tilt back
            //+ roll = tilt left
            float_t newRoll = (float) rollSet / 2.0; //half-degree increments
            float_t newPitch = (float) pitchSet / 2.0;
            float_t newYaw = yawSet;

            //order matters. We want to do yaw last
            adjustPitch(sampledPitch, newPitch, &motor1level, &motor3level);
            adjustRoll(sampledRoll, newRoll, &motor2level, &motor4level);
            adjustYaw(0.0, newYaw, &motor1level, &motor2level, &motor3level, &motor4level);
            
            motor1level = min(motor1level, thrustLevel*2); //if thrust is at 0 do nothing, limit the amount of change so it doesn't crash on takeoff
            motor2level = min(motor2level, thrustLevel*2);
            motor3level = min(motor3level, thrustLevel*2);
            motor4level = min(motor4level, thrustLevel*2);
        }
        
        PropServo::setThrust(MOTOR_FRONT, motor1level);
        PropServo::setThrust(MOTOR_RIGHT, motor2level);
        PropServo::setThrust(MOTOR_REAR, motor3level);
        PropServo::setThrust(MOTOR_LEFT, motor4level);
        PropServo::updateServoRPM();
        
        
        //for testing WITHOUT flying
        motor1debug = motor1level;
        motor2debug = motor2level;
        motor3debug = motor3level;
        motor4debug = motor4level;
        
        /*
        
        if(DEBUG_MODE_LEVEL > 1){
            print("gyroX = %d\n", gyroX);
            pause(40);
        }
        
        while(mpu9150Lock>0){
            //do nothing
            //this will also wait until it is initialized
        }
        pitch = spitch;
        roll = sroll;
        forceapp_m = vForceApprox;
        
        //use the complementary filter to see where we're at
        //ComplementaryFilter(accelX,accelY,accelZ,gyroX,gyroY,gyroZ,&pitch,&roll);
        
        //feed pitch and roll into leveling function
        adjustPitch(pitch, 0.0, &motor1level, &motor3level);
        adjustRoll(roll, 0.0, &motor2level, &motor4level);
        
        //adjusts the level of all the motors to maintain altitude
        adjustThrust(&motor1level, &motor2level, &motor3level, &motor4level, forceapp_m, 1.0);
        
        //set the motors
        PropServo::setThrust(MOTOR_FRONT, motor1level);
        PropServo::setThrust(MOTOR_RIGHT, motor2level);
        PropServo::setThrust(MOTOR_REAR, motor3level);
        PropServo::setThrust(MOTOR_LEFT, motor4level);
        PropServo::updateServoRPM();
        */
    }
}

/*
    Combines accelerometer and gyro sensors to get roll/pitch
    
    I use a complimentary filter - see http://www.pieter-jan.com/node/11
    
    TODO: add magnetometer sensor to add level of precision
*/
// int16_t COSEQ::combineAccelGyro(){
//  
// }

/*
    This method reads the sampled pitch, and adjusts the new thrust level to maintain the desired pitch
    Again, pitch only requires motors 1 and 3 to change because that is front and rear
*/
void COSEQ::adjustPitch(float_t sampledPitch, float_t desiredPitch, uint16_t* motor1level_in, uint16_t* motor3level_in){
//   if(DEBUG_MODE_LEVEL > 0){
//           print("Pitch = %f\n", sampledPitch);
//           pause(200);
//   }
    
    float_t chg = (sampledPitch - desiredPitch);
    
    float_t reqChange = 0.0;
    uint16_t motor1copy = *motor1level_in;
    uint16_t motor3copy = *motor3level_in;
    
    float_t motor1 = (float_t) motor1copy;
    float_t motor3 = (float_t) motor3copy;
    
    if(chg >= 0){
        reqChange = min(chg, 90.0); //cap the change
    } else {
        reqChange = max(chg, -90.0); //cap the change
    }
    
    reqChange = reqChange * 10;
    
    float_t levelimit = 0.0;
    
    float_t a = 0.0;
    float_t b = 0.0;
    
    if(reqChange >= 0.0){
        //we have to pitch down
        //levelimit = min(motor1 - reqChange, motor3 + reqChange - 1000.0);
        a = motor1 - reqChange;
        b = motor3 + reqChange;
        if(a < 0 && b > 1000){
            //this means we would make a motor output negative thrust, or over max - which our ESCs don't do (can't fly upside down!)
            //so we limit the change we make
            levelimit = max(-a, b-1000);
        } else if(a<0){
            levelimit = -a;
        } else if(b>1000){
            levelimit = b-1000;
        } else levelimit = 0;
        
        reqChange -= levelimit;

        motor1level_in = motor1level_in - (int16_t) reqChange;
        motor3level_in = motor3level_in + (int16_t) reqChange;
    } else {
        //we pitch up
        //levelimit = min(motor3 - reqChange, motor1 + reqChange - 1000.0);
        
        b = motor1 - reqChange; //subtracting a negative
        a = motor3 + reqChange;
        
        if(a < 0 && b > 1000){
            //this means we would make a motor output negative thrust, or over max - which our ESCs don't do (can't fly upside down!)
            //so we limit the change we make
            levelimit = max(-a, b-1000);
        } else if(a<0){
            levelimit = -a;
        } else if(b>1000){
            levelimit = b-1000;
        } else levelimit = 0;
        
        reqChange += levelimit;
        
        motor1level_in = motor1level_in - (int16_t) reqChange;//subtracting a negative
        motor3level_in = motor3level_in + (int16_t) reqChange;
    }
}

/*
    This method reads the sampled roll, and adjusts the new thrust level to maintain the desired pitch
    Again, pitch only requires motors 2 and 4 to change because that is right and left
*/
void COSEQ::adjustRoll(float_t sampledRoll, float_t desiredRoll, uint16_t* motor2level_in, uint16_t* motor4level_in){
//   if(DEBUG_MODE_LEVEL > 0){
//           print("Roll = %f\n", sampledRoll);
//           pause(200);
//   }
    
    float_t chg = (sampledRoll - desiredRoll);
    
    float_t reqChange = 0.0;
    
    uint16_t motor2copy = *motor2level_in;
    uint16_t motor4copy = *motor4level_in;
    
    float_t motor2 = (float_t) motor2copy;
    float_t motor4 = (float_t) motor4copy;
    
    float_t a = 0.0;
    float_t b = 0.0;
    
    if(chg >= 0){
        reqChange = min(chg, 90.0); //cap the change
    } else {
        reqChange = max(chg, -90.0); //cap the change
    }
    
    reqChange = reqChange * 10;
    
    float_t levelimit = 0.0;
    
    if(reqChange <= 0.0){
        //we have to roll left (CCW)
        a = motor4 + reqChange; //adding a negative
        b = motor2 - reqChange;
        
        if(a < 0 && b > 1000){
            //this means we would make a motor output negative thrust, or over max - which our ESCs don't do (can't fly upside down!)
            //so we limit the change we make
            levelimit = max(-a, b-1000);
        } else if(a<0){
            levelimit = -a;
        } else if(b>1000){
            levelimit = b-1000;
        } else levelimit = 0;

        reqChange += levelimit;

        motor2level_in = motor2level_in - (int16_t) reqChange; //subtracting a negative
        motor4level_in = motor4level_in + (int16_t) reqChange;

    } else {
        //we roll right (CW)
        b = motor4 + reqChange;
        a = motor2 - reqChange;
        
        if(a < 0 && b > 1000){
            //this means we would make a motor output negative thrust, or over max - which our ESCs don't do (can't fly upside down!)
            //so we limit the change we make
            levelimit = max(-a, b-1000);
        } else if(a<0){
            levelimit = -a;
        } else if(b>1000){
            levelimit = b-1000;
        } else levelimit = 0;

        reqChange -= levelimit;

        motor2level_in = motor2level_in - (int16_t) reqChange;
        motor4level_in = motor4level_in + (int16_t) reqChange;
    }
}

/*
    This is a little different - we don't need to keep track of what direction we are currently pointing,
    we just need to adjust our yaw to get there.
    
    A positive yaw is counter-clockwise
    
    TODO: use readings from (magnetometer compass) to use more precise (without drift)
*/
void COSEQ::adjustYaw(float_t sampledHeading, float_t desiredHeading, uint16_t* motor1level_in, uint16_t* motor2level_in, 
            uint16_t* motor3level_in, uint16_t* motor4level_in){
    
    float_t reqChange = sampledHeading - desiredHeading;
    
    float_t adjust = reqChange * 10;
    
    float_t a,b,levelimit, motor1, motor2, motor3, motor4;
    
    motor1 = (float) *motor1level_in;
    motor2 = (float) *motor2level_in;
    motor3 = (float) *motor3level_in;
    motor4 = (float) *motor4level_in;
    
    if(adjust <= 0.0){
        //yaw to right
        //spin up 2/4
        a = min(motor1 + adjust, motor3 + adjust); //adding a negative
        b = max(motor2 - adjust, motor4 - adjust);
        
        if(a < 0 && b > 1000){
            //this means we would make a motor output negative thrust, or over max - which our ESCs don't do (can't fly upside down!)
            //so we limit the change we make
            levelimit = max(-a, b-1000);
        } else if(a<0){
            levelimit = -a;
        } else if(b>1000){
            levelimit = b-1000;
        } else levelimit = 0;
        
        adjust += levelimit;
        
        *motor1level_in += adjust;
        *motor3level_in += adjust;
        *motor2level_in -= adjust;
        *motor4level_in -= adjust;
    
    } else {
        //yaw to left
        //spin up 1/3
        b = max(motor1 + adjust, motor3 + adjust);
        a = min(motor2 - adjust, motor4 - adjust);
        
        if(a < 0 && b > 1000){
            //this means we would make a motor output negative thrust, or over max - which our ESCs don't do (can't fly upside down!)
            //so we limit the change we make
            levelimit = max(-a, b-1000);
        } else if(a<0){
            levelimit = -a;
        } else if(b>1000){
            levelimit = b-1000;
        } else levelimit = 0;
        
        adjust += levelimit;
        
        *motor1level_in += adjust;
        *motor3level_in += adjust;
        *motor2level_in -= adjust;
        *motor4level_in -= adjust;
    }
}

/*
    Adjusts the thrust of all the motors to increase or decrease altitude
*/
void COSEQ::adjustThrust(uint16_t* motor1level_in, uint16_t* motor2level_in, uint16_t* motor3level_in, uint16_t* motor4level_in, 
        int16_t grav, float_t altitude){
    uint16_t motor1 = *motor1level_in;
    uint16_t motor2 = *motor2level_in;
    uint16_t motor3 = *motor3level_in;
    uint16_t motor4 = *motor4level_in;

    
    int32_t g_meas = (grav+(16384.0*altitude)) / 20.0;
    
    uint16_t minMotor = min(motor1, min(motor2, min(motor3, motor4)));
    uint16_t maxMotor = max(motor1, max(motor2, max(motor3, motor4)));
    
    int32_t newMin = minMotor - g_meas; // < 0.0;
    int32_t newMax = maxMotor - g_meas; //> 1000.0;
    
    if(abs(newMin) > abs(newMax - 1000.0)){
        if(newMin < 0.0){
            g_meas = g_meas + newMin;
        }
    } else {
        if(newMax > 1000.0){
            g_meas = g_meas - newMax;
        }
    }
    
    //we need to increase thrust
    *motor1level_in = motor1 - g_meas;
    *motor2level_in = motor2 - g_meas;
    *motor3level_in = motor3 - g_meas;
    *motor4level_in = motor4 - g_meas;
    
}


/*
    Complementary filter curtesy of Pieter-Jan Van de Maele
    http://www.pieter-jan.com/node/11
    
    I return the sum of the acceleration in order to try to gauge the lift
*/
int16_t COSEQ::ComplementaryFilter(short accX, short accY, short accZ, short gyrX, short gyrY, short gyrZ, float_t *roll, 
                float_t *pitch) {
        float_t pitchAcc, rollAcc;                           
 
        // Integrate the gyroscope data -> int(angularSpeed) = angle
        *pitch += ((float_t)gyrX / 65.536) * dyn_DT; // Angle around the X-axis
        *roll -= ((float_t)gyrY / 65.536) * dyn_DT;  // Angle around the Y-axis
 
        // Compensate for drift with accelerometer data if !bullshit
        // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
        int forceMagnitudeApprox = abs(accX) + abs(accY) + abs(accZ);
        if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
        {
    // Turning around the X axis results in a vector on the Y-axis
                pitchAcc = atan2f((float_t)accY, (float_t)accZ) * 180 / M_PI;
                *pitch = *pitch * 0.98 + pitchAcc * 0.02;
 
    // Turning around the Y axis results in a vector on the X-axis
                rollAcc = atan2f((float_t)accX, (float_t)accZ) * 180 / M_PI;
                *roll = *roll * 0.98 + rollAcc * 0.02;
        }
        
//           if(DEBUG_MODE_LEVEL > 0){
//               if (serialSample == 0) {
//                   print("roll = %f\n", *roll);
//                   pause(50);
//               } else {
//                   serialSample += 1;
//               }
//           }

        return accX+accY+accZ;
}

/*
    Returns pitch as degrees above horizon
    
    Treating motor 1 as front, and 3 as rear
*/
// int16_t COSEQ::getPitch(uint16_t gyroX, uint16_t gyroY, uint16_t gyroZ, uint16_t accelX, uint16_t accelY, uint16_t accelZ){
//  //sign the ints
//  int16_t gx = (int16_t gyroX);
//  int16_t gy = (int16_t gyroY);
//  int16_t gz = (int16_t gyroZ);
//  
//  int16_t ax = (int16_t accelX);
//  int16_t ay = (int16_t accelY);
//  int16_t az = (int16_t accelZ);
//  
//  
// }


/*
    MPU9150 Collection COG
*/
// void COSEQ::cog_mpu9150(void *){
//     MPU6050 mpu9150 = MPU6050(MPU6050_DEFAULT_ADDRESS, MPU9150_SDA_PIN, MPU9150_SCL_PIN);
//     while(! mpu9150.poll()) pause(100); //wait for device to respond
//     mpu9150.initialize();
//     pause(100);//give the I2C a chance to breathe
// 
//     //forward volatile ints to mpu9150
//     int16_t nvax, nvay, nvaz, nvgx, nvgy, nvgz, nvmx, nvmy, nvmz;
//     
//     int16_t accelX;
//     int16_t accelY;
//     int16_t accelZ;
//     int16_t gyroX;
//     int16_t gyroY;
//     int16_t gyroZ;
//     int16_t magX;
//     int16_t magY;
//     int16_t magZ;
//     
//     float_t newpitch;
//     float_t newroll;
//     
//     int countCyc = 0;
// 
//     while(1==1){   
//         countCyc = CNT;
//         
//         mpu9150.getMotion9(&nvax, &nvay, &nvaz, &nvgx, &nvgy, &nvgz, &nvmx, &nvmy, &nvmz);
// 
//         accelX  = nvax + MPU9150_CAL_ACCEL_X;
//         accelY  = nvay + MPU9150_CAL_ACCEL_Y;
//         accelZ  = nvaz + MPU9150_CAL_ACCEL_Z;
//         gyroX   = nvgx + MPU9150_CAL_GYRO_X;
//         gyroY   = nvgy + MPU9150_CAL_GYRO_Y;
//         gyroZ   = nvgz + MPU9150_CAL_GYRO_Z;
// 
//         vForceApprox = ComplementaryFilter(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, &newpitch, &newroll);
//         
//         
//         mpu9150Lock = 1;
//         spitch = newpitch;
//         sroll = newroll;
//         mpu9150Lock = 0;
//         
//         pause(20);
//         
//         dyn_DT = getSeconds(-countCyc + CNT);//maybe this will be incredibly fast, who knows...
//         
//         //pause(20); // wrong -> //20ms sample rate (50Hz) //a change here necessitates a change in the dt definition see complementaryFilter
//     }
// }

/*
    Returns the thrust-level to maintain current altitude by slowly increasing thrust until it gets airborne.
    
    This would be useful to export into a register to store on start up.
    
    Returns the 0-1000 thrust level
    
    !!!NOT IMPLEMENTED YET!!!
*/
uint16_t COSEQ::findNeutralThrust(){
    return 330;
}

//both i2c and serial in one place so they don't overlap (hopefully)
void COSEQ::cog_comms(void *){
    Quadcmd_t cmd;
    XbeeSerial xbs;
    
    int16_t nvax, nvay, nvaz, nvgx, nvgy, nvgz, nvmx, nvmy, nvmz;
    int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ; //, nvmx, nvmy, nvmz;
    float_t newpitch = 0.0;
    float_t newroll = 0.0;
    uint32_t countCyc = 0;
    
    MPU6050 mpu9150 = MPU6050(MPU6050_DEFAULT_ADDRESS, MPU9150_SDA_PIN, MPU9150_SCL_PIN);
    while(! mpu9150.poll()) pause(100); //wait for device to respond
    mpu9150.initialize();
    pause(100); //take a breath
    
    while(true){
        countCyc = CNT;
        //readers-writers problem! - just two threads though, so a simple semaphore works
        
        //oddly enough, the serial driver conflicts with the simpleI2C driver when run in seperate cogs. The full i2c.h is 
        //too big to fit on a multi-controller so we just have to make due and do everything here at once - shouldn't be 
        //the end of the world if and I mean **IF** we don't lose our serial connection. 
        
        //To account for a loss in a serial connection - we set the timeout low, and we use our auto-landing sequence
        cmd = xbs.readCommands();
        mark(); //reset the timeout
        
        mpu9150.getMotion9(&nvax, &nvay, &nvaz, &nvgx, &nvgy, &nvgz, &nvmx, &nvmy, &nvmz);
        
        accelX  = nvax + MPU9150_CAL_ACCEL_X;
        accelY  = nvay + MPU9150_CAL_ACCEL_Y;
        accelZ  = nvaz + MPU9150_CAL_ACCEL_Z;
        gyroX   = nvgx + MPU9150_CAL_GYRO_X;
        gyroY   = nvgy + MPU9150_CAL_GYRO_Y;
        gyroZ   = nvgz + MPU9150_CAL_GYRO_Z;
        
        vForceApprox = ComplementaryFilter(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, &newpitch, &newroll);
        
        //sem_wait(&serialW);
        while(serialHold > 0){
            //block - wait until copying is done
        }
        serialHold = 1;
        //critical section
            xbcmd = cmd;
            spitch = newpitch;
            sroll = newroll;
        //done
        //sem_post(&serialW);
        serialHold = 0;
        
        cmd = xbs.blankCmd();
        
        pause(20);//we can't sample much faster than this
        
        if(countCyc < CNT){
          dyn_DT = getSeconds(-countCyc + CNT);//maybe this will be incredibly fast, who knows...
        }//ignore the 32bit int overflow
    }
}

float COSEQ::getSeconds(unsigned int cycles){
    return (float)cycles / (float)CLKFREQ;
}

/*
    Returns the min
*/
int16_t COSEQ::min(int16_t a, int16_t b){
    if(a < b){
        return a;
    } else {
        return b;
    }
}

/*
    Returns the max
*/
int16_t COSEQ::max(int16_t a, int16_t b){
    if(a > b){
        return a;
    } else {
        return b;
    }
}

/*
    Returns the min
*/
uint16_t COSEQ::min(uint16_t a, uint16_t b){
    if(a < b){
        return a;
    } else {
        return b;
    }
}

/*
    Returns the max
*/
uint16_t COSEQ::max(uint16_t a, uint16_t b){
    if(a > b){
        return a;
    } else {
        return b;
    }
}

/*
    Returns the min
*/
float_t COSEQ::min(float_t a, float_t b){
    if(a < b){
        return a;
    } else {
        return b;
    }
}

/*
    Returns the max
*/
float_t COSEQ::max(float_t a, float_t b){
    if(a > b){
        return a;
    } else {
        return b;
    }
}

/*
    Lists the EEPROM memory values
*/
// void COSEQ::listEEPROM(){
// 
//  unsigned char buffer;
//  int address = 0x7FFF;
//  while(address < 0x7FFF * 2){
//      buffer = ee_getByte(address);
//      print("%u: ", address);
//      print("%u\n", buffer);
//      address = address + 1;
//  }
// }