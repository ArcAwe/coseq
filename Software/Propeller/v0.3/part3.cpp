/**

Main Code to run the COSEQ System.

Holland Gibson
May 2014

(USMA '14 Honors Project)

See readme for more information.
**/
#include "COSEQ.h"
#include "simpletools.h"
#include "servo.h"
#include "fdserial.h"
#include "Quadcmd.h"
#include "PropServo.h"


/**
 * This is the main program file.
 */
int main(void)
{
  //should help cogs print to terminal
    extern text_t *dport_ptr; // default debug port pointer gets reassigned to fdserial.
    simpleterm_close();
    dport_ptr = (fdserial_open(31,30,0,115200));

    uint8_t debug_mode = 0;
  
    COSEQ system = COSEQ();
    print("Starting COSEQ.\n");
    system.start(); 
  
    //all the threads are running on cogs, so we relax
    while(true){
        //sanity check with debug
        
        //this is on a pause 1000ms so it shouldn't interfere with anything (although it might)
        if(debug_mode == 1){
            Quadcmd_t cmd = COSEQ::xbcmd;
    
            print("CMD: %u\t", cmd.CMD_TYPE);
            print("%u\t", cmd.data_bytes[0]);
            print("%u\t", cmd.data_bytes[1]);
            print("%u\t", cmd.data_bytes[2]);
            print("%u\t", cmd.data_bytes[3]);
            print("%u\t", cmd.data_bytes[4]);
            print("%u",   cmd.data_bytes[5]);
            print("\n");
            
        } else if(debug_mode == 2) {
            print("%u\t", COSEQ::motor1debug);
            print("%u\t", COSEQ::motor2debug);
            print("%u\t", COSEQ::motor3debug);
            print("%u",   COSEQ::motor4debug);
            print("\n");
            
        } else if(debug_mode == 3) {
            while(COSEQ::mpu9150Lock == 1){
                //block
            }
            float p = COSEQ::spitch;
            float r = COSEQ::sroll;
            print("%f\t", p);
            print("%f\t", r);
            print("\n");
        }
        pause(1000);
    }
  //system.listEEPROM();
}

