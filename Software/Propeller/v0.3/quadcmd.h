#ifndef __QUADCMD_H
#define __QUADCMD_H

//Number of ms until connection is assumed dropped
#define COMMAND_TIMEOUT = 1000
//Number of seconds (not ms) until we automatically land
#define AUTOLAND_TIMEOUT = 35
//Number of Kill Now packets we receive in a row to shutdown power
#define SHUTDOWN_PACKET_MARGIN = 5
/*
  Defines the CMD structure:
  
  Note: These take the structure of 8bit bytes, so we use 
  unsigned chars. 

  If the time from the last command exceeds COMMAND_TIMEOUT; we default to
  a static hover. If the time from the last command exceeds 
  AUTOLAND_TIMEOUT or the battery reaches a dangerously low level; we 
  execute an autoland sequence and shut down.

  CMD_TYPE CANNOT BE 0 - this is used to check for validity
  of data
  
  This is similar to UDP - fast, valid, but no handshaking with the device (for now)

  General Structure - 8 Bytes:
  [0] CMD_TYPE
  [1-6] DATA_BYTES
  [7] CHECKSUM 

  CHECKSUM is a very simple addition - Xbee does its own checks on each individual byte, we just want to 
  make sure we didn't lose any whole packets == SUM[0-7] & 0xFF

  CMD1: Manual Control
       Control all rotors individually
  [0] 1
  [1] 0-255 Rotor1 speed 0-100%
  [2] 0-255 Rotor2 speed
  [3] 0-255 Rotor3 speed
  [4] 0-255 Rotor4 speed
  [5] 0
  [6] 0
  [8] CHECKSUM


  CMD2: Vector Control - manual thrust
      This tells the quacdopter to hold a specific orinetation (pitch/roll/yaw) @ given thrust
  [0] 2
  [1] 0-255 Thrust
  [2] 0-255 pitch vector in 0.5 degree increments, absolute
        (a) computed by (raw byte - 127)/2 = pitch 
        (b) computed range is [-63.5,64]
        (b) vector of 0 degrees (raw = 127) = hold forward/backward pitch
        (c) positive value = increase pitch -> tilt back and vice versa
  [3] 0-255 roll vector, 0.5 degree increments, absolute
        (a) Positive (over 127) is roll-left (CCW)
  [4] 0-255 yaw vector, relative, 1 degree increments, + = yaw right (CW from above)
        (a) this should be interpreted as a desired change, as in; a -2 would mean turn 2 degrees
            left, even though that can't be accomplished in one packet's time 
        (b) this is an attempt: a -100 will mean a faster turn than a 3
        (c)[-127,128] computed
  [5] 0
  [6] 0
  [7] checksum


  CMD3: Hold Command
      This tells the quacdopter to hold specific relative velocities
  [0] 3
  [1] 0-200 value-100 = (delta) for vertical lift delta, (0)=altitude hold
  [2] 0-180 heading in 2 degree increments
  [3] 0-255 horizontal speed (0)=hold (255)=max lateral speed
  [4] 0
  [5] 0
  [6] 0
  [7] checksum

  CMD4: Waypoint Set (relative)
    Tells the copter to set a new waypoint and navigate to it
  [0] 4
  [1] 0-180 heading in 2 degree increments
  [2] 0-255 horizontal distance to travel in meters (approx)
  [3] 0-255 value-128=(-128-127) vertical distance to travel
  [4] 0-1 
    (0) override/clear queue and set as new coordinate
    (1) add to waypoint queue
  [5] waypoint num, upper 8 bits
  [6] waypoint num, lower 8 bits
  [7] checksum

  ***CMD5: Waypoint Set (GPS) - unimplemented
    Tells the copter to set a new waypoint and navigate to it.
    To conserve space we send GPS coordinates as relative change to the
      'master home' address coded at compilation time - see GPS notes below
  [0] 5
  [1] Lat part 1
  [2] Lat part 2
  [3] Lon part 1
  [4] Lon part 2
  ***

  CMD6: Land
  [0] 6
  [1] 0-255 Distance from ground in Meters
  [2] 0-200 Landing speed in 1/5 Meters/Sec; (50)=10 M/s
    (0) = default landing speed
  [3] 0-2 Shutdown Mode after ground landing detected
    (0) Halt, Motors to 0, accept new commands
    (1) Halt, Motors to 0, safely shut down power to all systems
    (2) Shutdown NOW - no landing sequence executed, all other 
        variables ignored. For safety, this is only accepted if the same 
        command is received SHUTDOWN_PACKET_MARGIN times.
  [4] ### UNUSED
  [5] checksum
*/

/*
  GPS - How we use it
  Our goal is a corrected-accurate location.
*/



#include "simpletools.h"                      // Include simple tools
#include "fdserial.h"

// Definitions
#define RX_PIN 11
#define TX_PIN 10
#define BAUD_RATE 115200
#define XBEE_CONF_CONN_WAIT 10 //Timeout while Connecting or Reconnecting

typedef struct {
    volatile uint8_t CMD_TYPE;
    volatile uint8_t data_bytes[6];
} Quadcmd_t;

class XbeeSerial {
    public:
        XbeeSerial();
        
        Quadcmd_t readCommands();
        Quadcmd_t blankCmd();
        
    private:
        fdserial *xbee;
};




#endif