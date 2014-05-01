#include "quadcmd.h"

XbeeSerial::XbeeSerial(){
    xbee = fdserial_open(RX_PIN,TX_PIN,0,BAUD_RATE);
    
    //Empties the Recieving and Transmitting Buffers
    fdserial_rxFlush(xbee);
    fdserial_txFlush(xbee);

    writeChar(xbee, CLS); 
    fdserial_txFlush(xbee);
}

Quadcmd_t XbeeSerial::readCommands(){
    Quadcmd_t readCmd;

    //readCmd.CMD_TYPE = 0;
    
    bool complete = false;
    
    uint8_t c;
    uint8_t count = 0;

    uint16_t cstotal = 0;
    uint8_t cksum = 0;


    
//     uint8_t type = 0;
    
    while(!complete){
        c = fdserial_rxChar(xbee); //blocks and sends sets of 4ytes       
        count = count + 1;

        //fdserial_txChar(dport_ptr, c);

        //print("%u:",count);
        //print("%u\n",c);
        //print("CMD:%u\n",readCmd)
        
        if(count == 1){
            //beginning of command
            if(c == 1 || c == 2){
                //type1
                readCmd.CMD_TYPE = c;
            } else {
                count = 0;
            }
        } else if(count<8) {
            //COMMAND PORTION
            //This is where commands are read
            readCmd.data_bytes[count-2] = c;
//             if(readCmd.CMD_TYPE == 1) {
//                 if(count == 2){
//                     readCmd.motorspeed[0] = c;
//                 } else if(count == 3){
//                     readCmd.motorspeed[1] = c;
//                 } else if(count == 4){
//                     readCmd.motorspeed[2] = c;
//                 } else if(count == 5){
//                     readCmd.motorspeed[3] = c;
//                 } else if(count > 5){
//                     //ignore these bytes
//                 }
//             else if(readCmd.CMD_TYPE == 2 {
//                 if(count == 2){
//                     //thrust
//                     readCmd.thrust = c;
//                 } else if(count == 3){
//                     //pitch
//                     readCmd.type2[0] = c - 127;
//                 } else if(count == 4){
//                     //roll
//                     readCmd.type2[1] = c - 127;
//                 } else if(count == 5){
//                     //yaw
//                     readCmd.type2[2] = c - 127;
//                 } else if(count > 5){
//                     //ignore these bytes
//                 }
//             } else {
//                 //not valid command, try again
//                 count = 0;
//            }
        } else {
            //8 bytes sent
            //verify checksum
            cstotal = readCmd.CMD_TYPE + readCmd.data_bytes[0] + readCmd.data_bytes[1] + readCmd.data_bytes[2] 
                        + readCmd.data_bytes[3] + readCmd.data_bytes[4] + readCmd.data_bytes[5];
            cksum = cstotal & 0xFF;
            //print("CKSUM=%u",cksum);            
            if(cksum == c){
              //fdserial_txChar(dport_ptr, 0x2E);
              //print("CK!!\n");
              return readCmd;
            } else {
                  //bad checksum
                  count = 0;
            }
        }
        
        if(count == 0) fdserial_rxFlush(xbee);
    } /* while(true) */
}

Quadcmd_t XbeeSerial::blankCmd(){
  Quadcmd_t blank;  
  blank.CMD_TYPE = 0;
  blank.data_bytes[0] = 0;
  blank.data_bytes[1] = 0;
  blank.data_bytes[2] = 0;
  blank.data_bytes[3] = 0;
  blank.data_bytes[4] = 0;
  blank.data_bytes[5] = 0;
  
  return blank;
}