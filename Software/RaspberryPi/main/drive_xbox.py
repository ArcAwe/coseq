#! usr/bin/python

import time
import serial
from threading import *
from array import *
import xbox_read
import sys
import datetime

thrust = float(0.0)
pitch = float(0.0)
roll = float(0.0)
yaw = float(0.0)

CMD_TYPE = 0

outf = open("/home/pi/quadcopter/main/out.txt","a")

# serialEvent = Event()
# serialCondition = Condition()
# dataLock = Lock()
# serialSem = BoundedSemaphore()
# dataSem = BoundedSemaphore()

handshake = 1


#commandString = array('B');
commandString = [0,0,0]


def new_value(current, pos): # Function for changing range
     if pos == "left-up":
                     return (((current - 0) * (180 - 90)) / (100 - 0)) + 90 # Change range from 0 - 100 to 90 - 180
     elif pos == "right-down":
            return (((current - 0) * (0 - 90)) / (100 - 0)) + 90 # Change range from 0 - 100 to 90 - 0


def right_joystick():
    global thrust, pitch, roll, yaw, handshake, commandString
    for event in xbox_read.event_stream(deadzone=2000, scale=255):
        if event.key == 'RT' or event.key == 'LT':
            thrust = event.value
        elif event.key == 'X1': #left stick
            yaw = event.value
        elif event.key == 'X2':
            roll = event.value
        elif event.key == 'Y2':
            pitch = event.value
        
        if(handshake == 1):
            commandString = formatCommand(CMD_TYPE) #if we can we combine them to send off
        

# Format the byte string
def formatCommand(format):
    global thrust, pitch, roll, yaw, handshake, commandString
    command = []

    #see quadcmd.h for API / standards

    if format == 1: #full manual
        #positive pitch = pitch down

        #positive roll = roll right
        
        motor1 = float()
        motor2 = float()
        motor3 = float()
        motor4 = float()
        
        #  /4 to soften control
        motor1 = (-pitch/4.0)+255.0
        motor3 = (pitch/4.0)+255.0
        motor2 = (-roll/4.0)+255.0
        motor4 = (roll/4.0)+255.0
        
        motor1 *= (thrust/255.0)
        motor2 *= (thrust/255.0)
        motor3 *= (thrust/255.0)
        motor4 *= (thrust/255.0)
        
        yaw /= 2
        
        if(yaw <= 0):
            yaw = max(yaw, -254)
            #turn left - bring down 2 and 4
            yawFactor = (255.0+yaw)/255.0
            motor2 *= yawFactor
            motor4 *= yawFactor
            motor1 /= yawFactor
            motor3 /= yawFactor
        else: #if(yaw > 0):
            yaw = min(yaw, 254)
            #turn right - bring down 1 and 3
            yawFactor = (255.0-yaw)/255.0
            motor1 *= yawFactor
            motor3 *= yawFactor
            motor2 /= yawFactor
            motor4 /= yawFactor
        # do nothing if abs(yaw) = 255 (DIV BY 0 ERROR)
        
        
        motor1final = int(min(motor1, 255))
        motor2final = int(min(motor2, 255))
        motor3final = int(min(motor3, 255))
        motor4final = int(min(motor4, 255))
        
#         motor1final = min(motor1, 255)
#         motor2final = min(motor1, 255)
#         motor3final = min(motor1, 255)
#         motor4final = min(motor1, 255)

        checksum = (1+motor1final+motor2final+motor3final+motor4final) & 0xff
        
        command.append(1)
        command.append(motor1final)
        command.append(motor2final) #only between 0-254 so we good
        command.append(motor3final)
        command.append(motor4final)
        command.append(0) #unused
        command.append(0) #unused
        command.append(checksum) #checksum?
        
#         print command
    elif (format == 2):
        thrustFinal = int(thrust)
        
        pitchVector = float()
        rollVector = float()
        yawVector = float()
        
        #pitch *= -1
        #roll *= -1
        
        pitchVector = max((-pitch/3.0)/2.0 + 127, 0)
        rollVector = max((-roll/3.0)/2.0 + 127, 0)
        yawVector = (yaw/8.0 + 255.0)/2.0
        
        pitchFinal = int(pitchVector)
        rollFinal = int(rollVector)
        yawFinal = int(yawVector)
        
        checksum = int()
        checksum = int(2+thrust+pitchFinal+rollFinal+yawFinal) & 0xff
        
        command.append(2)
        command.append(thrustFinal)
        command.append(pitchFinal)
        command.append(rollFinal)
        command.append(yawFinal)
        command.append(0)
        command.append(0)
        command.append(checksum)
        
    return command

def send_command():
    global thrust, pitch, roll, yaw
    try:
        serialPort = serial.Serial("/dev/ttyAMA0", 115200, timeout=0)
        serialPort.open()
        
        print "Sending commands.."
        
        while(True):
            handshake = 0 #stop the commandString from being updated
            cmdCopy = array('B',commandString)
            
            if(commandString[0] > 0):
                d = serialPort.write(cmdCopy)
#                 time.sleep(0.5)
#                 r = 'A'
#                 while(r != ''):
#                     r = serialPort.read(1)
#                     sys.stdout.write(r);
#                 print "\\"
                #serialPort.flushInput()
                #print str(pitch) + "\t" + str(roll) + "\t" + str(yaw)
                print cmdCopy
                #print_out(cmdCopy)

            handshake = 1 #let it keep getting updated
            time.sleep(0.02) #0.2 and up should work
    finally:
        serialPort.close()


#        for event in xbox_read.event_stream(deadzone=2000, scale=100): # Loop that looks for changes from Xbox controler
#               if event.key=='X2': # If change 'right joystic' moved left or right
#                    x2_intensity = int(event.value) # Set varible for position by intensity
#                    if x2_intensity >= 0: # If position of 'right joystick' right 'more than or 0'
#                                                        move_servo(new_value(x2_intensity, "right-down"), 1) # Change range and move servo to new position
#                                        elif x2_intensity <= 0: # If position of 'right joystic' left 'less than or 0'
#                                                        move_servo(new_value(x2_intensity*-1, "left-up"), 1) # Change range and move servo to new position
#                    else: # If position of 'right joystic' left or right 0
#                           move_servo(0, 1) # Move servo to new position
#                    ##bus.write_byte_data(address,0x13,0x00) # Turn MCP23017 port 1 off for activity light
#               elif event.key=='Y2': # If change 'right joystic' moved up or down
#                                        #bus.write_byte_data(address,0x13,0x1) # Turn MCP23017 port 1 on for activity light
#                                        y2_intensity = int(event.value) # Set varible for position by intensity
#                                        if y2_intensity >= 0: # If position of 'right joystic' down 'more than or 0'
#                                                       move_servo(new_value(y2_intensity, "right-down"), 2) # Change range and move servo to new position
#                                        elif y2_intensity <= 0: # If position of 'right joystic' up 'less than or 0'
#                                                        move_servo(new_value(y2_intensity*-1, "left-up"), 2) # Change range and move servo to new position
#                                        else: # If position of 'right joystic' up or down 0
#                                                        move_servo(0, 2) # Move servo to new position
#                                        #bus.write_byte_data(address,0x13,0x00) # Turn MCP23017 port 1 off for activity light

def readConfig():
    global outf, CMD_TYPE

    f = open("/home/pi/quadcopter/main/config.txt","r")
    print_out("Reading config.txt")
    for line in f:
        if(line[0] == '#'):
            #comment
            pass
        else:
            args = line.split('=',1)
            if(args[0].upper() == "CMD_TYPE"):
                cmdVal = int(args[1].upper())
                if(cmdVal > 0):
                    CMD_TYPE = cmdVal
                    print_out("Set CMD_TYPE to " + str(cmdVal));
                else:
                    print_out("ERROR - CMD_TYPE value not recognized")
                    print_out("CRITICAL ERROR - SHUTTING DOWN!")
                    sys.exit("Critical error in config.txt")
                    
    print_out("Configurations read.")

def print_out(str):
    global outf
    s = datetime.datetime.now().strftime("%Y-%m-%d %H%M")
    s = s + " " + str + "\n"
    outf.write(s)

def main():
    global outf
    try: 
        outf = open("out.txt","a")
        print_out("Starting.")
        readConfig()
    

        serialThread = Thread(target=send_command)
        serialThread.daemon = True
        serialThread.start() # Set new thread for serial
    
        stickThread = Thread(target=right_joystick)
        stickThread.daemon = True
        stickThread.start() # Set new thread for controls
    
        while True:
            time.sleep(1)
            #keep main thread alive to let ^c kill it
    except KeyboardInterrupt:
        print_out("KeyboardInterrupt - ending\n")
    except IOError:
        raise
    except:
        print_out("Error in a thread\n")
    finally:
        outf.close()
    

main()

#Thread(target=move_servo).start() # Set new thread for moving servos
#Thread(target=right_joystick).start() # Set new thread for controls
