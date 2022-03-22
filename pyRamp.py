import iqmotion as iq
import time
import math

import numpy as np
import matplotlib.pyplot as plt
import turtle

# Setup com ports (Subject to change based on where USB is plugged in)
com1 = iq.SerialCommunicator("/dev/tty.usbserial-AB63836A") #bottom motor # "COM12" #port 2 on macbook adapter
com2 = iq.SerialCommunicator("/dev/tty.usbserial-AB5X22Z3") #top motor #"COM13" #port 3 on macbook adapter

# Angle offset
motorOff1 = 0
motorOff2 = 0.43 #0.43-0.47 

#Motor direction
motorDir1 = 1
motorDir2 = -1

# Initialize motors as IQ objects
vertiq1 = iq.Vertiq8108(com1, 0, firmware="servo")
vertiq2 = iq.Vertiq8108(com2, 0, firmware="servo")
# vertiq3 = iq.Vertiq2306(com3, 0, firmware="servo")
# vertiq4 = iq.Vertiq2306(com4, 0, firmware="servo")
vertiqs = [vertiq1, vertiq2]

# Target Speed (rad/s)
targetSpeed = 15 #no decimals

# Test Duration
testDuration = 5 #seconds 

# How often to update motors
time_step = .0001

class Motor(object):
    def __init__(self, vertiq, target, motorOff, motorDir):
        global Motor
        self.velocity = 0
        self.vertiq = vertiq
        self.motorOffset = motorOff
        self.motorDir = motorDir

    def set_velocity(self, velocity):
        self.velocity = velocity #* self.motorDir
   
    def get_velocity(self):
        return self.velocity
    
    def get_offset(self):
        return self.motorOffset

def graph(x,y):
    plt.plot(x, y)
    plt.show()


motor1 = Motor(vertiq1, targetSpeed, motorOff1, motorDir1)
motor2 = Motor(vertiq2, targetSpeed, motorOff2, motorDir2)
#motor3 = Motor(vertiq3, P, I, D, targetSpeed, motorOff3)
#motor4 = Motor(vertiq4, P, I, D, targetSpeed, motorOff4)
motors = [motor1, motor2]#, motor3, motor4]

# Set initial speed, initial position to default from motors.
# Slow ramp 
motor1.vertiq.set("multi_turn_angle_control", "ctrl_velocity", 0)
motor1.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", 0)
motor1.vertiq.set("multi_turn_angle_control", "trajectory_duration", 5)
motor1.vertiq.set("multi_turn_angle_control", "angular_speed_max", targetSpeed+5) #check later
time.sleep(2)
motor2.vertiq.set("multi_turn_angle_control", "ctrl_velocity", 0)
motor2.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", 0)
motor2.vertiq.set("multi_turn_angle_control", "trajectory_duration", 5)
motor2.vertiq.set("multi_turn_angle_control", "angular_speed_max", targetSpeed+5) #check later
time.sleep(2) #put in a delay between the two motors. Begin the top motor  

#set top motor to always hold at the new value
motor2.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", motorOff2)
motor2.vertiq.set("multi_turn_angle_control", "trajectory_duration", 5)
time.sleep(5)

# Store start time of Program
startTime = time.time()

time.sleep(3)

#begin standard control
motor1.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed) 
motor1.vertiq.set("multi_turn_angle_control", "trajectory_angular_velocity", targetSpeed)
#motor1.vertiq.set("multi_turn_angle_control", "trajectory_duration", testDuration)
motor1.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", 0) 
motor2.vertiq.set("multi_turn_angle_control", "ctrl_velocity", -targetSpeed)
#motor2.vertiq.set("multi_turn_angle_control", "trajectory_angular_velocity", targetSpeed)
motor2.vertiq.set("multi_turn_angle_control", "trajectory_duration", testDuration)
motor2.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", motorOff2) 
time.sleep(testDuration)    
    

#bcoast after run
for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "ctrl_coast")
time.sleep(2) 
       
motor1.vertiq.set("multi_turn_angle_control", "ctrl_velocity", 0)
motor2.vertiq.set("multi_turn_angle_control", "ctrl_velocity", 0)
time.sleep(2)

# Reset initial position to default from motors.  
motor1.vertiq.set("multi_turn_angle_control", "trajectory_duration", 3)
motor1.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", 0)
motor1.vertiq.set("multi_turn_angle_control", "angular_speed_max", 3)
motor2.vertiq.set("multi_turn_angle_control", "trajectory_duration", 3)
motor2.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", motorOff2)
motor2.vertiq.set("multi_turn_angle_control", "angular_speed_max", 3)


    
