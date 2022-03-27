#from curses import KEY_PPAGE
#from xxlimited import Null
import iqmotion as iq
import time
import math
import keyboard

import numpy as np
import matplotlib.pyplot as plt
import turtle

# Setup com ports (Subject to change based on where USB is plugged in)
#com1 = iq.SerialCommunicator("/dev/tty.usbserial-AB63836A") #bottom motor # "COM12" #port 2 on macbook adapter
#com2 = iq.SerialCommunicator("/dev/tty.usbserial-AB5X22Z3") #top motor #"COM13" #port 3 on macbook adapter
com1 = iq.SerialCommunicator("COM12") #bottom motor # "COM12" #port 2 on macbook adapter
com2 = iq.SerialCommunicator("COM13") #top motor #"COM13" #port 3 on macbook adapter 

# com1 = iq.SerialCommunicator("/dev/cu.usbserial-14510")
# com2 = iq.SerialCommunicator("/dev/cu.usbserial-14520")
# com3 = iq.SerialCommunicator("/dev/cu.usbserial-14530")
# com4 = iq.SerialCommunicator("/dev/cu.usbserial-14540")

# Initialize motors as IQ objects
vertiq1 = iq.Vertiq8108(com1, 0, firmware="cats")
vertiq2 = iq.Vertiq8108(com2, 0, firmware="cats")
# vertiq3 = iq.Vertiq2306(com3, 0, firmware="servo")
# vertiq4 = iq.Vertiq2306(com4, 0, firmware="servo")
vertiqs = [vertiq1, vertiq2]

# Boolean Setups
pause = True
PID = True

#Motor direction
motorDir1 = 1
motorDir2 = -1

# Target Speed (rad/s)
targetSpeed = 5
targetSpeed1 = targetSpeed * motorDir1
targetSpeed2 = targetSpeed * motorDir2
#add input rpm to rad/s 

# How often to update motors
time_step = .0001

# Angle offset
testOffset = 90 #angle offset for test *****DEGREES*****
testOffsetRAD = testOffset*3.14/180
motorOff1 = 0 #motor1 bottom
motorOff2 = .43+(testOffset*3.14/180) #motor2 top
#motorOff3 = 0
#motorOff4 = 0


# P I D
P = 1
I = .5
D = .1
# 1,1,.1

class PID(object):
    def __init__(self, KP, KI, KD, target, motorOff, motorDir):
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.target = target
        self.error = 0
        self.error_integral = 0
        self.error_last = 0
        self.error_derivative = 0
        self.output = 0
        self.prevTime = 0
        self.currentTime = 0
        self.motorOffset = motorOff
        self.motorDir = motorDir
        self.startTime = 0
   
    def compute(self, position):
        self.currentTime = time.time()
        self.error = ((self.currentTime-self.startTime) * self.target) - position + self.motorOffset #need to fix to allow for neg values
        self.error_integral += self.error * (self.currentTime - self.prevTime)
        self.error_derivative = self.motorDir*(self.error - self.error_last) / (self.currentTime - self.prevTime)
        self.error_last = self.error
        self.output = self.kp*self.error + self.ki*self.error_integral + self.kd*self.error_derivative

        self.prevTime = self.currentTime
        #Limits
        if self.output > 50:
            self.output = 50
        if self.output < -50:
            self.output = -50
        return self.output

class Motor(object):
    def __init__(self, vertiq, KP, KI, KD, target, motorOff, motorDir):
        global Motor
        self.velocity = 0
        self.vertiq = vertiq
        self.motorOffset = motorOff
        self.PID = PID(KP, KI, KD, target, motorOff,motorDir)

    def set_velocity(self, velocity):
        self.velocity = velocity
   
    def get_velocity(self):
        return self.velocity
    
    def get_offset(self):
        return self.motorOffset

def graph(x,y):
    plt.plot(x, y)
    plt.show()

motor1 = Motor(vertiq1, P, I, D, targetSpeed, motorOff1, motorDir1)
motor2 = Motor(vertiq2, P, I, D, targetSpeed, motorOff2, motorDir2)
#motor3 = Motor(vertiq3, P, I, D, targetSpeed, motorOff3)
#motor4 = Motor(vertiq4, P, I, D, targetSpeed, motorOff4)
motors = [motor1, motor2]#, motor3, motor4]


# motor1.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed) 
# motor1.vertiq.set("multi_turn_angle_control", "trajectory_angular_velocity", targetSpeed)
# #motor1.vertiq.set("multi_turn_angle_control", "trajectory_duration", testDuration)
# motor1.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", 0) 


#set max speed
motor1.vertiq.set("multi_turn_angle_control", "angular_speed_max", targetSpeed)
motor2.vertiq.set("multi_turn_angle_control", "angular_speed_max", targetSpeed)

for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", 0)
    motor.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", 0)
    motor.vertiq.set("multi_turn_angle_control", "trajectory_duration", 1)
time.sleep(1.5)

# Set initial speed of motors
for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", motor.motorOffset)
    motor.vertiq.set("multi_turn_angle_control", "trajectory_duration", 1)

time.sleep(1.5)



# MATPLOTLIB
#poses = np.array([])
#times = np.array([])
   
i = 0
print("Now Beginning Ramp-Up\n")
while i < targetSpeed: #Ramp up... wait for current == target speed
    motor1.vertiq.set("multi_turn_angle_control", "ctrl_velocity", i)
    motor2.vertiq.set("multi_turn_angle_control", "ctrl_velocity", -i)
    i = i + 1
    time.sleep(.05)
    print(motor1.vertiq.get("multi_turn_angle_control", "ctrl_velocity"))

#Hold speed... wait for keyboard input to start PID (Before data aqu starts)
value = input("Press B to begin test\n")
if value == 'b' or value == 'B':
    pause = False
    
    
# Store start time of Program
startTime = time.time()


for motor in motors:
    #motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed)
    motor.PID.startTime = startTime
    motor.PID.prevTime = startTime
#PID
print("Now Beginning PID-Phase Control\n")
print("Press S to stop test\n")
# while PID: #PID...wait for keyboard input to stop phase control
#     for index, motor in enumerate(motors):
#         obsDisplacement = motor.vertiq.get("multi_turn_angle_control", "obs_angular_displacement")
#         if obsDisplacement is not None:
#             velocity = motor.PID.compute(obsDisplacement)
#             motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed + velocity)

# while PID: #PID...wait for keyboard input to stop phase control
#     for index, motor in enumerate(motors):
#         obsDisplacement = motor.vertiq.get("multi_turn_angle_control", "obs_angular_displacement")
#         if obsDisplacement is not None:
#             velocity = motor.PID.compute(obsDisplacement)
#             motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", motor.get_offset()*targetSpeed + velocity)

while PID:
    obsDisplacement1 = motor1.vertiq.get("multi_turn_angle_control", "obs_angular_displacement")
    if obsDisplacement1 is not None:
        velocity1 = motor1.PID.compute(obsDisplacement1)
        motor1.vertiq.set("multi_turn_angle_control", "ctrl_velocity", motorDir1*(targetSpeed + velocity1))
    obsDisplacement2 = motor2.vertiq.get("multi_turn_angle_control", "obs_angular_displacement")
    if obsDisplacement2 is not None:
        velocity2 = motor2.PID.compute(obsDisplacement2)
        motor2.vertiq.set("multi_turn_angle_control", "ctrl_velocity", motorDir2*(targetSpeed + velocity2))
            #print("Motor %s: Position Error: %s" % (index+1, motor.PID.error))

            # """
            # print(motor1.PID.error)
           
            # if (time.time()-startTime > 9):
            #     poses = np.append(poses, velocity)
            #     times = np.append(times, motor1.PID.currentTime)
            # """
    if keyboard.is_pressed('s'):  # if key 'z' is pressed 
                print("Test stopped\n")
                PID = False
                break  # finishing the loop


#coast ... wait for vel/pos/everything to = 0 
print("Now Beginning Coasting")
for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "ctrl_coast")
time.sleep(2) 
       
motor1.vertiq.set("multi_turn_angle_control", "ctrl_velocity", 0)
motor2.vertiq.set("multi_turn_angle_control", "ctrl_velocity", 0)
time.sleep(2)

# # Reset initial position to default from motors.  
# print("Now Beginning Reset")
# motor1.vertiq.set("multi_turn_angle_control", "trajectory_duration", 3)
# motor1.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", 0)
# motor1.vertiq.set("multi_turn_angle_control", "angular_speed_max", 3)
# motor2.vertiq.set("multi_turn_angle_control", "trajectory_duration", 3)
# motor2.vertiq.set("multi_turn_angle_control", "trajectory_angular_displacement", motorOff2)
# motor2.vertiq.set("multi_turn_angle_control", "angular_speed_max", 3)





# # graph(times, poses)
# #def resetVar():
# #    global traj_pos
# #    traj_pos 
    