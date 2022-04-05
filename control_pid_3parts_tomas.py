# from curses import KEY_PPAGE
# from xxlimited import Null
import iqmotion as iq
import time
import math
import keyboard

import numpy as np
import matplotlib.pyplot as plt
import turtle
import csv


class MotorPID:
    def __init__(self, kp, ki, kd, target_speed, motor_off):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target_speed = target_speed
        self.error = 0
        self.error_integral = 0
        self.prev_error = 0
        self.error_derivative = 0
        self.output = 0
        self.out_p = 0
        self.out_i = 0
        self.out_d = 0
        self.prev_time = 0
        self.current_time = 0
        self.motor_offset = motor_off
        # self.motor_dir = motor_dir
        self.start_time = 0

    def compute(self, position):
        self.current_time = time.time()
        self.error = ((self.current_time - self.start_time) *
                      self.target_speed) - position + self.motor_offset
        delta_error = self.error - self.prev_error
        delta_time = self.current_time - self.prev_time
        # need to fix to allow for neg values
        self.error_integral += self.error * delta_time
        self.error_derivative = delta_error / delta_time
        self.out_p = self.kp * self.error
        self.out_i = self.ki * self.error_integral
        self.out_d = self.kd * self.error_derivative
        self.output = self.out_p + self.out_i + self.out_d

        self.prev_error = self.error
        self.prev_time = self.current_time

        # Limits
        if self.output > 50:
            self.output = 50
        if self.output < -50:
            self.output = -50
        return self.output

    def print_state(self):
        print(f'error_proportional {self.error}')
        print(f'error_integral {self.error_integral}')
        print(f'error_derivative {self.error_derivative}')
        print(f'out_p {self.out_p}')
        print(f'out_i {self.out_i}')
        print(f'out_d {self.out_d}')
        print(f'output {self.output}')
        
    def return_state(self):
        return [self.error,self.error_integral,self.error_derivative,self.out_p,self.out_i,self.out_d,self.output]


class Motor:
    def __init__(self, vertiq, kp, ki, kd, target_speed, motor_off, motor_dir):
        # global Motor
        self.velocity = 0
        self.vertiq = vertiq
        self.motor_offset = motor_off
        self.motor_pid = MotorPID(kp, ki, kd, target_speed, motor_off)

    def set_velocity(self, velocity):
        self.velocity = velocity

    def get_velocity(self):
        return self.velocity

    def get_offset(self):
        return self.motor_offset


# Setup com ports (Subject to change based on where USB is plugged in)
# com1 = iq.SerialCommunicator("/dev/tty.usbserial-AB63836A") #bottom motor
# "COM12" #port 2 on macbook adapter
# com2 = iq.SerialCommunicator("/dev/tty.usbserial-AB5X22Z3") #top motor
# #"COM13" #port 3 on macbook adapter
com1 = iq.SerialCommunicator("COM12")
# bottom motor # "COM12" #port 2 on macbook adapter
com2 = iq.SerialCommunicator("COM13")
# top motor #"COM13" #port 3 on macbook adapter

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
keep_running = True

# Motor direction
motor_dir1 = 1
motor_dir2 = -1

# Target Speed (rad/s)
desired_speed = 5
# target_speed1 = target_speed * motor_dir1
# target_speed2 = target_speed * motor_dir2
# add input rpm to rad/s

# How often to update motors
# time_step = .0001

# Angle offset
test_offset = 90     # angle offset for test *****DEGREES*****
test_offset_rad = test_offset * np.pi / 180
motor_off1 = 0   # motor1 bottom
motor_off2 = 0.43 + (test_offset * np.pi / 180)   # motor2 top
# motorOff3 = 0
# motorOff4 = 0

# PID gains
p_gain = 1
i_gain = .5
d_gain = .1
# 1,1,.1
motor1 = Motor(
    vertiq1, p_gain, i_gain, d_gain, desired_speed, motor_off1, motor_dir1)
motor2 = Motor(
    vertiq2, p_gain, i_gain, d_gain, desired_speed, motor_off2, motor_dir2)
# motor3 = Motor(vertiq3, P, I, D, targetSpeed, motorOff3)
# motor4 = Motor(vertiq4, P, I, D, targetSpeed, motorOff4)
motors = [motor1, motor2]   # , motor3, motor4]

# motor1.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed) 
# motor1.vertiq.set("multi_turn_angle_control",
# "trajectory_angular_velocity", targetSpeed)
# #motor1.vertiq.set("multi_turn_angle_control",
# "trajectory_duration", testDuration)
# motor1.vertiq.set("multi_turn_angle_control",
# "trajectory_angular_displacement", 0)

dataFile = open('test.csv', 'w')
fileWriter = csv.writer(dataFile)


# set max speed
for motor in motors:
    motor.vertiq.set(
        "multi_turn_angle_control", "angular_speed_max", desired_speed * 1.2)

for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", 0)
    motor.vertiq.set("multi_turn_angle_control",
                     "trajectory_angular_displacement", 0)
    motor.vertiq.set("multi_turn_angle_control", "trajectory_duration", 1)
time.sleep(1.5)

# Set initial speed of motors
for motor in motors:
    motor.vertiq.set("multi_turn_angle_control",
                     "trajectory_angular_displacement", motor.motor_offset)
    motor.vertiq.set("multi_turn_angle_control", "trajectory_duration", 1)
time.sleep(1.5)

# MATPLOTLIB
# poses = np.array([])
# times = np.array([])
   
# i = 0
# print("Now Beginning Ramp-Up\n")
# while i < desired_speed:      # Ramp up... wait for current == target speed
#     motor1.vertiq.set("multi_turn_angle_control", "ctrl_velocity", i)
#     motor2.vertiq.set("multi_turn_angle_control", "ctrl_velocity", -i)
#     i = i + 1
#     time.sleep(.05)
#     print(motor1.vertiq.get("multi_turn_angle_control", "ctrl_velocity"))

# # Hold speed... wait for keyboard input to start PID (before data aqu starts)
# value = input("Press B to begin test\n")
# if value == 'b' or value == 'B':
#     pause = False
# Store start time of Program
start_time = time.time()
for motor in motors:
    # motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity", targetSpeed)
    motor.motor_pid.start_time = start_time
    motor.motor_pid.prev_time = start_time
time.sleep(.05)
# PID
print("Now Beginning Phase Control\n")
print("Press S to stop test\n")
# while MotorPID: #MotorPID...wait for keyboard input to stop phase control
#     for index, motor in enumerate(motors):
#         obsDisplacement = motor.vertiq.get("multi_turn_angle_control",
#         "obs_angular_displacement")
#         if obsDisplacement is not None:
#             velocity = motor.MotorPID.compute(obsDisplacement)
#             motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity",
#             targetSpeed + velocity)

# while MotorPID: #MotorPID...wait for keyboard input to stop phase control
#     for index, motor in enumerate(motors):
#         obsDisplacement = motor.vertiq.get("multi_turn_angle_control",
#         "obs_angular_displacement")
#         if obsDisplacement is not None:
#             velocity = motor.MotorPID.compute(obsDisplacement)
#             motor.vertiq.set("multi_turn_angle_control", "ctrl_velocity",
#             motor.get_offset()*targetSpeed + velocity)

cmd_speed1 = 0
cmd_speed2 = 0
while keep_running:
    obs_displacement1 = motor1.vertiq.get(
        "multi_turn_angle_control", "obs_angular_displacement")
    if obs_displacement1 is not None:
        velocity1 = motor1.motor_pid.compute(obs_displacement1)
        cmd_speed1 = motor_dir1 * desired_speed + velocity1
        motor1.vertiq.set("multi_turn_angle_control", "ctrl_velocity",
                          cmd_speed1)

    obs_displacement2 = motor2.vertiq.get(
        "multi_turn_angle_control", "obs_angular_displacement")
    if obs_displacement2 is not None:
        velocity2 = motor2.motor_pid.compute(obs_displacement2)
        cmd_speed2 = (motor_dir2 * desired_speed + velocity2)
        motor2.vertiq.set("multi_turn_angle_control", "ctrl_velocity",
                          cmd_speed2)

    obs_delta_disp = obs_displacement1 - obs_displacement2
    print(f'obs_displacement1 {obs_displacement1}')
    motor1.motor_pid.print_state()
    print(f'obs_displacement2 {obs_displacement2}')
    motor2.motor_pid.print_state()
    print(f'obs_delta_disp {obs_delta_disp * 180 / np.pi}')
    print(f'cmd_speed1 {cmd_speed1}')
    print(f'cmd_speed2 {cmd_speed2}')
    print('--------------------------------------')
    
    
    csvData = np.concatenate(([obs_displacement1],motor1.motor_pid.return_state(),[obs_displacement2],motor2.motor_pid.return_state(),[obs_delta_disp * 180 / np.pi],[cmd_speed1],[cmd_speed2]))
    #obs_displacement1,error_proportional,error_integral,error_derivative,out_p,out_i,out_d,output,obs_displacement2,error_proportional,error_integral,error_derivative,out_p,out_i,out_d,output,obs_delta_disp,cmd_speed1,cmd_speed2
    fileWriter.writerow(csvData)



    if keyboard.is_pressed('s'):  # if key 'z' is pressed 
        print("Test stopped\n")
        keep_running = False
        break  # finishing the loop


# coast ... wait for vel/pos/everything to = 0
print("Now Beginning Coasting")
dataFile.close()
for motor in motors:
    motor.vertiq.set("multi_turn_angle_control", "ctrl_coast")
time.sleep(2) 
       
motor1.vertiq.set("multi_turn_angle_control", "ctrl_velocity", 0)
motor2.vertiq.set("multi_turn_angle_control", "ctrl_velocity", 0)
time.sleep(2)

# # Reset initial position to default from motors.  
# print("Now Beginning Reset")
# motor1.vertiq.set("multi_turn_angle_control", "trajectory_duration", 3)
# motor1.vertiq.set("multi_turn_angle_control",
# "trajectory_angular_displacement", 0)
# motor1.vertiq.set("multi_turn_angle_control", "angular_speed_max", 3)
# motor2.vertiq.set("multi_turn_angle_control", "trajectory_duration", 3)
# motor2.vertiq.set("multi_turn_angle_control",
# "trajectory_angular_displacement", motorOff2)
# motor2.vertiq.set("multi_turn_angle_control", "angular_speed_max", 3)

# # graph(times, poses)
# #def resetVar():
# #    global traj_pos
# #    traj_pos 
    