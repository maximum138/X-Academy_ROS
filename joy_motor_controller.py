#!/usr/bin/env
import rclpy
import time
import pi_servo_hat
import atexit
from rclpy.node import Node
from sensor_msgs.msg import Joy
import numpy as np
import math

def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class PiHat:
    def __init__(self):
        self.hat = pi_servo_hat.PiServoHat()
        self.hat.restart()
        
        atexit.register(self.stop_all_motors) # this only works when run in terminal

    def send_pwm(self, channel, signal):
        # signal (in microseconds for PWM)
        
        angle = (signal - 922) / 10.8 #trim here
        
        self.hat.move_servo_position(channel, angle)


    def drive_motor(self, channel, velocity):
        # velocity is a float between -1 and 1.
        # -1 corresponds to full backwards,
        # 1 corresponds to full forwards,
        # 0 corresponds to stopped
        
        velocity = max(min(velocity, 1), -1)
        velocity = map_value(velocity, -1, 1, 1400, 1600)
        print(f'Velocity: {velocity}')
        self.send_pwm(channel, velocity)
    
    def stop_all_motors(self):
        for channel in range(16):
            self.drive_motor(channel, 0)

class JoyControllerNode(Node):

    def __init__(self):
        super().__init__("joy_controller")
        self.joy_subscriber_ = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10 #queue size
            )
        self.get_logger().info("Controller has been started.")
        self.hat = PiHat()

    def joy_callback(self, msg: Joy):
        self.get_logger().info(str(msg))

        #motor mapping
        raw_input = np.matrix([[msg.axes[0]],
                               [msg.axes[1]]])
        max_component = max(abs(raw_input[0,0]),abs(raw_input[1,0]))
        try:
            converted_input = (max_component/(math.sqrt((raw_input[0,0]**2) + (raw_input[1,0]**2)))) * raw_input
        except:
            converted_input = raw_input

        converted_input = (max_component/(math.sqrt((raw_input[0,0]**2) + (raw_input[1,0]**2)))) * raw_input

        if math.isnan(converted_input[0,0]):
            converted_input[0,0] = 0
        if math.isnan(converted_input[1,0]):
            converted_input[1,0] = 0

        sqrt_two = math.sqrt(2)
        rov_basis_change = np.matrix([[sqrt_two,sqrt_two],
                                      [-sqrt_two,sqrt_two]])
        motion_vector = rov_basis_change * converted_input
        motion_vector_lengthened = np.matrix([[motion_vector[0,0]],
                                              [motion_vector[1,0]],
                                              [motion_vector[1,0]],
                                              [motion_vector[0,0]]])
        
        turning_vector = (-1)*np.matrix([[-msg.axes[2]],
                                    [msg.axes[2]],
                                    [-msg.axes[2]],
                                    [msg.axes[2]],])
        
        final_vector = (-1)*((max_component / (max_component + abs(msg.axes[2]))) * motion_vector_lengthened) + ((abs(msg.axes[2]) / (max_component + abs(msg.axes[2]))) * turning_vector)

        for i in range(final_vector.shape[0]):
            if math.isnan(final_vector[i,0]):
                final_vector[i,0] = 0

        self.hat.drive_motor(1,final_vector[0,0])
        self.hat.drive_motor(5,final_vector[1,0])
        self.hat.drive_motor(0,final_vector[2,0])
        self.hat.drive_motor(4,final_vector[3,0])
        
        #TEMP VERTICAL CONTROL
        if msg.buttons[4]==1:
            self.hat.drive_motor(2,msg.buttons[4]*(-1))
            self.hat.drive_motor(3,msg.buttons[4]*(-1))
        elif msg.buttons[2]==1:
            self.hat.drive_motor(2,msg.buttons[2])
            self.hat.drive_motor(3,msg.buttons[2])
        else:
            self.hat.drive_motor(2,0)
            self.hat.drive_motor(3,0)

        print(final_vector)
        
def main(args = None):
    rclpy.init(args = args)
    node  = JoyControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
