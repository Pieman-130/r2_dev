from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import rclpy
import RPi.GPIO as GPIO
import busio
from board import SCL,SDA
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

#pin assignments
LEFT_DIR = 17 #GPIO
RIGHT_DIR = 27 #GPIO
LEFT_MOTOR = 14 #PWM Channel on PCA9685
RIGHT_MOTOR = 15 #PWM Channel on PCA9685


class MotorDrive(Node):
    '''Node for moving the hoverboard motors'''
    def __init__(self):
        super().__init__('Motors')
        '''
        self.send_navplan_publisher = self.create_publisher(
            GeoPath,
            'navigator/navplan',
            10)
        '''
        self.motor_subscriber = self.create_subscription(
            Twist,
            'r2_move/motor',
            self.twist_callback,
            10)

        #if not set already change the GPIO mode to BCM
        if GPIO.getmode() != 11:
            GPIO.setmode(GPIO.BCM)

        #initialize direction pins
        GPIO.setup(LEFT_DIR, GPIO.OUT)
        GPIO.setup(RIGHT_DIR, GPIO.OUT)

        #initialize the PCA9685
        i2c = busio.I2C(SCL,SDA) #use std I2C GPIO PINS
        pca = PCA9685(i2c)
        pca.frequency = 50
        self.leftmotor = servo.Servo(pca.channels[LEFT_MOTOR], min_pulse = 0, max_pulse = 3000)
        self.rightmotor = servo.Servo(pca.channels[RIGHT_MOTOR], min_pulse = 0, max_pulse = 3000)
        self.set_direction('left','fwd')
        self.set_direction('right','fwd')

        self.get_logger().info("Motor control starting up.")


    def set_direction(self,motor,direction):
        if motor == 'left':
            m = LEFT_DIR
            if direction == 'fwd':
                d = GPIO.LOW
            elif direction == 'rev':
                d = GPIO.HIGH
        if motor == 'right':
            m = RIGHT_DIR
            if direction == 'fwd':
                d = GPIO.HIGH
            elif direction == 'rev':
                d = GPIO.LOW

        GPIO.output(m,d) #set motor and direction

    def set_motor_speed(self,motor,speed):
        '''valid values for servo library are 0 - 180
        values will always be positive.  
        set_direction sets whether fwd or rev'''
        #30 should be max speed for angular velocity of wheels
        #180 is max setting for servo library
    
        scaled_speed = abs(speed) * 6 #only need magnitude
        if scaled_speed > 180:
            scaled_speed = 180

        if motor == 'left':
            self.leftmotor.angle = scaled_speed
        elif motor == 'right':
            self.rightmotor.angle = scaled_speed
        else:
            self.get_logger().warn("Motor assignment not valid")
            return


    def twist_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        #kinematics:
        #--------------------------------------
        #wheel diameter = 150mm
        #distance between wheel centers = 225mm
        #--------------------------------------
        #motors are likely 150w hover-1 my first hoverboard
        #top speed of 5mph = 8046m/hr
        #max rpm = 285rpm based on 5mph max speed
        #max speed = 2.2m/s
        #-------------------------------------- 
        #reference: https://en.wikipedia.org/wiki/Differential_wheeled_robot
        #kinematic model w_r = (V + w * b/2)/r
        #r - radius of wheel
        #b - distance between wheels
        #V = forward linear velocity
        #w = angular velocity
        #-----------------------------
        
        r = 0.075 #in meters
        b = 0.225 #in meters

        wl = (linear - angular*b/2)/r
        wr = (linear + angular*b/2)/r

        #set left wheel direction
        if wl >= 0:
            self.set_direction('left','fwd')
        else:
            self.set_direction('left','rev')

        #set right wheel direction
        if wr >= 0:
            self.set_direction('right','fwd')
        else:
            self.set_direction('right', 'rev')

        #set motor velocities
        self.set_motor_speed('left',wl)
        self.set_motor_speed('right',wr)


def main():

    rclpy.init()
    motor = MotorDrive()
    rclpy.spin(motor)
    rclpy.shutdown()


if __name__=='__main__':
    main()




