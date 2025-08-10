from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
import time
import rclpy
import busio
import math
from board import SCL,SDA
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

#pwm channel assignments
YAW = 13 #0-15 16-channel pwm
PITCH = 12

#servo specific configs
PWM_LOW = 500 #micro sec
PWM_HIGH = 2500 #micro sec
FREQ = 50

PUB_RATE = 0.2 #5Hz

class HeadMotion(Node):
    '''Node for moving R2's head'''
    def __init__(self,pub_rate=PUB_RATE):
        super().__init__('Head')
      
        self.head_pos_publisher = self.create_publisher(
            Quaternion,
            'head/position',
            10)
    
        self.head_move_subscriber = self.create_subscription(
            Quaternion,
            'head/move_head',
            self.move_head_callback,
            10)
        
        self.head_speed_subscriber = self.create_subscription(
            Float32,
            'head/move_head_speed',
            self.move_head_speed_callback,
            10)
        
        self.head_pos_timer = self.create_timer(pub_rate, self.pub_head_pos)

        i2c = busio.I2C(SCL,SDA)
        pca = PCA9685(i2c)
        pca.frequency = FREQ
        self.yaw = servo.Servo(pca.channels[YAW], min_pulse=PWM_LOW, max_pulse=PWM_HIGH)
        self.pitch = servo.Servo(pca.channels[PITCH], min_pulse=PWM_LOW, max_pulse=PWM_HIGH)

        self.move_speed = None
     
        self.get_logger().info("Head Motion control starting up.")
        self.get_logger().info("Moving Head to home position of Pitch 145deg and Yaw 95deg")
     
        self._set_servos(145,95,self.pitch.angle,self.yaw.angle,0.025) #home position

    def _convert_quat_to_euler(self,quat):
        x,y,z,w = quat.x,quat.y,quat.z,quat.w

        yaw_rad = math.atan2(2.0 * (w * z + x * y),
                             1.0 - 2.0 * (y * y + z * z))
        
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch_rad = math.copysign(math.pi / 2, sinp)
        else:
            pitch_rad = math.asin(sinp)

        pitch_deg = math.degrees(pitch_rad)
        yaw_deg = math.degrees(yaw_rad)

        self.get_logger().info(f'Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°')

        return pitch_deg, yaw_deg


    def move_head_speed_callback(self,msg):
        self.move_speed = -1*msg.data+1 #sets delay for how long to take to move
        #higher numbers are slower speeds


    def move_head_callback(self, msg):
        pd, yd = self._convert_quat_to_euler(msg)

        #read current state
        current_yaw = self.yaw.angle
        current_pitch = self.pitch.angle
     
        #get requested speed
        if self.move_speed:
            delay = self.move_speed
        else: #if not set
            delay = 0.025 #good marginal speed

        self._set_servos(pd,
                         yd,
                         current_pitch,
                         current_yaw,
                         delay)

    
    def _set_servos(self,pd,yd,cp,cy,delay):
        '''set the servo values so they move together
        in a sort-of-smooth motion'''
        #correct for small neg values
        if cp: #None is not set yet
            if cp < 0:
                cp = 0
        else:
            cp = 0
        if cy: #None is not set yet
            if cy < 0:
                cy = 0
        else:
            cy = 0

        #determine number of steps to move both servos
        dif_y = int(abs(cy - yd))
        dif_p = int(abs(cp - pd))
        if dif_y > dif_p: #whichever is largest
            steps = dif_y
        else:
            steps = dif_p
        try:
            step_size_y = dif_y/steps #step size dependent for each servo
        except ZeroDivisionError:
            step_size_y = 0
        try:
            step_size_p = dif_p/steps
        except ZeroDivisionError:
            step_size_p = 0

        for s in range(steps):
            if cp > pd:
                #next_pitch = step_size_p*s + cp
                next_pitch = cp - step_size_p*s
            else:     
                next_pitch = step_size_p*s + cp 
                #next_pitch = cp - step_size_p*s
            
            #set pitch limits
            if next_pitch < 100.0:
                next_pitch = 100.0
            elif next_pitch > 180.0:
                next_pitch = 180.0
            
            if cy > yd:
                #next_yaw = step_size_y*s + cy
                next_yaw = cy - step_size_y*s
            else:
                next_yaw = step_size_y*s + cy
                #next_yaw = cy - step_size_y*s

            #set yaw limits
            if next_yaw < 5.0:
                next_yaw = 5.0
            elif next_yaw > 180.0:
                next_yaw = 180.0
         
            self.pitch.angle = next_pitch
            self.yaw.angle = next_yaw
            time.sleep(delay) #slows motion to make it more fluid

 
    def pub_head_pos(self):

        msg = Quaternion()

        try:
            yaw_rad = self.yaw.angle*math.pi/180
            pitch_rad = self.pitch.angle*math.pi/180
        except TypeError: #Can be none if it was never set before
            yaw_rad = 0
            pitch_rad = 0

        self.get_logger().debug(f"Current Yaw: {self.yaw.angle}, Current Pitch: {self.pitch.angle}")

        #convert euler to quaternion
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(0 * 0.5) #no roll
        sr = math.sin(0 * 0.5) #no roll

        msg.w = cy * cp * cr + sy * sp * sr
        msg.x = cy * cp * sr - sy * sp * cr
        msg.y = sy * cp * sr + cy * sp * cr
        msg.z = sy * cp * cr - cy * sp * sr

        self.head_pos_publisher.publish(msg)
        self.get_logger().debug(f"Published Head Position: ({msg.w},{msg.x},{msg.y},{msg.z})")
    

def main():

    rclpy.init()
    head = HeadMotion()
    rclpy.spin(head)
    rclpy.shutdown()


if __name__=='__main__':
    main()






