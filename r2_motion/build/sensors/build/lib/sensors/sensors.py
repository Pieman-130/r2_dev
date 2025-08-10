import serial
import struct
from rclpy.node import Node
import rclpy
from std_msgs.msg import Float32

PAYLOAD_SIZE = 28 #7 4byte ints
STRUCT_READ = '<7i' #7 ints little endian
PREAMBLE = bytes([0xAA,0x55])
POSTAMBLE = bytes([0x55,0xAA])
CHECKSUM_SIZE = 1
PACKET_SIZE = len(PREAMBLE) + PAYLOAD_SIZE + CHECKSUM_SIZE + len(POSTAMBLE) #33 bytes
PUB_RATE = 0.2 #5Hz

'''
Decipher the serial data:
    0 - Left Distance Sensor
    1 - Right Distance Sensor
    2 - Rear Distance Sensor
    3 - Front Cliff Sensor
    4 - Rear Cliff Sensor
    5 - Right Hall Sensor
    6 - Left Hall Sensor

To convert values:
    Distance:
        value*0.0343/2/100 = dist in meters
    Clif:
        x = value*0.0048828125
        13*pow(x,-1)/100
    Hall:
        RPS = 1/((value/1000000)*90)
        M/S = dia * pi * RPS
'''

class SensorRead(Node):
    '''For reading sensors connected via arduino'''
    def __init__(self,port='/dev/ttyUSB0',data_rate=115200,pub_rate=PUB_RATE):
        super().__init__('Arduino_Sensors')
        try:
            self.ser = serial.Serial(port, data_rate)
        except:
            self.get_logger().error("Failed to connect to Arduino!")
            rclpy.shutdown()

        self.current_sensor_data = {'lt_dist_sensor': None,
                                    'rt_dist_sensor': None,
                                    'rr_dist_sensor': None,
                                    'ft_clif_sensor': None,
                                    'rr_clif_sensor': None,
                                    'lt_hall_sensor': None,
                                    'rt_hall_sensor': None}
        
        self.ltdist_publisher = self.create_publisher(Float32, 'raw_arduino/leftdistance', 10)
        self.rtdist_publisher = self.create_publisher(Float32, 'raw_arduino/rightdistance', 10)
        self.rrdist_publisher = self.create_publisher(Float32, 'raw_arduino/reardistance', 10)
        self.ftclif_publisher = self.create_publisher(Float32, 'raw_arduino/frontcliff', 10)
        self.rrclif_publisher = self.create_publisher(Float32, 'raw_arduino/rearcliff', 10)
        self.ltsped_publisher = self.create_publisher(Float32, 'raw_arduino/leftwheelspeed', 10)
        self.rtsped_publisher = self.create_publisher(Float32, 'raw_arduino/rightwheelspeed', 10)

        self.read_sensors_timer = self.create_timer(pub_rate/2, self.read_sensors) #reads twice as fast 
        self.ltdist_timer = self.create_timer(pub_rate, self.lt_dst_calbk)
        self.rtdist_timer = self.create_timer(pub_rate, self.rt_dst_calbk) 
        self.rrdist_timer = self.create_timer(pub_rate, self.rr_dst_calbk)
        self.ftclif_timer = self.create_timer(pub_rate, self.ft_clf_calbk)
        self.rtclif_timer = self.create_timer(pub_rate, self.rr_clf_calbk)
        self.ltsped_timer = self.create_timer(pub_rate, self.lt_rpm_calbk)
        self.rtsped_timer = self.create_timer(pub_rate, self.rt_rpm_calbk)


    def _validate_chunk(self,chunk):
        skip = 2+1+PAYLOAD_SIZE
        if chunk[0:2] == PREAMBLE and chunk[skip:skip+2] == POSTAMBLE:
            payload = chunk[2:PAYLOAD_SIZE+2]
            checksum = chunk[2+PAYLOAD_SIZE]
            calc_cksum = 0
            for b in payload:
                calc_cksum ^= b

            if calc_cksum == checksum:
                return payload
            else:
                self.get_logger().debug("Serial packet data bad checksum.")
                return False
        else:
            return False


    def read_sensors(self):
        if self.ser.in_waiting >= PACKET_SIZE:
            data = self.ser.read(PACKET_SIZE)

            payload = self._validate_chunk(data)
            if payload:
                try:
                    rcvd_data = struct.unpack(STRUCT_READ, payload)
                    self.current_sensor_data = {
                                'lt_dist_sensor': rcvd_data[0],
                                'rt_dist_sensor': rcvd_data[1],
                                'rr_dist_sensor': rcvd_data[2],
                                'ft_clif_sensor': rcvd_data[3],
                                'rr_clif_sensor': rcvd_data[4],
                                'rt_hall_sensor': rcvd_data[5],
                                'lt_hall_sensor': rcvd_data[6]
                                }
                except:
                    self.get_logger().warn("Failed to decode data from arduino sensors.")


    def lt_dst_calbk(self):
        msg = Float32()
        data = self.current_sensor_data['lt_dist_sensor']

        if data: #none signifies nothing received yet
            distance = data*0.0343/2/100 #distance in meters
            msg.data = distance
            self.ltdist_publisher.publish(msg)
            self.get_logger().debug(f"Published Left Distance: {distance} meters")
    
    
    def rt_dst_calbk(self):
        msg = Float32()
        data = self.current_sensor_data['rt_dist_sensor']

        if data: #none signifies nothing received yet
            distance = data*0.0343/2/100 #distance in meters
            msg.data = distance
            self.rtdist_publisher.publish(msg)
            self.get_logger().debug(f"Published Right Distance: {distance} meters")


    def rr_dst_calbk(self):
        msg = Float32()
        data = self.current_sensor_data['rr_dist_sensor']

        if data: #none signifies nothing received yet
            distance = data*0.0343/2/100 #distance in meters
            msg.data = distance
            self.rrdist_publisher.publish(msg)
            self.get_logger().debug(f"Published Rear Distance: {distance} meters")


    def ft_clf_calbk(self):
        msg = Float32()
        data = self.current_sensor_data['ft_clif_sensor']

        if data:
            x = data*0.0048828125
            distance=13*pow(x,-1)/100
            msg.data = distance
            self.ftclif_publisher.publish(msg)
            self.get_logger().debug(f"Published Front Cliff Distance: {distance} meters")

    def rr_clf_calbk(self):
        msg = Float32()
        data = self.current_sensor_data['rr_clif_sensor']

        if data:
            x = data*0.0048828125
            distance=13*pow(x,-1)/100
            msg.data = distance
            self.rrclif_publisher.publish(msg)
            self.get_logger().debug(f"Published Rear Cliff Distance: {distance} meters")
    

    def lt_rpm_calbk(self):
        msg = Float32()
        data = self.current_sensor_data['lt_hall_sensor']

        if data or data == 0.0:
            if data > 0.0:
                rpm = 1/((data/1000000)*90)*60
            else:
                rpm = 0.0
            msg.data = rpm
            self.ltsped_publisher.publish(msg)
            self.get_logger().debug(f"Published Left Wheel Speed: {rpm} RPM") 


    def rt_rpm_calbk(self):
        msg = Float32()
        data = self.current_sensor_data['rt_hall_sensor']

        if data or data == 0.0:
            if data > 0.0:
                rpm = 1/((data/1000000)*90)*60
            else:
                rpm = 0.0
            msg.data = rpm
            self.rtsped_publisher.publish(msg)
            self.get_logger().debug(f"Published Right Wheel Speed: {rpm} RPM")


def main():

    rclpy.init()
    sensors = SensorRead()
    rclpy.spin(sensors)
    sensors.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


