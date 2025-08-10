from sensor_msgs.msg import NavSatFix, NavSatStatus
from rclpy.node import Node
import rclpy
from serial import Serial
from pynmeagps import NMEAReader
import threading

PUB_RATE = 1.0  #1Hz

class GPSNode(Node):
    def __init__(self,serial_port='/dev/ttyUSB1',baud=4800,pub_rate=PUB_RATE):
        super().__init__('GPS_BU353S4')

        self.port = serial_port
        self.baud = baud
        self.gps_publisher = self.create_publisher(NavSatFix, 'gps', 10)
        self.gps_msg = {'lat': None,
                        'lon': None,
                        'alt': None,
                        'qul': None,
                        'nsv': None,
                        'hdp': None}

        self.gps_timer = self.create_timer(pub_rate, self.gps_tmr_calbk)

    def serial_listener(self):
        def serial_thread():

            with Serial(self.port, self.baud, timeout=3) as stream:
                while True:
                    nmr = NMEAReader(stream)
                    raw_data, parsed_data = nmr.read()
                    if parsed_data is not None:
                        if parsed_data.msgID == 'GGA': #this is the NMEA location message
                            self.gps_msg['lat'] = parsed_data.lat
                            self.gps_msg['lon'] = parsed_data.lon
                            self.gps_msg['alt'] = parsed_data.alt
                            self.gps_msg['qul'] = parsed_data.quality
                            self.gps_msg['nsv'] = parsed_data.numSV
                            self.gps_msg['hdp'] = parsed_data.HDOP
        s = threading.Thread(target=serial_thread, name='GPS Listener', daemon=True)
        s.start()


    def gps_tmr_calbk(self):
        
        msg = NavSatFix()
        if self.gps_msg['lat']: #else no data received yet
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'gps_frame'
            msg.status.service = NavSatStatus.SERVICE_GPS
            if self.gps_msg['qul'] > 0:
                msg.status.status = NavSatStatus.STATUS_FIX
            else:
                msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.latitude = self.gps_msg['lat']
            msg.longitude = self.gps_msg['lon']
            msg.altitude = self.gps_msg['alt']

            #From CHATGPT
            h_var = (self.gps_msg['hdp'] * 5.0) ** 2 #5m nominal accuracy scaled by HDOP
            v_var = (10.0) ** 2 #assumed 10m std dev for alt
            msg.position_covariance = [ h_var, 0.0, 0.0,
                                        0.0, h_var, 0.0,
                                        0.0, 0.0, v_var]
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            self.gps_publisher.publish(msg)
            self.get_logger().debug(f"Published: Lat: {msg.latitude}, Long: {msg.longitude}, Alt: {msg.altitude}m")


def main():
    rclpy.init()
    gps = GPSNode()
    gps.serial_listener()
    rclpy.spin(gps)
    rclpy.shutdown()



if __name__ == '__main__':
    main()
