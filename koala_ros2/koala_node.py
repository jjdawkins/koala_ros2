#! /usr/bin/python3
import rclpy
from rclpy.node import Node

import serial as s
import math
import time

from std_msgs.msg import Float32, String, UInt32MultiArray
from geometry_msgs.msg import Twist

class koalaDriver(Node):

    def __init__(self):
        super().__init__('koala_driver')

        self.track_width = 0.285 # Robot track track_width
        self.wheel_radius = 0.045 # 45 mm wheel_radius

        self.ser = s.Serial('/dev/ttyUSB0',38400,bytesize=s.EIGHTBITS, parity=s.PARITY_NONE, stopbits=s.STOPBITS_TWO)

        self.vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velcmd_callback,
            10)

        self.vel_pub = self.create_publisher(Twist,'vel',10)
        self.bat_pub = self.create_publisher(Float32,'battery',10)
        self.prox_pub = self.create_publisher(UInt32MultiArray,'proximity',10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.run_loop)
        self.i = 0
        self.init_time = time.time()

    def velcmd_callback(self,msg):
        u = msg.linear.x
        r = msg.angular.z
        omL = int(u/self.wheel_radius - (self.track_width/(2*self.wheel_radius))*r)
        omR = int(u/self.wheel_radius + (self.track_width/(2*self.wheel_radius))*r)

        msg = 'D,'+str(omL)+','+str(omR)+'\n'

        #self.publisher_.publish(msg)
        print(msg)
        self.ser.write(msg.encode())
        ack = self.ser.read_until().decode()

    def readStatus(self):

        msg = 'M,0\n'
        self.ser.write(msg.encode())
        res0 = self.ser.read_until().decode()
        res0 = res0.strip()
        res0 = res0.split(',')
        volts = float(res0[1])*21.23/1000
        print(volts)

    def readProximity(self):
        msg = 'N\n'
        self.ser.write(msg.encode())

        prox = self.ser.read_until().decode()
        prox = prox.split(',')
        #prox = int(prox[1:15])
        prox_msg = UInt32MultiArray()
        for i in range(1,17):
            prox_msg.data.append(int(prox[i]))

        self.prox_pub.publish(prox_msg)
        #print(prox_msg.data)

    def readWheelSpeed(self):
        msg = 'E\n'
        self.ser.write(msg.encode())

        whl_spds = self.ser.read_until().decode()
        whl_spds = whl_spds.strip()
        whl_spds = whl_spds.split(',')
        print(whl_spds)
        wl = float(whl_spds[1])
        wr = float(whl_spds[2])
        vel_msg = Twist()
        vel_msg.linear.x = self.wheel_radius*(wl+wr)/2
        vel_msg.angular.z = self.wheel_radius*(wr-wl)/self.track_width
        self.vel_pub.publish(vel_msg)


    def run_loop(self):
        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        t = time.time()-self.init_time
        self.readProximity()
        self.readWheelSpeed()
        #self.readStatus()
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1



def main(args=None):
    rclpy.init(args=args)

    koala_driver = koalaDriver()

    rclpy.spin(koala_driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    koala_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
