#! /usr/bin/python3
import rclpy
from rclpy.node import Node

import serial as s
import math
import time

from std_msgs.msg import Float32, String, UInt32MultiArray
from geometry_msgs.msg import Twist
from surfer_msgs.msg import Status

def saturation(val,min,max):
    
    if(val < min):
        val = min

    if(val > max):
        val = max
    
    return val

class koalaDriver(Node):

    def __init__(self):
        super().__init__('koala_driver')

        self.declare_parameter('max_speed',1.0)
        self.declare_parameter('max_yawrate',1.57)
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.max_yawrate = self.get_parameter('max_yawrate').get_parameter_value().double_value

        
        self.track_width = 0.285 # Robot track track_width
        self.wheel_radius = 0.045 # 45 mm wheel_radius
        self.armed = False

        self.ser = s.Serial('/dev/ttyUSB0',38400,bytesize=s.EIGHTBITS, parity=s.PARITY_NONE, stopbits=s.STOPBITS_TWO)

        #self.reset_odom_srv = self.create_service()

        self.vel_sub = self.create_subscription(Twist,'cmd_vel',self.velcmd_callback,10)
        self.stat_sub = self.create_subscription(Status,'status',self.status_callback,10)

        self.vel_pub = self.create_publisher(Twist,'vel',10)
        self.bat_pub = self.create_publisher(Float32,'battery',10)
        self.prox_pub = self.create_publisher(UInt32MultiArray,'proximity',10)
        self.light_pub = self.create_publisher(UInt32MultiArray,'light',10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.run_loop)

        self.init_time = time.time()

    def status_callback(self,msg):
        self.armed = msg.armed

    def velcmd_callback(self,msg):

        if(self.armed):
            u = saturation(msg.linear.x,-self.max_speed,self.max_speed)
            r = saturation(msg.angular.z,-self.max_yawrate,self.max_yawrate)
        else:
            u = 0
            r = 0

        omL = int(u/self.wheel_radius - (self.track_width/(2*self.wheel_radius))*r)
        omR = int(u/self.wheel_radius + (self.track_width/(2*self.wheel_radius))*r)

        msg = 'D,'+str(omL)+','+str(omR)+'\n'

        #self.publisher_.publish(msg)
        #print(msg)
        self.ser.write(msg.encode())
        ack = self.ser.read_until().decode()

    def configureSpeedPID(self,Kp,Ki,Kd):
        msg = 'A,'+str(Kp)+','+str(Ki)+','+str(Kd)+'\n'
        self.ser.write(msg.encode())
        res = self.ser.read_until().decode()
        if(res=='a'):
            return True
        else:
            return False

    def configurePositionPID(self,Kp,Ki,Kd):
        msg = 'F,'+str(Kp)+','+str(Ki)+','+str(Kd)+'\n'
        self.ser.write(msg.encode())
        res = self.ser.read_until().decode()
        if(res=='f'):
            return True
        else:
            return False  

    def setPositionCounter(self,left_val,right_val):
        msg = 'G,'+str(left_val)+','+str(right_val)+'\n'
        self.ser.write(msg.encode())
        res = self.ser.read_until().decode()
        if(res=='g'):
            return True
        else:
            return False    

    def readPositionCounter(self):
        msg = 'H\n'
        self.ser.write(msg.encode())
        res = self.ser.read_until().decode()
        res = res.strip()
        res = res.split(',')
        #TODO add publisher for encoder based odometry
        #print(res)

    def readADC(self,channel):
        msg = 'I,'+channel+'\n'
        self.ser.write(msg.encode())
        res = self.ser.read_until().decode()
        res = res.strip()
        res = res.split(',')
        print(res)        

        #TODO add interfaces to Read and Write from Digital


    def readStatus(self):
        msg = 'M,0\n'
        self.ser.write(msg.encode())
        res0 = self.ser.read_until().decode()
        res0 = res0.strip()
        res0 = res0.split(',')
        volts = float(res0[1])*21.23/1000

        bat_msg = Float32()
        bat_msg.data = volts
        self.bat_pub.publish(bat_msg)
        #print(res0)

    def readProximitySensors(self):
        msg = 'N\n'
        self.ser.write(msg.encode())

        prox = self.ser.read_until().decode()
        prox = prox.split(',')
        prox_msg = UInt32MultiArray()
        for i in range(1,17):
            prox_msg.data.append(int(prox[i]))

        self.prox_pub.publish(prox_msg)
        #print(prox_msg.data)

    def readLightSensors(self):
        msg = 'O\n'
        self.ser.write(msg.encode())

        light = self.ser.read_until().decode()
        light = light.split(',')
        light_msg = UInt32MultiArray()
        for i in range(1,17):
            light_msg.data.append(int(light[i]))

        self.light_pub.publish(light_msg)
     

    def readWheelSpeed(self):
        msg = 'E\n'
        self.ser.write(msg.encode())

        whl_spds = self.ser.read_until().decode()
        whl_spds = whl_spds.strip()
        whl_spds = whl_spds.split(',')
        #print(whl_spds)
        wl = float(whl_spds[1])
        wr = float(whl_spds[2])
        vel_msg = Twist()
        vel_msg.linear.x = self.wheel_radius*(wl+wr)/2
        vel_msg.angular.z = self.wheel_radius*(wr-wl)/self.track_width
        self.vel_pub.publish(vel_msg)


    def run_loop(self):

        t = time.time()-self.init_time
        self.readProximitySensors()
        self.readLightSensors()
        
        self.readWheelSpeed()

        self.readPositionCounter()

        #self.setPositionCounter(100,-100)
        self.readStatus()
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
