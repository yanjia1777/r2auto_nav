# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time
import ast

# constants
rotatechange = 0.5
speedchange = 0.2
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.35
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
smaller_angle = 25
smaller_angles = range(-smaller_angle,smaller_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt' 

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians


class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        self.temp_subscriber = self.create_subscription(String, 'temperature', self.temp_callback, 10)
        
        #create publisher for ang for servo motor(1st Adjust)
        self.ang_publisher = self.create_publisher(String, 'Angu', 8)
        
        #create publisher for ang for servo motor(2nd Adjust)
        self.ang1_publisher = self.create_publisher(String, 'Angu1', 9)
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

#update temperature and temperature_array values
    def temp_callback(self, msg):
        global temperature
        global temperature_array
        temperature = ast.literal_eval(msg.data)
        temperature_array = [(temperature[0],temperature[1],temperature[2],temperature[3]),(temperature[4],temperature[5],temperature[6],temperature[7]),(temperature[8],temperature[9],temperature[10],temperature[11]),(temperature[12],temperature[13],temperature[14],temperature[15])]
        # print(temperature_array)
    
    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        # np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def pick_direction(self):
        # self.get_logger().info('In pick_direction')
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmax(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        self.rotatebot(float(lr2i))

        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)
        
    def moveforward(self):
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.0
        time.sleep(1)
        self.publisher_.publish(twist)


    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            self.pick_direction()
            toPublish1 = String()
            while rclpy.ok():
                 if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        for j in range(360):                   
                            self.rotatebot(1)
                            time.sleep(0.01)
                            rclpy.spin_once(self)
                            if max(temperature) >= 86:
                                self.stopbot()
                                break
                        rclpy.spin_once(self)
                        if max(temperature) >=86:
                            print('1st Calibration')
                            print(max(temperature))
                            if (max(temperature) == temperature_array[0][0]): #Top Left array
                                print('[0][0]')    
                                self.rotatebot(18)      #Move by 2 pixel units to the right
                                n=(90-23)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif (max(temperature) == temperature_array[0][1]):#More than or equal to 45 degree
                                print('[0][1]')    
                                self.rotatebot(-9)      #Move by 1 pixel units to the right
                                n=(90-23)
                                ang = (n/180*10) +2.5                     
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif (max(temperature) == temperature_array[0][2]):
                                print('[0][2]')    
                                self.rotatebot(9)
                                n=(90-23)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif  (max(temperature) == temperature_array[0][3]):
                                print('[0][3]')    
                                self.rotatebot(-18)
                                n=(90-23)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif (max(temperature) == temperature_array[1][0]):
                                print('[1][0]')    
                                self.rotatebot(18)
                                n=(90-12)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif  (max(temperature) == temperature_array[1][1]):
                                print('[1][1]')    
                                self.rotatebot(-9)
                                n=(90-12)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif  (max(temperature) == temperature_array[1][2]):
                                print('[1][2]')
                                self.rotatebot(-9)
                                n=(90-12)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif (max(temperature) == temperature_array[1][3]):
                                print('[1][3]')
                                self.rotatebot(-18)
                                n=(90-12)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif  (max(temperature) == temperature_array[2][0]):
                                print('[2][0]')
                                self.rotatebot(18)
                                n=(90+12)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                #self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif (max(temperature) == temperature_array[2][1]):
                                print('[2][1]')
                                self.rotatebot(9)
                                n=(90+12)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif (max(temperature) == temperature_array[2][2]):
                                print('[2][2]')
                                self.rotatebot(-9)
                                n=(90+12)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif (max(temperature) == temperature_array[2][3]):
                                print('[2][3]')
                                self.rotatebot(-18)
                                n=(90+12)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif (max(temperature) == temperature_array[3][0]):
                                print('[3][0]')
                                self.rotatebot(18)
                                n=(90+23)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif (max(temperature) == temperature_array[3][1]):
                                print('[3][1]')
                                self.rotatebot(9)
                                n=(90+23)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif (max(temperature) == temperature_array[3][2]):
                                print('[3][2]')
                                self.rotatebot(3)
                                n=(90+23)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            elif (max(temperature) == temperature_array[3][3]):
                                print('[3][3]')
                                self.rotatebot(-18)
                                n=(90+23)
                                ang = (n/180*10) +2.5
                                toPublish1.data = str(ang)
                                self.ang_publisher.publish(toPublish1)
                                time.sleep(2)
                            lri1 = (self.laser_range[front_angles]>=float(0.6)).nonzero()
                            lri2 = (self.laser_range[front_angles]<float(0.6)).nonzero() 
                            if(len(lri1[0])>0):
                                while 1:
                                    self.moveforward()
                                    print('Moving Closer')
                                    rclpy.spin_once(self)
                                    lri2 = (self.laser_range[front_angles]<float(0.6)).nonzero()
                                    if(len(lri2[0])>0):
                                        self.stopbot()
                                        break
                            lri2 = (self.laser_range[front_angles]<float(0.6)).nonzero()
                            if(len(lri2[0])>0):
                                self.stopbot()
                                print('Pray Hard')
                                rclpy.spin_once(self)
                                print(max(temperature))
                                if max(temperature) >=86:
                                    print('2nd Calibration where with Moving Forward')
                                    print(max(temperature))
                                    if (max(temperature) == temperature_array[0][0]): #Top Left array
                                        print('[0][0]')    
                                        self.rotatebot(18)      #Move by 2 pixel units to the right
                                        n=(90-23)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif (max(temperature) == temperature_array[0][1]):#More than or equal to 45 degree
                                        print('[0][1]')    
                                        self.rotatebot(-9)      #Move by 1 pixel units to the right
                                        n=(90-23)
                                        ang = (n/180*10) +2.5                     
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif (max(temperature) == temperature_array[0][2]):
                                        print('[0][2]')    
                                        self.rotatebot(9)
                                        n=(90-23)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif  (max(temperature) == temperature_array[0][3]):
                                        print('[0][3]')    
                                        self.rotatebot(-18)
                                        n=(90-23)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif (max(temperature) == temperature_array[1][0]):
                                        print('[1][0]')    
                                        self.rotatebot(18)
                                        n=(90-12)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif  (max(temperature) == temperature_array[1][1]):
                                        print('[1][1]')    
                                        self.rotatebot(-9)
                                        n=(90-12)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif  (max(temperature) == temperature_array[1][2]):
                                        print('[1][2]')
                                        self.rotatebot(-9)
                                        n=(90-12)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif (max(temperature) == temperature_array[1][3]):
                                        print('[1][3]')
                                        self.rotatebot(-18)
                                        n=(90-12)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif  (max(temperature) == temperature_array[2][0]):
                                        print('[2][0]')
                                        self.rotatebot(18)
                                        n=(90+12)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif (max(temperature) == temperature_array[2][1]):
                                        print('[2][1]')
                                        self.rotatebot(9)
                                        n=(90+12)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif (max(temperature) == temperature_array[2][2]):
                                        print('[2][2]')
                                        self.rotatebot(-9)
                                        n=(90+12)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif (max(temperature) == temperature_array[2][3]):
                                        print('[2][3]')
                                        self.rotatebot(-18)
                                        n=(90+12)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif (max(temperature) == temperature_array[3][0]):
                                        print('[3][0]')
                                        self.rotatebot(18)
                                        n=(90+23)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif (max(temperature) == temperature_array[3][1]):
                                        print('[3][1]')
                                        self.rotatebot(9)
                                        n=(90+23)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif (max(temperature) == temperature_array[3][2]):
                                        print('[3][2]')
                                        self.rotatebot(3)
                                        n=(90+23)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                                    elif (max(temperature) == temperature_array[3][3]):
                                        print('[3][3]')
                                        self.rotatebot(-18)
                                        n=(90+23)
                                        ang = (n/180*10) +2.5
                                        toPublish1.data = str(ang)
                                        self.ang1_publisher.publish(toPublish1)
                                        time.sleep(10)
                        rclpy.spin_once(self)
                        lri2 = (self.laser_range[front_angles]<float(0.6)).nonzero()
                        if(len(lri2[0])<0):
                            if max(temperature) >=86:
                                print('3rd Calibration without moving forward')
                                print(max(temperature))
                                if (max(temperature) == temperature_array[0][0]): #Top Left array
                                    print('[0][0]')    
                                    self.rotatebot(18)      #Move by 2 pixel units to the right
                                    n=(90-23)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif (max(temperature) == temperature_array[0][1]):#More than or equal to 45 degree
                                    print('[0][1]')    
                                    self.rotatebot(-9)      #Move by 1 pixel units to the right
                                    n=(90-23)
                                    ang = (n/180*10) +2.5                     
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif (max(temperature) == temperature_array[0][2]):
                                    print('[0][2]')    
                                    self.rotatebot(9)
                                    n=(90-23)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif  (max(temperature) == temperature_array[0][3]):
                                    print('[0][3]')    
                                    self.rotatebot(-18)
                                    n=(90-23)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif (max(temperature) == temperature_array[1][0]):
                                    print('[1][0]')    
                                    self.rotatebot(18)
                                    n=(90-12)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif  (max(temperature) == temperature_array[1][1]):
                                    print('[1][1]')    
                                    self.rotatebot(-9)
                                    n=(90-12)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif  (max(temperature) == temperature_array[1][2]):
                                    print('[1][2]')
                                    self.rotatebot(-9)
                                    n=(90-12)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif (max(temperature) == temperature_array[1][3]):
                                    print('[1][3]')
                                    self.rotatebot(-18)
                                    n=(90-12)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif  (max(temperature) == temperature_array[2][0]):
                                    print('[2][0]')
                                    self.rotatebot(18)
                                    n=(90+12)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif (max(temperature) == temperature_array[2][1]):
                                    print('[2][1]')
                                    self.rotatebot(9)
                                    n=(90+12)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif (max(temperature) == temperature_array[2][2]):
                                    print('[2][2]')
                                    self.rotatebot(-9)
                                    n=(90+12)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif (max(temperature) == temperature_array[2][3]):
                                    print('[2][3]')
                                    self.rotatebot(-18)
                                    n=(90+12)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif (max(temperature) == temperature_array[3][0]):
                                    print('[3][0]')
                                    self.rotatebot(18)
                                    n=(90+23)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif (max(temperature) == temperature_array[3][1]):
                                    print('[3][1]')
                                    self.rotatebot(9)
                                    n=(90+23)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif (max(temperature) == temperature_array[3][2]):
                                    print('[3][2]')
                                    self.rotatebot(3)
                                    n=(90+23)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                                elif (max(temperature) == temperature_array[3][3]):
                                    print('[3][3]')
                                    self.rotatebot(-18)
                                    n=(90+23)
                                    ang = (n/180*10) +2.5
                                    toPublish1.data = str(ang)
                                    self.ang1_publisher.publish(toPublish1)
                                    time.sleep(10)
                        # find direction with the largest distance from the Lidar
                        # rotate to that direction
                        # start moving
                        print('continue on')
                        self.pick_direction()
                    
                # allow the callback functions to run
                 rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()

def main(args=None):
    rclpy.init(args=args)
    global auto_nav
    auto_nav = AutoNav()
 #   toPublish = String()
    # while 1:
    #     toPublish.data = '120'
    #     print(toPublish)
    #     print('')
    #     auto_nav.ang_publisher.publish(toPublish)
            
    auto_nav.mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
