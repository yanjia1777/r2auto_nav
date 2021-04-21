#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 20 20:15:37 2021

@author: facestomperx
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time
import cv2
import random as rng
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from aStarPlanner import astar

import matplotlib.pyplot as plt
from PIL import Image

import scipy.stats


from getFrontierAndCentroids import getfrontier 
#from getFilter import getfrontier 

# constants
rotatechange = 0.1
speedchange = 0.05

occ_bins = [-1, 0, 50,100]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'


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


class bokuNoNav(Node):
    def __init__(self):
        super().__init__('boku_no_nav')
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        
    #callbacks
        
    def odom_callback(self, msg):
        #self.get_logger().info('In odom_callback')
        #print(msg)
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        

    def occ_callback(self, msg):
        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transformation found')
            return
        
        #print(trans)
        
        cur_pos = trans.transform.translation
        cur_rot = trans.transform.rotation
        self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        self.get_logger().info('Rot-Yaw: Rad: %f Deg: %f' % (yaw, np.degrees(yaw)))
        
        frontierImg, frontierCells, mBotPos, grid_edges = getfrontier(msg,cur_pos)
        print("FRONTIERS")
        
        aStarConfig = {
            "occupied": 1,
            "vacant": 0
        }
        binary_grid_edges = []
        for row in grid_edges:
            temp = []         
            for col in row:
                if col == 255:
                    temp.append(1)
                else:
                    temp.append(0)
            binary_grid_edges.append(temp)
        
        print(frontierCells)
        for row in binary_grid_edges:
            print(row)
        """
        for frontiers in frontierCells:
            startPos = mBotPos
            endPos = tuple(frontiers)
            print(startPos,endPos)
            path = astar(binary_grid_edges, startPos, endPos, aStarConfig)
            print(path)
        """
        
        
        occdata = np.array(msg.data)
        #print(occdata.shape)
        checkOcc = np.uint8(occdata.reshape(msg.info.height,msg.info.width))
        #for i,x in enumerate(checkOcc):
        #    print(x)
            #for j,y in enumerate(x):
            #    checkOcc[i][j] = math.floor(1 / (y+0.1) * 255) 
            #print(x)
        
        occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
        iwidth = msg.info.width
        iheight = msg.info.height
        total_bins = iwidth * iheight
        #print(occ_counts)
        #self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0], occ_counts[1], occ_counts[2], total_bins))
        #print(msg.info.height,msg.info.width)
        odata = np.uint8(binnum.reshape(msg.info.height,msg.info.width))
        
        
        """
        fc = cv2.findContours(img4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        (major, minor, _) = cv2.__version__.split(".")
        if(major == '3'):
            contours = fc[1]
            hierarchy = fc[2]
        else:
            contours = fc[0]
            hierarchy = fc[1]
        lc = len(contours)
        print(len(contours))
        # Draw contours
        drawing = np.zeros((msg.info.height, msg.info.width, 3), dtype=np.uint8)
        for i in range(len(contours)):
            color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
            cv2.drawContours(drawing, contours, i, color, 2, cv2.LINE_8, hierarchy, 0)
            """
        """
        for i in range(lc):
            points = contours[i]
            for point in points:
                print(point)
                x = point[0] 
                y = point[1] 
                contourImg[x][y] = 1
        """
        
        img = Image.fromarray(frontierImg)
        # show the image using grayscale map
        plt.imshow(img, cmap='gray', origin='lower')
        plt.draw_all()
        
        plt.pause(0.00000000001)

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan


    """
     * Get consecutive unknown cells that are of turtlebot dimmension
     * @param[in] map 2d array representing map
     * @param[out] list of indexes in map that are unknown contours
    """
    def get_unknown_contours(self,map):
        print("get_unknown_contours")
        start_pos,map_edge = self.find_overall_edge(map)
       # print(start_pos)
        #print(map_edge.shape())
        #print(map_edge)
        map_edge_array = np.asarray(map_edge)
        return map

    """"
     * Either shrink and dilate (opencv), so called edge detection,
     * or for now just scan thru and find values other than unknown
     *
     *
    """
    def find_overall_edge(self,map):
        #map_edge = []
        start_pos = [0,0]
        for i,row in enumerate(map):
            #current_row = []
            #print(row)
            for j,col in enumerate(row):
                if col == 1 or col == 2:
                    #current_row.append(UNKNOWN_CELL)
                    map[i,j] = 1
                elif col == 3:
                    #current_row.append(KNOWN_CELL)
                    map[i,j] = 2
                    start_pos = [i,j]
                    
                    
                
            
        return [start_pos,map]
    
    
    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)
    
    
    
    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            #self.pick_direction()

            while rclpy.ok():
                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty
                    """
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        # find direction with the largest distance from the Lidar
                        # rotate to that direction
                        # start moving
                        self.pick_direction()
                    """
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

    auto_nav = bokuNoNav()
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