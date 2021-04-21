#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 20 20:15:37 2021

@author: yanjia1777
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
from copy import copy

import matplotlib.pyplot as plt
from PIL import Image

import scipy.stats


#from getFilter import getfrontier 

# constants
rotatechange = 0.1
speedchange = 0.1
#cali = 0

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


class autoNav(Node):
    def __init__(self):
        super().__init__('auto_nav')
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
        self.path = [[[]]]*50
        self.pos = np.array([])
        self.co = 0
        
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
        #self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
        # convert quaternion to Euler angles
        roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        #self.get_logger().info('Rot-Yaw: Rad: %f Deg: %f' % (yaw, np.degrees(yaw)))
        
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
        
        """
        
        print(frontierCells)
        for row in binary_grid_edges:
            print(row)
        
        """
               
        for index,frontiers in enumerate(frontierCells):
            startPos = mBotPos
            endPos = tuple(frontiers)
            #print(startPos,endPos)
            path = astar(binary_grid_edges, startPos, endPos, aStarConfig)
            #print(path)
            self.path[index] = path
        
        print("Frontiers left " + str(len(frontierCells)))
        
        """
        if(self.co==0):
            self.pos = mBotPos
        elif (mBotPos[0]-10)<mBotPos[0]<(mBotPos[0]+10):
            if (mBotPos[1]-10)<mBotPos[0]<(mBotPos[0]+10):
                self.pos[0] = mBotPos[0]
                self.pos[1] = mBotPos[1]
                self.co = 1
        """
        self.pos = mBotPos
        
        #print(self.path)
        
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
        
        #img = Image.fromarray(frontierImg)
        # show the image using grayscale map
        #plt.imshow(img, cmap='gray', origin='lower')
        #plt.draw_all()
        
        #plt.pause(0.00000000001)

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
        target_yaw = math.radians(rot_angle) #changed
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
        
    def moving(self,cali):
        twist = Twist()
        path = self.path[0]
        
        print(path)
        if path:
            for (index,cord) in enumerate(range(1,len(path))):
                xshift = path[index][0] - self.pos[0]
                yshift = path[index][1] - self.pos[1]
                if(xshift == 0):
                    xshift+=1

                print('start')
                print('direction = ' + str(xshift))
                print('direction = ' + str(yshift))
                print("Target Pos = " + str(path[index]))  
                if(yshift!=0 or xshift!=0):
                    ang = math.degrees(math.atan(yshift/xshift))
                    print("initial " + str(math.degrees(cali)))
                    if(xshift<0):
                        if(yshift>0):
                            ang = 180 + ang
                        else:
                            ang = ang - 180
                    angg = ((ang+math.degrees(cali)+180)%360)-180
                    self.rotatebot(angg)

                    print('forrr')
                    
                    count = 0
                    twist.linear.x = speedchange
                    self.publisher_.publish(twist)
                    while((abs(self.pos[0]-path[index][0])>0 or abs(self.pos[1]-path[index][1])>0) and count <190):

                        rclpy.spin_once(self)
                        #print("Bot Pos = " + str(self.pos))
                        #print("Target Pos = " + str(path[index]))
                        count += 1
                        if(count==190):
                            print("limit reached")
                    print("Bot Pos = " + str(self.pos))
                    
                    self.stopbot()
                
        print("GAAAAAAAAAH")



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
                    #lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty
                    #print(self.path)
                    check = 0
                    if(check == 0):
                        cali = self.yaw
                        check = 1
                    
                    self.moving(cali)
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

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        #print("INIT")
        self.parent = parent
        self.position = position
        #print(position)
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    
def astar(occupancy_grid, start, end, aStarConfig):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    
    # check what is occupied and not
    occ_val = aStarConfig["occupied"]
    vac_val = aStarConfig["vacant"]


    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []
    open_set = set()
    closed_set = set()

    # Add the start node
    open_list.append(start_node)
    open_set.add(start)

    # Loop until you find the end
    while len(open_list) > 0 and len(open_set) > 0:
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        open_set.remove(current_node.position)
        closed_list.append(current_node)
        closed_set.add(current_node.position)

        # Found the goal
        if current_node == end_node:
            break

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(occupancy_grid) - 1) or node_position[0] < 0 or node_position[1] > (len(occupancy_grid[len(occupancy_grid)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if occupancy_grid[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            
            if child.position in closed_set:
                continue
            
            """
            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue
            """
            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h
            
            if child.position in open_set:
                continue
            # Child is already in the open list
            for open_node in open_list:
                if child.g > open_node.g:
                #if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)
            open_set.add(child.position)
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    return path[::-1] # Return reversed path

occ_bins_all_edge = [-1,0.1,100]
occ_bins_occupied = [-1,50,100]
occ_bins_within = [-1,0,100] 
original_occ_bins = [-1, 0, 50,100]
cannyThresh = 250
ALTHRESH = 10
DILATE_PIXELS = 5
ERODE_PIXELS = 7

def getfrontier(mapData,botPos):
    print("Getting frontiers")
    data=mapData.data
    w=mapData.info.width
    h=mapData.info.height
    resolution=mapData.info.resolution
    Xstartx=mapData.info.origin.position.x
    Xstarty=mapData.info.origin.position.y
    
    
    checkOcc = np.uint8(np.array(data).reshape(h,w))
    #return checkOcc 
    occ_counts, edges, binnum = scipy.stats.binned_statistic(np.array(data), np.nan, statistic='count', bins=occ_bins_occupied)
    
    """
    for idx,x in enumerate(binnum):
        if (binnum[idx] == 1):
            binnum[idx] = 0
        elif (binnum[idx] == 2):
            binnum[idx] = 25
        else:
            binnum[idx] = 255
     """
    for idx,x in enumerate(binnum):
        if (binnum[idx] == 1):
           binnum[idx] = 0
        else:
           binnum[idx] = 255
    binned_grid = np.uint8(binnum.reshape(h,w))
    #element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(ERODE_PIXELS,ERODE_PIXELS))
    #img4 = cv2.erode(binned_grid,element)
    #return binned_grid
    #ret,img2 = cv2.threshold(checkOcc,2,255,0)
    element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(DILATE_PIXELS,DILATE_PIXELS))
    img4 = cv2.dilate(binned_grid,element)
    #return img4
    canny_output = cv2.Canny(checkOcc, 225, 250)
    #return canny_output
    #getUnknownEdgesGrid(checkOcc,canny_output,w,h)
    edge_output = getUnknownEdgesGrid2(canny_output,img4,w,h)
    
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    edge_output = cv2.dilate(edge_output,element)
    #return edge_output
    contours, hierarchy = cv2.findContours(edge_output,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    #main_contour = contours[3]
    #contoured_image = cv2.drawContours(edge_output, [main_contour], 0, (0,255,0), 1)
    contoured_image = cv2.drawContours(edge_output, contours, -1, (255,255,255), 1)
    #return contoured_image
    all_pts=[]
    all_world_pts = []
    print(len(contours))
    if len(contours)>0:
        for i in range(0,len(contours)):
                cnt = contours[i]
                M = cv2.moments(cnt)
                cx = int(M['m10']/(M['m00']+ 1e-5))
                cy = int(M['m01']/(M['m00']+ 1e-5))
                xr=cx*resolution+Xstartx
                yr=cy*resolution+Xstarty
                pt=[np.array((cx,cy))]
                w_pt = [np.array((xr,yr))]
                if len(all_pts)>0:
                    all_pts=np.vstack([all_pts,pt])
                    all_world_pts = np.vstack([all_world_pts,w_pt])
                else:
                    all_pts=pt
                    all_world_pts = w_pt
    #print(all_pts)
    res = np.zeros((h,w),dtype=np.uint8)
    
    for points in all_pts:
        res[points[1]][points[0]] = 255
        
    #print("World origin: ", Xstartx, Xstarty)
    #for world_points in all_world_pts:
        #print(world_points[0], " ", world_points[1])
    
    mBotPos = worldToMap(botPos,mapData.info.origin.position,resolution)
    print("Bot position in cells: " + str(mBotPos[0]) + " " + str(mBotPos[1]))
    res[mBotPos[1]][mBotPos[0]] = 255
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(7,7))        
    res = cv2.dilate(res,element)
    contoured_image += res;
    return contoured_image,all_pts, mBotPos,binned_grid
    #o=cv2.bitwise_not(o) 
    #res = cv2.bitwise_and(o,edges)
    

def mapToWorld(coords,world_origin, resolution):
    wx = world_origin.x + coords.x * resolution
    wy = world_origin.y + coords.y * resolution
    return (wx,wy)


def worldToMap(coords, world_origin, resolution):
    if (coords.x < world_origin.x or coords.y < world_origin.y):
        print("Does this imply smth")
    mx = int((coords.x - world_origin.x) // resolution)
    my = int((coords.y - world_origin.y) // resolution)
    return (mx,my)



def getUnknownEdgesGrid2(canny_grid, binned_grid, width, height):
    #must line up properly so nah 
    res = []
    #print(canny_grid.shape)
    #print(binned_grid.shape)
    for row in range(height):
        temprow = []
        for col in range(width):
            if (canny_grid[row][col]  < binned_grid[row][col]):
                temp = 0
            else:
                temp = canny_grid[row][col] - binned_grid[row][col]
            temprow.append(temp)
        res.append(temprow)
        #print(binned_grid[row])
    return np.array(res,dtype=np.uint8)


def getUnknownEdgesGrid(occupancy_grid, edges_grid, width, height):
    EXPLORE_RANGE = 3
    padded_occupancy = copy(occupancy_grid)
    horizontal_padding = np.zeros((height,EXPLORE_RANGE),dtype=np.uint8)
    vertical_padding = np.zeros((EXPLORE_RANGE,width + EXPLORE_RANGE * 2),dtype=np.uint8)
    padded_occupancy = np.hstack((horizontal_padding,padded_occupancy,horizontal_padding))
    padded_occupancy = np.vstack((vertical_padding,padded_occupancy,vertical_padding))
    checked_cells = set() #set of coordinates in tuple
    res = [] #grid of edges
    for row in range(height):
        for col in range(width):
            if(edges_grid[row][col] != 0):
                print(row,col)
                checkPixels(occupancy_grid,EXPLORE_RANGE,(row,col))
    
def checkPixels(occupancy_grid,explore_range,coord):
    #heck care about zero index, cuz got enough padding. Actually good point hor, pad image then wont have zero index
    print("Checking coord:" + str(coord[0]) + " " + str(coord[1]))
    curPixel_x = coord[0]
    curPixel_y = coord[1]
    #check for vertical boundary
    if(occupancy_grid[curPixel_x-explore_range][curPixel_y] != occupancy_grid[curPixel_x+explore_range][curPixel_y]):
        print(occupancy_grid[curPixel_x-explore_range][curPixel_y])
        print(occupancy_grid[curPixel_x+explore_range][curPixel_y])


def main(args=None):
    rclpy.init(args=args)

    auto_nav = autoNav()
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