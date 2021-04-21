#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 28 21:34:39 2021

@author: facestomperx
"""
#--------Include modules---------------
from copy import copy
#import rclpy
#from nav_msgs.msg import OccupancyGrid

import numpy as np
import cv2
import scipy.stats
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
        
    print("World origin: ", Xstartx, Xstarty)
    for world_points in all_world_pts:
        print(world_points[0], " ", world_points[1])
    
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
    print(canny_grid.shape)
    print(binned_grid.shape)
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
    
    
