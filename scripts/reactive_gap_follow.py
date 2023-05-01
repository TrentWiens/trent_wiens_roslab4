#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size = 10)
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = []
        i = 1
        maxDist = 3
        
        for r in ranges:
            if np.isnan(r):
                proc_ranges.append(0)
            elif r > maxDist or np.isinf(r):
                proc_ranges.append(maxDist)
            else: 
                proc_ranges.append(0)
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
       
        ranges = free_space_ranges
        
        currentStart = 0
        currentSize = 0
        maxStart = 0
        maxSize = 0
        currentIndex = 0
        
        while currentIndex < len(ranges):
            currentStart = currentIndex
            currentSize = 0
            
            while currentIndex < len(ranges) and ranges[currentIndex] > .1:
                currentSize = currentSize + 1
                currentIndex = currentIndex + 1
                
            if currentSize > maxSize:
                maxStart = currentStart
                maxSize = currentSize
                currentSize = 0
                
            currentIndex = currentIndex + 1
            
        if currentSize > maxSize: 
            maxStart = currentStart
            maxSize = currentSize
            
        return maxStart, maxStart + maxSize -1
        
        """a = 1
        index = []
        for r in ranges:
        	if r > np.amax(ranges) - 2: 
        		index.append(a)
       		a = a + 1
        	
        gapEdgeFront = [index[0]]
        gapEdgeBack = []
        iPrev = 0
        for i in index: 
        	if i - iPrev > 1: 
        		gapEdgeFront.append(i)
        		gapEdgeBack.append(iPrev)
        	iPrev = i
        	
        gapEdgeBack.append(index[-1])
        
        gapSize = 0
        count = 0
        
        print("----------------------")
        print(gapEdgeFront)
        print(gapEdgeBack) 
        
        for gap in gapEdgeFront:
        	if gapEdgeBack[count] - gap > gapSize:
        		gapSize = gapEdgeBack[gapEdgeFront.index(gap)] - gap
        		gapNumber = count
        		
        	count = count + 1
        		
        print(gapSize) 
        print(gapEdgeFront[gapNumber])
        print(gapEdgeBack[gapNumber])
        print(gapNumber)
        
        start_i = gapEdgeFront[gapNumber]
        end_i = gapEdgeBack[gapNumber]
        
        return start_i, end_i"""
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        
        pointInd = (end_i + start_i) /2
        
        return pointInd
        
        """
        gapSize = end_i - start_i
        
        indAdd = 0
        maxPoint = 0;
        pointInd = 0;
        while indAdd < gapSize: 
        	if ranges[start_i + indAdd] > maxPoint: 
        		maxPoint = ranges[start_i + indAdd]
        		pointInd = start_i + indAdd
        	indAdd = indAdd + 1
        	
        if pointInd > end_i: 
            pointInd = end_i
        	
        return maxPoint, pointInd"""


    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        minPoint = np.amin(proc_ranges)
        
        minPointIndex = proc_ranges.index(minPoint)
        
        r = 10
        
        maxMPI = minPointIndex + r
        minMPI = minPointIndex - r
        
        i = 0
        while i < (r*2 + 1):
            index = minMPI + i
            if index < len(ranges):
                proc_ranges[index] = 0
                i = i + 1
            else:
                i = i + 1
        
        start_i, end_i = self.find_max_gap(proc_ranges)
        
        print('------------')
        print(start_i)
        print(end_i)
        
        pointInd = self.find_best_point(start_i, end_i, ranges)
        
        minAng = data.angle_min
        angInc = data.angle_increment
        maxRange = data.range_max
        minRange = data.range_min
        
        angle = minAng + angInc * pointInd
        
        if angle > math.pi:
            angle = angle - 2 * math.pi
            
        print(pointInd)
        
        midPoint = len(ranges)/2
        
        if pointInd < midPoint: 
            """need neg steering angle"""
        
            steeringAngle = -angInc * (midPoint - pointInd)
        
        else: 
            """ need pos steering angle"""
            steeringAngle = angInc * (midPoint - pointInd)
            
        if steeringAngle > 1.57:
            steeringAngle = 1.57
        elif steeringAngle < -1.57:
            steeringAngle = -1.57
            
        steeringAngleDeg = steeringAngle * 180/math.pi
        print(midPoint)
        print(steeringAngleDeg)
        
        
        
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steeringAngle
        drive_msg.drive.speed = 1
        self.drive_pub.publish(drive_msg)
        

        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 

        #Find max length gap 

        #Find the best point in the gap 

        #Publish Drive message

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
