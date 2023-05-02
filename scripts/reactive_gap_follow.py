#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float32

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
        
        front_angles = []
        count = 0 
        for r in ranges:
            if count < len(ranges)/4 or count > 3*len(ranges)/4:
                front_angles.append(0)
            else:
                front_angles.append(1)
            count = count + 1


        proc_ranges = []
        i = 0
        maxDist = 6
        minDist = 1
        
        for r in ranges:
            if front_angles[i]:
                if np.isnan(r) or r < minDist:
                    proc_ranges.append(0)
                elif r > maxDist or np.isinf(r):
                    proc_ranges.append(maxDist)
                else: 
                    proc_ranges.append(r)
            else:
                proc_ranges.append(0)
            i = i + 1
            

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
            
        return maxStart, maxStart + maxSize - 1

    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        
	pointInd = (end_i + start_i) / 2 

        farInd = 0
        i = 0
        
        for r in ranges:
            if i >= start_i and i <= end_i:
                if ranges[i] > ranges[pointInd]:
                    farInd = i
            i = i + 1
        
        return pointInd, farInd
      
    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        #print(proc_ranges)
        
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
        

        pointInd, farInd = self.find_best_point(start_i, end_i, proc_ranges)
        
        minAng = data.angle_min
        angInc = data.angle_increment
        maxRange = data.range_max
        minRange = data.range_min
        
        angle = minAng + angInc * pointInd
        
       # if angle > math.pi:
        #    angle = angle - 2 * math.pi
        
        midPoint = len(ranges)/2
        

        steeringAngle = angInc * (midPoint + pointInd) - 2 * np.pi

            
        #if steeringAngle > 1.57:
         #   steeringAngle = 1.57
        #elif steeringAngle < -1.57:
         #   steeringAngle = -1.57
            
        steeringAngleDeg = steeringAngle * 180/math.pi

        farthest = ranges[farInd]
        
        if minPoint > 4: 
            velocity = .5
        else:
            velocity = 1

        
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steeringAngle
        drive_msg.drive.speed = velocity
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
