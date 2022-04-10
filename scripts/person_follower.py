#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

#set global variable for minimum distance
minDist = 0.4


class Follower:

        def __init__(self):

                # subscribe to the robot's RGB camera data stream
                self.image_sub = rospy.Subscriber('scan',
                        LaserScan, self.laser_callback)
                
                # setup publisher to the cmd_vel ROS topic
                self.robot_movement_pub = rospy.Publisher('cmd_vel', 
                        Twist, queue_size=10)

        def laser_callback(self, data):

            myArr = np.array(data.ranges)

            #if value is 0, set to 60 because that's outside of range
            myArr[myArr == 0] = 60
            #get the angle and the distance of the closest object
            closeAngle = np.argmin(myArr)
            closeDist = np.min(myArr) - minDist

            myLinear = Vector3(.5 * closeDist, 0, 0)

            if closeAngle <= 180:
                #if angle is in the first 180 degrees, turn that way
                angProp = closeAngle
            else: 
                #if angle is in the next 180, we want to turn the other way,
                #so negative rotation
                angProp = (closeAngle - 360)
            
            #proportional angular speed, largest is 180 so
            #.01 makes sense, don't want robot going faster than 2
            myAngular = Vector3(0, 0, .01 * angProp)

            my_twist = Twist(
                #move forward at constant speed
                linear = myLinear,
                angular = myAngular
            )
            # allow the publisher enough time to set up before publishing the first msg
            #rospy.sleep(0.5)

            # publish the message
            self.robot_movement_pub.publish(my_twist)

        def run(self):
            rospy.spin()
                
if __name__ == '__main__':
        #initialize node and run code
        rospy.init_node('person_follower')
        follower = Follower()
        follower.run()