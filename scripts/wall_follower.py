#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3

#set global variable for minimum distance
minDist = 0.2


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

            #print("The closest Distance is ", closeDist)
            #print("The closest angle is ", closeAngle)

            #if the robot is too far or too close to the wall
            #implement person follower to get close to the wall
            if closeDist > .3 or closeDist <= -.1:
                myLinear = Vector3(.3 * closeDist, 0, 0)

                if closeAngle <= 180:
                    #if angle is in the first 180 degrees, turn that way
                    angProp = closeAngle
                else: 
                    #if angle is in the next 180, we want to turn the other way,
                    #so negative rotation
                    angProp = (closeAngle - 360)
                
                #proportional angular speed, largest is 180 so
                #.01 makes sense, don't want robot going faster than .4 so set minimum
                myAngular = Vector3(0, 0, 
                    np.minimum(.01 * angProp, .4))

            else: #if close to wall, put it perpidicular and move forward
                range = 5 #degree angle to wall to drive forward
                #if angle is close to right, drive forwad
                if closeAngle >= (90 - range) and closeAngle <= (90 + range):
                    myLinear = Vector3(.15, 0, 0)
                #if angle is off, don't drive forward just rotate    
                else:
                    myLinear = Vector3(0, 0, 0)
                #keep wall at 90 degrees
                myAngular = Vector3(0, 0, 
                    np.minimum(.02 * (closeAngle - 90), .5))
                    #never want it to turn too fast
            

            my_twist = Twist(
                #move forward at constant speed
                linear = myLinear,
                angular = myAngular
            )
            # allow the publisher enough time to set up before publishing the first msg
            #rospy.sleep(0.5)

            # publish the message
            self.robot_movement_pub.publish(my_twist)
            #print(my_twist)

        def run(self):
            rospy.spin()
                
if __name__ == '__main__':
        #initialize node and run code
        rospy.init_node('wall_follower')
        follower = Follower()
        follower.run()