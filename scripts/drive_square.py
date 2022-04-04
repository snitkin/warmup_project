#!/usr/bin/env python3

import rospy

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3
from math import pi #for angles

class DriveSquare(object):
    """ This node publishes ROS messages containing the 3D coordinates of a single point """

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('square_drive')
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):

        rospy.Rate(10).sleep()

        for i in range (0, 4): #loop through 4 times to complete square
            # setup the Twist message we want to send
            my_twist = Twist(
                linear=Vector3(0.2, 0, 0), #move forward
                angular=Vector3(0, 0, 0)
            )

            # allow the publisher enough time to set up before publishing the first msg
            rospy.sleep(1)

            # publish the message
            self.robot_movement_pub.publish(my_twist)

            #do action for 9 seconds
            rospy.sleep(4)

            # setup the Twist message we want to send
            my_twist = Twist(
                linear=Vector3(0, 0, 0),
                angular=Vector3(0, 0, pi/10) #turn 90 degrees
            )

            # allow the publisher enough time to set up before publishing the first msg
            rospy.sleep(1)

            # publish the message
            self.robot_movement_pub.publish(my_twist)

            #do action for 9 seconds
            rospy.sleep(4)
        
        stop_twist = Twist(
                linear=Vector3(0, 0, 0),
                angular=Vector3(0, 0, 0) #stop turning
            )
        # allow the publisher enough time to set up before publishing the first msg
        rospy.sleep(1)

        # publish the message
        self.robot_movement_pub.publish(stop_twist)


if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()