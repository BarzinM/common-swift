#!/usr/bin/env python

# localizer.py
import rospy
from geometry_msgs.msg import PoseStamped
from transformations import quaternion_from_euler
from communication import LabNavigation, setupLogger
from os import getcwd
import argparse
from sys import exit


class Localizer(object):

    def __init__(self, ip_address='192.168.0.25'):
        self.logger = setupLogger(verbose, 'root.communication')
        self.logger.debug("All the errors of communication module will be logged in %s", getcwd())

        self.logger.info('Initiating the localizer.')
        self.pos_publisher = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=2)
        try:
            rospy.init_node('lab_localizer', anonymous=False)
        except rospy.exceptions.ROSInitException:
            self.logger.warning('Master is not running. Initiation is not completed.')
            exit(1)
        self.rate = rospy.Rate(15)
        self.lab_navigation = LabNavigation()
        self.logger.info('Connection established to: %s', ip_address)

    def updatePose(self, num):
        position_msg = PoseStamped()
        position_msg.header.stamp = rospy.Time.now()

        states = self.lab_navigation.getStates(str(num))
        x_value = states[1]
        if x_value != x_value:
            self.logger.warning("Received NaN from camera system")
            print 'Recieved nan!!!!'
            return
        position_msg.pose.position.x = x_value
        position_msg.pose.position.y = states[2]
        position_msg.pose.position.z = states[3]
        roll = states[4]
        pitch = states[5]
        yaw = states[6]
        if verbose:
            print '%i: % .2f % .2f % .2f % .2f % .2f % .2f' % states

        quaternions = quaternion_from_euler(roll, pitch, yaw)
        position_msg.pose.orientation.x = quaternions[0]
        position_msg.pose.orientation.y = quaternions[1]
        position_msg.pose.orientation.z = quaternions[2]
        position_msg.pose.orientation.w = quaternions[3]

        self.pos_publisher.publish(position_msg)
        self.rate.sleep()

    def close(self):
        self.lab_navigation.close()
        self.logger.info('Connection Closed')
        print

if __name__ == "__main__":
    print 'Localizer process started. Press Ctrl+C to stop.'
    print '...'
    verbose = True
    parser = argparse.ArgumentParser(description="Localizes the Hexacopter using the ROS topic vision_pose/pose")
    parser.add_argument('-v', '--verbose', help='Prints many many stuff happening in the process.', action="store_true")
    args = parser.parse_args()
    if args.verbose:
        print "Verbose mode enabled."
        verbose = True

    localizer = Localizer()
    try:
        while not rospy.is_shutdown():
            localizer.updatePose(0)
        localizer.logger.info('Connection closed by rospy and user.')
        localizer.close()
    except KeyboardInterrupt:
        localizer.close()
        localizer.logger.info('Connection closed by user.')
    except Exception:
        localizer.close()
        raise
