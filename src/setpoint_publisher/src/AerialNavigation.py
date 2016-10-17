#!/usr/bin/env python

# __          __        _
# \ \        / /       | |
#  \ \  /\  / /__  _ __| | __
#   \ \/  \/ / _ \| '__| |/ /
#    \  /\  / (_) | |  |   <
#  ___\/  \/ \___/|_|  |_|\_\
# |_   _|
#   | |  _ __
#   | | | '_ \
#  _| |_| | | |
# |_____|_| |_|
# |  __ \
# | |__) | __ ___   __ _ _ __ ___  ___ ___
# |  ___/ '__/ _ \ / _` | '__/ _ \/ __/ __|
# | |   | | | (_) | (_| | | |  __/\__ \__ \
# |_|   |_|  \___/ \__, |_|  \___||___/___/
#                   __/ |
#                  |___/


import threading
from time import time, sleep
import sys
import math  # TODO: clarify
from sys import path

import rospy
import mavros
from mavros.utils import *  # TODO: clarify
from mavros import setpoint
from control import PID

from transformations import quaternion_from_euler
from communication import LabLocalization
path.append(path[0] + '/../../Modules/dev_box/')
import dev_and_text_tools as dvt

# dvt.bigAlert('Here')
# loger = dvt.setupLogger(True, 'Aerial_nav', True)
# loger.debug('start of the process')

nan_value = float('nan')


def isNan(x):
    return x!=x

def getError(x, y=[0, 0, 0], bound=None):
    distance = math.sqrt((x[0] - y[0])**2 + (x[1] - y[1])**2 + (x[2] - y[2])**2)
    if bound is None:
        return distance
    return distance < bound


class IsMoving:
    def __init__(self, pos=nan_value):
        self.fromHere(pos)

    def fromHere(self, pos):
        self.pos = pos

    def hasMoved(self, pos, bound=.05):
        is_in_place = getError(self.pos, pos, bound)
        has_moved = not is_in_place
        if has_moved:
            print 'Object moved from', self.pos, 'to', pos
            self.fromHere(pos)
        return has_moved


class AerialNavigation:
    """
    This class position or velocity sends setpoint targets to FCU's controller.
    """

    def __init__(self):
        self.localizer = LabLocalization()
        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        self.setpoint_z = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.yaw_degrees = 0.
        self.minimum_altitude = []
        self.horizontal_pid = .3, .0001, 0
        self.velocity_max_horizontal = None
        self.vertical_pid = 1, 0, 0
        self.veloctiy_max_vertical = .05
        self.nav_mod = "POSITION"

        # publisher for mavros/setpoint_position/local
        self.setpoint_position = setpoint.get_pub_position_local(queue_size=1)
        # publisher for mavros/setpoint_velocity/cmd_vel
        self.setpoint_velocity = setpoint.get_pub_velocity_cmd_vel(queue_size=1)
        # subscriber for mavros/local_position/local
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                                    setpoint.PoseStamped, self.__hasReached, queue_size=1)

        self.my_thread = threading.Thread(target=self.positionBeat)
        self.my_thread.setDaemon(True)
        self.my_thread.start()

        self.done = False
        self.done_evt = threading.Event()

    def getStates(self):
        return (self.pos_x, self.pos_y, self.pos_z, self.yaw_degrees)

    def positionBeat(self):
        rate = rospy.Rate(8)   # in Hz
        rate_mode_check = rospy.Rate(20)

        message_pos = setpoint.PoseStamped(
            header=setpoint.Header(
                frame_id="base_footprint",  # no matter, plugin doesn't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )

        print "Navigation is starting ..."
        while not rospy.is_shutdown():
            if self.nav_mod != "POSITION":
                rate_mode_check.sleep()
                continue
            message_pos.pose.position.x = self.setpoint_x
            message_pos.pose.position.y = self.setpoint_y
            message_pos.pose.position.z = self.setpoint_z

            yaw = math.radians(self.yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw)
            message_pos.pose.orientation = setpoint.Quaternion(*quaternion)

            self.setpoint_position.publish(message_pos)
            rate.sleep()

        else:
            print 'Process terminated. Stopping navigation ...'

        print 'Navigation Exited!!!!'

    def __assignVelocity(self, x, y, z, yaw):
        message_velocity = setpoint.TwistStamped(
            header=setpoint.Header(
                frame_id="some_frame_id",
                stamp=rospy.Time.now()
            )
        )
        message_velocity.twist.linear.x = x
        message_velocity.twist.linear.y = y
        message_velocity.twist.linear.z = z
        message_velocity.twist.angular.z = yaw
        try:
            self.setpoint_velocity.publish(message_velocity)
        except rospy.ROSException:
            print 'Process terminated. Stopping Navigation!'

    def __conditionedVelocity(self, x, y, z, yaw, operation_condition_function):
        rate = rospy.Rate(5)

        message_velocity = setpoint.TwistStamped(
            header=setpoint.Header(
                frame_id="some_frame_id",
                stamp=rospy.Time.now()
            )
        )
        message_velocity.twist.linear.x = x
        message_velocity.twist.linear.y = y
        message_velocity.twist.linear.z = z
        if yaw is not None:
            message_velocity.twist.angular.z = yaw

        self.nav_mod = "VELOCITY"
        while not rospy.is_shutdown() and operation_condition_function():
            message_velocity.header.stamp = rospy.Time.now()
            self.setpoint_velocity.publish(message_velocity)
            rate.sleep()

        message_velocity_zero = setpoint.TwistStamped(
            header=setpoint.Header(
                frame_id="some_frame_id",
                stamp=rospy.Time.now()
            )
        )
        self.setpoint_velocity.publish(message_velocity_zero)
        self.targetPoint(self.pos_x, self.pos_y, self.pos_z)

    def targetPoint(self, x, y, z, delay=0, confirm=True, verbose=True):
        if verbose:
            print 'Going to %.2f, %.2f, %.2f ...' % (x, y, z)
        self.done = False
        self.setpoint_x = x
        self.setpoint_y = y
        self.setpoint_z = z
        self.nav_mod = "POSITION"

        if confirm:
            rate = rospy.Rate(2)
            while not self.done and not rospy.is_shutdown():
                rate.sleep()
        
        if rospy.is_shutdown():
            print 'Exiting the process. Rospy is down. Probably by user\'s KeyboardInterrupt.'
            sys.exit()

        if delay:
            if verbose:
                print 'Waiting for %.2f seconds ...' % delay
            sleep(delay)

        if verbose:
            print '@ %.2f, %.2f, %.2f' % (self.pos_x, self.pos_y, self.pos_z)

    def __hasReached(self, topic):
        self.pos_x = topic.pose.position.x
        self.pos_y = topic.pose.position.y
        self.pos_z = topic.pose.position.z
        if getError(self.getPosition(), [self.setpoint_x, self.setpoint_y, self.setpoint_z], bound=.1):
            self.done = True
            self.done_evt.set()

    def getPosition(self):
        return self.pos_x, self.pos_y, self.pos_z

    ##############################################
    #  __  __                                    #
    # |  \/  |__ _ _ _  ___ _  ___ _____ _ _ ___ #
    # | |\/| / _` | ' \/ -_) || \ V / -_) '_(_-< #
    # |_|  |_\__,_|_||_\___|\_,_|\_/\___|_| /__/ #    
    ##############################################

    def changeAltitude(self, altitude):
        self.targetPoint(self.pos_x, self.pos_y, self.pos_z + altitude, 1, True)

    def inPlaceLand(self, exit=True):
        self.minimum_altitude = self.pos_z
        z = self.pos_z
        print 'Landing ...',
        while True:
            z -= .03
            self.targetPoint(self.pos_x, self.pos_y, z, .1, False, False)
            if self.__hasLanded():
                break
        print 'Landed'
        if exit:
            sys.exit(0)

    def exactLand(self, x, y, exit=True):
        self.minimum_altitude = self.pos_z
        z = self.pos_z
        print 'Landing ...',
        while True:
            z -= .03
            self.targetPoint(x, y, z, .1, False, False)
            if self.__hasLanded():
                break
        print 'Landed'
        if exit:
            sys.exit(0)

    def __hasLanded(self, duration=2.0):
        # start the time
        time_landed = time()
        # save target height
        target_height = self.setpoint_z
        # while duration is acceptable
        while time() - time_landed < duration:
            # save current height
            current_height = self.pos_z
            # if it's currently lower
            if current_height < self.minimum_altitude:
                # update minimum known height which means that it can probably go even lower. So, return False
                self.minimum_altitude = current_height
                return False

        # if it seems not to be able to reach the target height then it's probably as low as it can be
        if target_height < self.minimum_altitude:
            return True

        # otherwise return False
        return False

    def hoverAbove(self, qualisys_object_id, distance):
        states = self.localizer.getStates(qualisys_object_id)
        x = states[1]
        y = states[2]
        z = states[3] + distance
        if isNan(x):
            print "ERROR: Received NAN for location of body #", qualisys_object_id
            return
        print 'Target destination:', x, y, z
        self.targetPoint(self.pos_x, self.pos_y, z, 3, True)
        print 'go above it'
        self.targetPoint(x, y, z, 5, True)

    def dynamicHover(self, qualisys_object_id, distance, duration):
        z = self.localizer.getStates(qualisys_object_id)[3] + distance
        if not isNan(z):
            self.targetPoint(self.pos_x, self.pos_y, z, 1, True)
        else:
            return

        start_time = time()
        while time() - start_time < duration:
            states = self.localizer.getStates(qualisys_object_id)
            x = states[1]
            y = states[2]
            z = states[3] + distance
            self.yaw_degrees = states[6] * 180 / 3.14
            if isNan(x):
                print "ERROR: Received NAN for location of body #", qualisys_object_id
                return
            self.targetPoint(x, y, z, 0, False)

    def horizontalTarget(self, x, y):
        self.targetPoint(x, y, self.pos_z, 1, True)

    def goToAltitude(self, z):
        self.targetPoint(self.pos_x, self.pos_y, z, 1, True)

    def objectLand(self, qualisys_object_id):
        # save initial position before landing in case needed for mission abort
        initial_position = self.getStates()[:3]

        # current position of the target object
        object_position = self.localizer.getStates(qualisys_object_id)[1:3]

        # Monitor the object to make sure it doesn't move
        landing_station = IsMoving(object_position)
        sleep(1)
        print 'Landing ...',
        
        # keeping track of minimum height during the landing operation
        self.minimum_altitude = self.pos_z

        # current target height will be decreased slowly
        z = self.pos_z

        # while the target is well defined
        while not isNan(object_position[0]):
            # make sure it hasn't moved, if it has, abort landing and go to initial position
            if landing_station.hasMoved(object_position):
                # mission_healthy = False
                print 'Target has moved. Aborting landing mission!'
                self.targetPoint(*initial_position,delay=0,confirm=False,verbose=True)
                break
            
            # decrease target height
            z -= .03
            self.targetPoint(object_position[0], object_position[1], z, .1, False, False)

            # check if landing has been detected
            if self.__hasLanded():
                # if so, finish the process
                print 'Landed'
                break

            # update the target's position
            object_position = self.localizer.getStates(qualisys_object_id)[1:3]

    # NOT TESTED, NOT SAFE, WIP
    def verticalVelocity(self, velocity, duration):
        message_velocity = setpoint.TwistStamped(
            header=setpoint.Header(
                frame_id="some_frame_id",
                stamp=rospy.Time.now()
            )
        )
        pid_x = PID(*self.horizontal_pid)
        pid_y = PID(*self.horizontal_pid)

        initial_point = self.getStates()[:4]

        def calculateAndSendOne():
            current = self.getStates()[:4]
            x_error = current[0] - initial_point[0]
            y_error = current[1] - initial_point[1]
            x_action = pid_x.action(x_error)
            y_action = pid_y.action(y_error)
            message_velocity.twist.linear.x = x_action
            message_velocity.twist.linear.y = y_action
            message_velocity.twist.linear.z = velocity
            message_velocity.twist.angular.z = 0
            self.setpoint_velocity.publish(message_velocity)

        calculateAndSendOne()
        self.nav_mod = "VELOCITY"

        rate = rospy.Rate(20)
        time_start = time()
        while not rospy.is_shutdown() and time() - time_start < duration:
            calculateAndSendOne()
            rate.sleep()

    def stabilize(self, duration, point=None):
        if point is None:
            point = self.getStates()
        else:
            if len(point) < 3:
                raise 'point needs to be a sequence with 3 or 4 elements: (x, y, z[, yaw])'
        point = point[:4]

        rate = rospy.Rate(50)
        message_velocity = setpoint.TwistStamped(
            header=setpoint.Header(
                frame_id="some_frame_id",
                stamp=rospy.Time.now()
            )
        )

        controller_x = PID(*self.horizontal_pid, windup=self.velocity_max_horizontal)
        controller_y = PID(*self.horizontal_pid, windup=self.velocity_max_horizontal)
        controller_z = PID(*self.vertical_pid, windup=self.veloctiy_max_vertical)

        self.nav_mod = "VELOCITY"

        time_start = time()
        while time() - time_start < duration and not rospy.is_shutdown():
            states = self.getStates()

            error_x = point[0] - states[0]
            error_y = point[1] - states[1]
            error_z = point[2] - states[2]

            velocity_action_x = controller_x.action(error_x)
            velocity_action_y = controller_y.action(error_y)
            velocity_action_z = controller_z.action(error_z)

            print "errors %+0.2f, %+0.2f, %+0.2f" % (error_x, error_y, error_z), "| action %+0.2f, %+0.2f, %+0.2f" % (velocity_action_x, velocity_action_y, velocity_action_z)

            message_velocity.twist.linear.x = velocity_action_x
            message_velocity.twist.linear.y = velocity_action_y
            message_velocity.twist.linear.z = velocity_action_z
            message_velocity.header.stamp = rospy.Time.now()

            self.setpoint_velocity.publish(message_velocity)

            rate.sleep()
        self.targetPoint(*self.getStates())

def setpoint_demo(navigation, ):
    sleep_time = 3
    while sleep_time > 0:
        print "Mission starts in:", sleep_time
        sleep_time -= 1
        sleep(1)
    start_x, start_y, start_z = navigation.getPosition()
    print 'Starting from', start_x, start_y, start_z

    navigation.stabilize(600., [0., 0., .22])

    navigation.inPlaceLand()


if __name__ == '__main__':

    rospy.init_node('setpoint_position_demo')
    mavros.set_namespace()  # initialize mavros module with default namespace
    vehicle = AerialNavigation()
    try:
        setpoint_demo(vehicle)
    except Exception, e:
        print 'EXCEPTION', e
        vehicle.inPlaceLand(False)
        sleep(10)
        raise
