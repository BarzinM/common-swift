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


import rospy
import threading
from time import time, sleep
# import mavros
import sys
from control import PID

from math import *  # TODO: clarify
# from mavros.utils import *  # TODO: clarify
# from mavros import setpoint
from transformations import quaternion_from_euler
from communication import LabLocalization
from sys import path
path.append(path[0] + '/../../Modules/dev_box/')
print path[-1]
import dev_and_text_tools as dvt

# dvt.bigAlert('Here')

nan_value = float('nan')

loger = dvt.setupLogger(True,'Aerial_nav',True)
loger.debug('start of the process')

def getError(x, y=[0, 0, 0], bound=None):
    distance = sqrt(x[0] - y[0])**2 + (x[1] - y[1])**2 + (x[2] - y[2])**2
    if bound is None:
        return distance
    return distance < bound


def isPositionNear(x, y, bound=.1):
    return all([abs(x[i] - y[i]) < bound for i in range(len(x))])


class IsMoving:
    def __init__(self, pos=nan_value):
        self.fromHere(pos)

    def fromHere(self, pos):
        self.pos = pos

    def moved(self, pos, bound=.05):
        is_in_place = isPositionNear(self.pos, pos, bound)
        has_moved = not is_in_place
        if has_moved:
            print 'Object moved from', self.pos, 'to', pos
            self.fromHere(pos)
        return has_moved


def isNear(x, y, bound=.05):
    return abs(x - y) < bound


class AerialNavigation:
    """
    This class sends position targets to FCU's position controller
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
        self.horizontal_pid = 0, 1., 0
        self.vertical_pid = 0, 0., 0
        self.nav_mod = "POSITION"

        # publisher for mavros/setpoint_position/local
        self.setpoint_position = setpoint.get_pub_position_local(queue_size=1)
        # publisher for mavros/setpoint_velocity/cmd_vel
        self.setpoint_velocity = setpoint.get_pub_velocity_cmd_vel(queue_size=1)
        # subscriber for mavros/local_position/local
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                                    setpoint.PoseStamped, self.reached, queue_size=1)

        self.my_thread = threading.Thread(target=self.navigate)
        self.my_thread.setDaemon(True)
        self.my_thread.start()

        self.done = False
        self.done_evt = threading.Event()

    def getStates(self):
        return (self.pos_x, self.pos_y, self.pos_z, self.yaw_degrees)

    def navigate(self):
        rate = rospy.Rate(8)   # in Hz
        rate_mode_check = rospy.Rate(20)

        message_pos = setpoint.PoseStamped(
            header=setpoint.Header(
                frame_id="base_footprint",  # no matter, plugin doesn't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )

        print "Navigation is starting."
        self.yaw_degrees = 0  # Initialize to North, TODO: remove/modify
        while not rospy.is_shutdown():
            if self.nav_mod != "POSITION":
                rate_mode_check.sleep()
                continue
            message_pos.pose.position.x = self.setpoint_x
            message_pos.pose.position.y = self.setpoint_y
            message_pos.pose.position.z = self.setpoint_z

            yaw = radians(self.yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw)
            message_pos.pose.orientation = setpoint.Quaternion(*quaternion)

            try:
                # self.setpoint_position.publish(message_pos)
                rate.sleep()
            except rospy.ROSException:
                print 'Process terminated. Stoping Navigation!'

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

    def conditionedVelocity(self, x, y, z, yaw, operation_condition_function):
        rate = rospy.Rate(5)
        message_velocity_zero = setpoint.TwistStamped(
            header=setpoint.Header(
                frame_id="some_frame_id",
                stamp=rospy.Time.now()
            )
        )
        message_velocity = message_velocity_zero
        message_velocity.twist.linear.x = x
        message_velocity.twist.linear.y = y
        message_velocity.twist.linear.z = z
        if yaw is not None:
            message_velocity.twist.angular.z = yaw
        try:
            self.nav_mod = "VELOCITY"
            while not rospy.is_shutdown() and operation_condition_function():
                # message_velocity.header.stamp = rospy.Time.now()
                self.setpoint_velocity.publish(message_velocity)
                rate.sleep()
            # message_velocity_zero.header.stamp = rospy.Time.now()
            self.setpoint_velocity.publish(message_velocity_zero)
            self.targetPoint(self.pos_x, self.pos_y, self.pos_z)
        except rospy.ROSException:
            print 'Process terminated. Stoping Navigation!'

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
                print 'Waiting for %.2f seconds' % delay
            sleep(delay)

        if verbose:
            print 'Done: %.2f, %.2f, %.2f' % (self.pos_x, self.pos_y, self.pos_z)

    def reached(self, topic):
        self.pos_x = topic.pose.position.x
        self.pos_y = topic.pose.position.y
        self.pos_z = topic.pose.position.z
        if getError(self.getPosition,[self.setpoint_x,self.setpoint_y,self.setpoint_z],bound=.1):
            self.done = True
            self.done_evt.set()

    def getPosition(self):
        return self.pos_x, self.pos_y, self.pos_z

    def changeAltitude(self, altitude):
        self.targetPoint(self.pos_x, self.pos_y, self.pos_z + altitude, 1, True)

    def inPlaceLand(self, exit=True):
        self.minimum_altitude = self.pos_z
        z = self.pos_z
        print 'Landing ...',
        while True:
            z -= .03
            self.targetPoint(self.pos_x, self.pos_y, z, .1, False, False)
            if self.__isLanded():
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
            if self.__isLanded():
                break
        print 'Landed'
        if exit:
            sys.exit(0)

    def __isLanded(self, duration=2.0):
        time_landed = time()
        target_height = self.setpoint_z
        while time() - time_landed < duration:
            current_height = self.pos_z
            if current_height < self.minimum_altitude:
                self.minimum_altitude = current_height
                return False

        if target_height < self.minimum_altitude:
            return True
        return False

    def hoverAbove(self, object_id, distance):
        states = self.localizer.getStates(object_id)
        x = states[1]
        y = states[2]
        z = states[3] + distance
        if x != x:
            print "ERROR: Received NAN for location of body #", object_id
            return
        print 'Target destination:', x, y, z
        self.targetPoint(self.pos_x, self.pos_y, z, 3, True)
        print 'go above it'
        self.targetPoint(x, y, z, 5, True)

    def dynamicHover(self, object_id, distance, duration):
        z = self.localizer.getStates(object_id)[3] + distance
        if z == z:
            self.targetPoint(self.pos_x, self.pos_y, z, 1, True)
        else:
            return

        start_time = time()
        while time() - start_time < duration:
            states = self.localizer.getStates(object_id)
            x = states[1]
            y = states[2]
            z = states[3] + distance
            self.yaw_degrees = states[6] * 180 / 3.14
            if x != x:
                print "ERROR: Received NAN for location of body #", object_id
                return
            self.targetPoint(x, y, z, 0, False)

    def horizontalTarget(self, x, y):
        self.targetPoint(x, y, self.pos_z, 1, True)

    def goToAltitude(self, z):
        self.targetPoint(self.pos_x, self.pos_y, z, 1, True)

    def objectLand(self, object_id):
        # get object location
        # mission_healthy = True
        landed = False
        object_position = self.localizer.getStates(object_id)[1:3]
        landing_station = IsMoving(object_position)
        sleep(1)
        while True and not landed:
            object_position = self.localizer.getStates(object_id)[1:3]
            if landing_station.moved(object_position) and not landed:
                # mission_healthy = False
                print 'Aborting landing mission'
                break
            self.minimum_altitude = self.pos_z
            z = self.pos_z
            print 'Landing ...',

            z -= .03
            self.targetPoint(object_position[0], object_position[1], z, .1, False, False)
            if self.__isLanded():
                landed = True
                break
                print 'Landed'

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

        horizontal_p, horizontal_i, horizontal_d = self.horizontal_pid
        vertical_p, vertical_i, vertical_d = self.vertical_pid
        integrated_error_x = 0
        integrated_error_y = 0
        integrated_error_z = 0
        previous_x, previous_y, previous_z = self.getStates()[:3]
        time_old = time()
        time_start = time()
        self.nav_mod = "VELOCITY"
        while time() - time_start < duration and not rospy.is_shutdown():
            states = self.getStates()
            time_now = time()
            time_difference = time_now - time_old
            time_old = time_now
            error_x = states[0] - point[0]
            integrated_error_x += error_x * time_difference
            error_y = states[1] - point[1]
            integrated_error_y += error_y * time_difference
            error_z = states[2] - point[2]
            integrated_error_z += error_z * time_difference

            velocity_action_x = horizontal_p * error_x +\
                horizontal_i * integrated_error_x +\
                horizontal_d * (states[0] - previous_x) / time_difference

            velocity_action_y = horizontal_p * error_y +\
                horizontal_i * integrated_error_y +\
                horizontal_d * (states[0] - previous_y) / time_difference

            velocity_action_z = vertical_p * error_z +\
                vertical_i * integrated_error_z +\
                vertical_d * (states[0] - previous_z) / time_difference

            print "errors %+0.2f, %+0.2f, %+0.2f" % (error_x, error_y, error_z),
            print "action %+0.2f, %+0.2f, %+0.2f" % (velocity_action_x, velocity_action_y, velocity_action_z)

            message_velocity.twist.linear.x = velocity_action_x
            message_velocity.twist.linear.y = velocity_action_y
            message_velocity.twist.linear.z = velocity_action_z
            message_velocity.header.stamp = rospy.Time.now()
            self.setpoint_velocity.publish(message_velocity)

            x_previous, y_previous, z_previous = states[:3]
            rate.sleep()


def setpoint_demo(navigation, ):
    # time_sleep = 3
    sleep(2)
    start_x, start_y, start_z = navigation.getPosition()
    print 'Started from', start_x, start_y, start_z
    # navigation.changeAltitude(.4)
    # sleep(time_sleep)
    for i in range(1):
        print "Attempt", i

        # navigation.horizontalTarget(0, 0)
        # sleep(time_sleep)
        cond_pos = navigation.getStates()

        def cond():
            current_pos = navigation.getStates()
            return getError(current_pos, cond_pos, .2)

        print('Condition test:', cond())

        navigation.stabilize(60., [0., .1, 0.])

    navigation.inPlaceLand()


if __name__ == '__main__':
    sleep_time = 3
    while sleep_time > 0:
        print "Mission starts in:", sleep_time
        sleep_time -= 1
        sleep(1)

    rospy.init_node('setpoint_position_demo')
    mavros.set_namespace()  # initialize mavros module with default namespace
    hexacopter_navigation = AerialNavigation()
    try:
        setpoint_demo(hexacopter_navigation)
    except Exception, e:
        print 'EXCEPTION', e
        hexacopter_navigation.inPlaceLand(False)
        sleep(10)
        raise
        raise e
