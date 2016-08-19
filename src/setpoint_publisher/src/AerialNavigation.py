#!/usr/bin/env python

import rospy
import threading
from time import time, sleep
import mavros
import sys

from math import *
from mavros.utils import *
from mavros import setpoint
from transformations import quaternion_from_euler
from communication import LabNavigation

nan_value = float('nan')


def getError(x, y=[0, 0, 0], bound=None):
    distance = sqrt(sum([(xi - yi)**2 for xi, yi in zip(x, y)]))
    if bound == None:
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
        self.localizer = LabNavigation()
        self.setpoint_x = 0.0
        self.setpoint_y = 0.0
        self.setpoint_z = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.minimum_altitude = []

        # publisher for mavros/setpoint_position/local
        self.position_setpoint = setpoint.get_pub_position_local(queue_size=1)
        # publisher for mavros/setpoint_velocity/cmd_vel
        self.velocity_setpoint = setpoint.get_pub_velocity_cmd_vel(queue_size=1)
        # subscriber for mavros/local_position/local
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                                    setpoint.PoseStamped, self.reached, queue_size=1)

        self.my_thread = threading.Thread(target=self.navigate)
        self.my_thread.setDaemon(True)
        self.my_thread.start()

        self.done = False
        self.done_evt = threading.Event()

    def navigate(self):
        rate = rospy.Rate(5)   # in Hz

        pos_message = setpoint.PoseStamped(
            header=setpoint.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )

        print "Navigation is starting."
        self.yaw_degrees = 0  # North
        while not rospy.is_shutdown():
            if self.nav_mod != "POSITION":
                continue
            pos_message.pose.position.x = self.setpoint_x
            pos_message.pose.position.y = self.setpoint_y
            pos_message.pose.position.z = self.setpoint_z

            # For demo purposes we will lock yaw/heading to north.
            yaw = radians(self.yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw)
            pos_message.pose.orientation = setpoint.Quaternion(*quaternion)

            try:
                self.position_setpoint.publish(pos_message)
                rate.sleep()
            except rospy.ROSException:
                print 'Process terminated. Stoping Navigation!'

        print 'Navigation Exited!!!!'

    def targetVelocity(self, x, y, z, yaw, operation_condition_function):
        rate = rospy.Rate(5)
        velocity_message_zero = setpoint.TwistStamped(
            header=setpoint.Header(
                frame_id="some_frame_id",
                stamp=rospy.Rime.now()
            )
        )
        velocity_message = velocity_message_zero
        velocity_message.twist.linear.x = x
        velocity_message.twist.linear.y = y
        velocity_message.twist.linear.z = z
        if yaw is not None:
            velocity_message.twist.angular.z = yaw
        try:
            self.nav_mod = "VELOCITY"
            while operation_condition_function():
                self.velocity_setpoint.publish(velocity_message)
                rate.sleep()
            self.velocity_setpoint.publish(velocity_message_zero)
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
        if isNear(self.pos_x, self.setpoint_x) and \
           isNear(self.pos_y, self.setpoint_y) and \
           isNear(self.pos_z, self.setpoint_z):
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

    def hoverAboveDynamic(self, object_id, distance, duration):
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
        mission_healthy = True
        landed = False
        object_position = self.localizer.getStates(object_id)[1:3]
        landing_station = IsMoving(object_position)
        sleep(1)
        while True and not landed:
            object_position = self.localizer.getStates(object_id)[1:3]
            if landing_station.moved(object_position) and not landed:
                mission_healthy = False
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

    def verticalVelocity(self, velocity):
        pass


def setpoint_demo(navigation, sleep_time):
    sleep(2)
    start_x, start_y, start_z = hexacopter_navigation.getPosition()
    print 'Started from', start_x, start_y, start_z
    navigation.changeAltitude(.4)
    sleep(sleep_time)
    for i in range(1):
        print "Attempt", i

        navigation.horizontalTarget(0, 0)
        sleep(sleep_time)
        # navigation.horizontalTarget(1, 1)
        # sleep(sleep_time)
        # navigation.hoverAboveDynamic(2, .8, 10)
        # sleep(sleep_time)
        # navigation.hoverAboveDynamic(2, .6, 10)
        # sleep(sleep_time)
        # navigation.hoverAboveDynamic(2, .6, 10)
        # sleep(sleep_time)

        # navigation.hoverAboveDynamic(2, .8, 5)
        # sleep(sleep_time)
        # navigation.horizontalTarget(0, 0)
        # sleep(sleep_time)
        # navigation.goToAltitude(start_z + .1)
        navigation.exactLand(0, 0)

        # navigation.horizontalTarget(start_x + .3, start_y + .3)
        # sleep(sleep_time)
        # navigation.horizontalTarget(start_x, start_y)
        # sleep(sleep_time)
        # navigation.targetPoint(start_x, start_y, start_z + .5, 1, True)
        # navigation.targetPoint(start_x, start_y, start_z + .2)
        # navigation.hoverAbove(1, .3)
        # sleep(sleep_time)
        # navigation.goToAltitude(.3)
        # sleep(sleep_time)
        # navigation.changeAltitude(.2)
        # sleep(sleep_time)
        # navigation.horizontalTarget(start_x, start_y)
        # sleep(sleep_time)
        # navigation.inPlaceLand(False)
        # navigation.changeAltitude(-.1)
        # navigation.changeAltitude(-.1)
    navigation.inPlaceLand()


if __name__ == '__main__':
    sleep_time = 1
    while sleep_time > 0:
        print "Mission starts in:", sleep_time
        sleep_time -= 1
        sleep(1)

    rospy.init_node('setpoint_position_demo')
    mavros.set_namespace()  # initialize mavros module with default namespace
    hexacopter_navigation = AerialNavigation()
    try:
        setpoint_demo(hexacopter_navigation, 3)
    except Exception, e:
        print 'EXCEPTION', e
        hexacopter_navigation.inPlaceLand(False)
        sleep(10)
        raise e
