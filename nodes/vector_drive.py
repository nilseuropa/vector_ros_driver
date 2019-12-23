#! /usr/bin/env python
from __future__ import division

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

from vector_ros_driver import controller

class ControllerNode:

    def __init__(self):
        self.controller = controller.Controller()
        self.linearVelocity = 0.0
        self.angularVelocity = 0.0

    def main(self):
        self.leftPub = rospy.Publisher('/vector/left_wheel_desired_rate',
                                       Int32, queue_size=1)
        self.rightPub = rospy.Publisher('/vector/right_wheel_desired_rate',
                                        Int32, queue_size=1)

        rospy.init_node('diff_drive')
        self.nodeName = rospy.get_name()
        rospy.loginfo("{0} started".format(self.nodeName))

        rospy.Subscriber("cmd_vel", Twist, self.twistCallback)

        self.ticksPerMeter = 1000.0
        self.wheelSeparation = 0.055
        self.maxMotorSpeed = 3000
        self.rate = 20.0
        self.timeout = 0.2

        self.controller.setWheelSeparation(self.wheelSeparation)
        self.controller.setTicksPerMeter(self.ticksPerMeter)
        self.controller.setMaxMotorSpeed(self.maxMotorSpeed)

        rate = rospy.Rate(self.rate)
        self.lastTwistTime = rospy.get_time()
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        if rospy.get_time() - self.lastTwistTime < self.timeout:
            speeds = self.controller.getSpeeds(self.linearVelocity,
                                               self.angularVelocity)
            self.leftPub.publish(int(speeds.left))
            self.rightPub.publish(int(speeds.right))
        else:
            self.leftPub.publish(0)
            self.rightPub.publish(0)

    def twistCallback(self, twist):
        self.linearVelocity = twist.linear.x
        self.angularVelocity = twist.angular.z
        self.lastTwistTime = rospy.get_time()

if __name__ == '__main__':
    try:
        node = ControllerNode()
        node.main()
    except rospy.ROSInterruptException:
        pass