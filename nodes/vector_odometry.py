#! /usr/bin/env python3
from __future__ import division

import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import sin, cos
from vector_ros_driver.pose import Pose
from vector_ros_driver import odometry

class OdometryNode:

    def __init__(self):
        self.odometry = odometry.Odometry()

    def main(self):
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tfPub = TransformBroadcaster()

        rospy.init_node('odometry')
        self.nodeName = rospy.get_name()
        rospy.loginfo("{0} started".format(self.nodeName))

        rospy.Subscriber("/vector/left_wheel_speed", Int32, self.leftCallback)
        rospy.Subscriber("/vector/right_wheel_speed", Int32, self.rightCallback)
        rospy.Subscriber("/vector/initialpose", PoseWithCovarianceStamped,
                         self.on_initial_pose)

        self.baseFrameID = rospy.get_param('~base_frame_id', 'base_footprint')
        self.odomFrameID = rospy.get_param('~odom_frame_id', 'odom')
        self.publishTF = rospy.get_param('~publish_tf', True) # default False !

        self.odometry.setTrackSeparation(0.047)
        self.odometry.setTrackLength(0.055)
        self.odometry.setSpeedUnitConversion(0.001) # mm / sec -> m / sec
        self.odometry.setTime(rospy.get_time())

        self.rate = 10.0
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def publish(self):
        self.odometry.updatePose(rospy.get_time())
        now = rospy.get_rostime()
        pose = self.odometry.getPose()
        q = quaternion_from_euler(0, 0, pose.theta)

        if self.publishTF:
            self.tfPub.sendTransform(
                (pose.x, pose.y, 0),
                (q[0], q[1], q[2], q[3]),
                now,
                self.baseFrameID,
                self.odomFrameID
            )

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odomFrameID
        odom.child_frame_id = self.baseFrameID
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = pose.xVel
        odom.twist.twist.angular.z = pose.thetaVel
        self.odomPub.publish(odom)

    def on_initial_pose(self, msg):
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        pose = Pose()
        pose.x = msg.pose.pose.position.x
        pose.y = msg.pose.pose.position.y
        pose.theta = yaw

        rospy.loginfo('Setting initial pose to %s', pose)
        self.odometry.setPose(pose)

    def leftCallback(self, msg):
        self.odometry.updateLeftWheel(msg.data)

    def rightCallback(self, msg):
        self.odometry.updateRightWheel(msg.data)


if __name__ == '__main__':
    try:
        node = OdometryNode()
        node.main()
    except rospy.ROSInterruptException:
        pass
