#!/usr/bin/env python3.6

import rospy
import anki_vector
import numpy

from geometry_msgs.msg import Vector3

class Imu(object):
    def __init__(self, async_robot, publish_rate=100):
        self.async_robot = async_robot
        self.rate = rospy.Rate(publish_rate)
        self.gyro_publisher = rospy.Publisher("~gyro", Vector3, queue_size=1)
        self.accelero_publisher = rospy.Publisher("~accelero", Vector3, queue_size=1)
        self.publish_inertial_vectors()

    def publish_inertial_vectors(self):

        while not rospy.is_shutdown():
            gyroscope = Vector3(self.async_robot.gyro.x,self.async_robot.gyro.y,self.async_robot.gyro.z)
            accelerometer = Vector3(self.async_robot.accel.x,self.async_robot.accel.y,self.async_robot.accel.z)
            self.gyro_publisher.publish(gyroscope)
            self.accelero_publisher.publish(accelerometer)

            # make sure to publish at required rate
            self.rate.sleep()

if __name__=="__main__":
    rospy.init_node("imu")
    async_robot = anki_vector.AsyncRobot()
    async_robot.connect()
    Imu(async_robot)
    rospy.spin()
