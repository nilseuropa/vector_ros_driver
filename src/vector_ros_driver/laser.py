#!/usr/bin/env python3.6

import rospy
import anki_vector
import numpy

from sensor_msgs.msg import Range

class Laser(object):
    def __init__(self, async_robot, publish_rate=20):
        self.async_robot = async_robot
        self.rate = rospy.Rate(publish_rate)
        self.range_publisher = rospy.Publisher("~laser", Range, queue_size=1)
        self.publish_laser_range()

    def publish_laser_range(self):

        while not rospy.is_shutdown():
            proximity_data = self.async_robot.proximity.last_sensor_reading
            if proximity_data is not None:
                range = Range()
                range.header.frame_id = "tof_link"
                range.radiation_type = 1
                range.field_of_view = 0.2617993878
                range.min_range = 0.0
                range.max_range = 0.3
                range.header.stamp = rospy.Time.now()
                #print('Proximity distance: {0}'.format(proximity_data.distance))
                range.range = proximity_data.distance.distance_mm/1000.0
                self.range_publisher.publish(range)
                self.rate.sleep()

if __name__=="__main__":
    rospy.init_node("laser")
    async_robot = anki_vector.AsyncRobot()
    async_robot.connect()
    Imu(async_robot)
    rospy.spin()

