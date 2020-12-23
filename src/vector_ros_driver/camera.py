#!/usr/bin/env python3

import rospy
import anki_vector
import numpy
import cv_bridge
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Camera(object):
    def __init__(self, async_robot, publish_rate=30):
        self.async_robot = async_robot
        self.async_robot.camera.init_camera_feed()
        self.rate = rospy.Rate(publish_rate)
        self.image_publisher = rospy.Publisher("~camera", Image, queue_size=1)
        self.bridge = CvBridge()
        self.publish_camera_feed()

    def publish_camera_feed(self):
        bridge = cv_bridge.CvBridge()

        while not rospy.is_shutdown():

            pil_image = self.async_robot.camera.latest_image.raw_image.convert('RGB')
            cv_image = numpy.array(pil_image)
            cv_image = cv_image[:, :, ::-1].copy()
            try:
                self.image_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)

            self.rate.sleep()

if __name__=="__main__":
    rospy.init_node("camera")
    async_robot = anki_vector.AsyncRobot()
    async_robot.connect()
    async_robot.camera.init_camera_feed()
    Camera(async_robot)
    rospy.spin()
