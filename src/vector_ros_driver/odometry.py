from __future__ import division
from math import pi, sin, cos
from vector_ros_driver.pose import Pose

class Odometry:

    def __init__(self):
        self.pose = Pose()
        self.lastTime = 0
        self.leftTrackSpeed = 0
        self.rightTrackSpeed = 0

    def setTrackSeparation(self, separation):
        self.trackSeparation = separation

    def setTrackLength(self, length):
        self.trackLength = length

    def setSpeedUnitConversion(self, conv):
        self.unitConversion = conv

    def setTime(self, newTime):
        self.lastTime = newTime

    def updateLeftWheel(self, newSpeed):
        self.leftTrackSpeed = newSpeed

    def updateRightWheel(self, newSpeed):
        self.rightTrackSpeed = newSpeed

    def updatePose(self, newTime):

        leftTravel = self.leftTrackSpeed * self.trackLength * self.unitConversion
        rightTravel = self.rightTrackSpeed * self.trackLength * self.unitConversion
        deltaTime = newTime - self.lastTime

        deltaTravel = (rightTravel + leftTravel) / 2
        deltaTheta = (rightTravel - leftTravel) / self.trackSeparation

        if rightTravel == leftTravel:
            deltaX = leftTravel*cos(self.pose.theta)
            deltaY = leftTravel*sin(self.pose.theta)
        else:
            radius = deltaTravel / deltaTheta

            # Find the instantaneous center of curvature (ICC).
            iccX = self.pose.x - radius*sin(self.pose.theta)
            iccY = self.pose.y + radius*cos(self.pose.theta)

            deltaX = cos(deltaTheta)*(self.pose.x - iccX) \
                - sin(deltaTheta)*(self.pose.y - iccY) \
                + iccX - self.pose.x

            deltaY = sin(deltaTheta)*(self.pose.x - iccX) \
                + cos(deltaTheta)*(self.pose.y - iccY) \
                + iccY - self.pose.y

        self.pose.x += deltaX
        self.pose.y += deltaY
        self.pose.theta = (self.pose.theta + deltaTheta) % (2*pi)
        self.pose.xVel = deltaTravel / deltaTime if deltaTime > 0 else 0.
        self.pose.yVel = 0
        self.pose.thetaVel = deltaTheta / deltaTime if deltaTime > 0 else 0.

        self.lastTime = newTime

    def getPose(self):
        return self.pose;

    def setPose(self, newPose):
        self.pose = newPose
