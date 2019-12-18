#!/usr/bin/env python

import rospy, time

from detector import Detector
from motordriver import MotorDriver


class AutoDrive:
    def __init__(self):
        rospy.init_node('xycar_driver')
        self.detector = Detector('/usb_cam/image_raw')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.cnt = 0
        self.hipass = True
        self.tollgateMode = False
        self.tollbar = False

    def trace(self):
        line_l, line_r = self.detector.detect_lines()[0], self.detector.detect_lines()[1]
        angle, speed = self.steer(line_l, line_r)
        if self.detector.sign_detect():
            self.tollgateMode = True

        self.tollbar = self.detector.tollbar_detect()

        if self.tollgateMode:
            if self.tollbar:
                self.driver.drive(angle + 90, 90)
            else:
                self.driver.drive(angle + 90, 120)  # speed + 90 - 60
        else:
            self.driver.drive(angle + 90, speed + 90)

        print(self.tollgateMode)
        self.cnt += 1
        if self.cnt>30:
            for i in range(5):
                self.driver.drive(90, 130)
            exit()

    def steer(self, right, left):
        maxSpeed = 60
        minSpeed = 40
        maxAngle = 80
        mid = (left + right) // 2
        if self.tollgateMode & self.hipass:
            mid -= 90
        elif self.tollgateMode is True and self.hipass is False:
            mid += 90

        if mid < 150:
            angle = -(max((150 - mid), 1) * maxAngle / 150)
            speed = max((mid * maxSpeed / 150), minSpeed)
        elif mid > 150:
            angle = (max((mid - 150), 1) * maxAngle / 150)
            speed = max(((300 - mid) * maxSpeed / 150), minSpeed)
        else:
            angle = 0
            speed = maxSpeed

        return angle, speed

    def exit(self):
        print('finished')


if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(15)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)
