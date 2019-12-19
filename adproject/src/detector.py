import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Detector:
    def __init__(self, topic):
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)
        self.pts1 = np.float32([[280 - 180, 280], [0, 320], [360 + 180, 280], [640, 320]])
        self.pts2 = np.float32([[0, 0], [0, 700], [300, 0], [300, 700]])
        self.lestLine = [30, 270]

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def detect_lines(self):
        frame = self.cam_img

        pts1 = np.float32([[70, 280], [50, 300], [540, 280], [560, 300]])
        pts2 = np.float32([[0, 0], [0, 500], [300, 0], [300, 500]])

        M = cv2.getPerspectiveTransform(pts1, pts2)
        dst = cv2.warpPerspective(frame, M, (300, 500))

        edge = cv2.Canny(dst, 100, 200)

        lines = cv2.HoughLines(edge, 1, np.pi / 180, 150)

        lline = [0]
        rline = [300]
        angle = []
        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                ang = theta * (180 / 3.141592) + 90
                ang %= 180

                if abs(self.lestLine[0] - x0) < 40:
                    lline.append(x0)
                    cv2.line(dst, (x1, y1), (x2, y2), (0, 0, 255), 1)
                    angle.append(ang)
                    continue

                if abs(self.lestLine[1] - x0) < 40:
                    rline.append(x0)
                    cv2.line(dst, (x1, y1), (x2, y2), (0, 0, 255), 1)
                    angle.append(ang)
                    continue

        a = [min(rline) - 240 if max(lline) == 0 else max(lline),
             max(lline) + 240 if min(rline) == 300 else min(rline)]

        if a == [60, 240]:
            a = self.lestLine

        self.lestLine = a

        cv2.waitKey(1)
        return self.lestLine

    def tollbar_detect(self):
        frame = self.cam_img
        pixel_cnt_threshold = 0.33 * 200 * 120

        lower_yellow = np.array([22, 50, 30])
        upper_yellow = np.array([50, 250, 250])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(gray, lower_yellow, upper_yellow)
        roi = mask[180:300, 220:420]

        # cv2.imshow('yellow_all', mask)
        # cv2.imshow('yellow', roi)

        if cv2.countNonZero(roi) > pixel_cnt_threshold:
            return True

        else:
            return False

    def sign_detect(self):
        frame = self.cam_img
        pixel_cnt_threshold = 0.3 * 100 * 110
        lower_blue = np.array([100, 70, 55])
        upper_blue = np.array([216, 255, 255])
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(gray, lower_blue, upper_blue)
        roi = mask[70:180, 400:500]

        # cv2.imshow('blue_all', mask)
        # cv2.imshow('blue', roi)

        if cv2.countNonZero(roi) > pixel_cnt_threshold:
            return True

        else:
            return False
