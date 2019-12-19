import cv2
import numpy as np


def yellow_detect(capture):
    ret, frame = capture.read()
    lower_yellow = np.array([22, 50, 30])
    upper_yellow = np.array([50, 255, 255])

    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(gray, lower_yellow, upper_yellow)
        roi = mask[360:380, :]
        cv2.imshow('yee1', mask)
        # cv2.imshow('yeeha', roi)
        cv2.waitKey(1)


def blue_detect(capture):
    ret, frame = capture.read()
    lower_blue = np.array([100, 70, 55])
    upper_blue = np.array([216, 255, 255])

    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(gray, lower_blue, upper_blue)
        cv2.imshow('yee2', mask)
        cv2.waitKey(1)


if __name__ == '__main__':
    capture = cv2.VideoCapture(0)
    while True:
        yellow_detect(capture)
        # blue_detect(capture)
        if cv2.waitKey(1) > 0:
            break

    capture.release()
    cv2.destroyAllWindows()

