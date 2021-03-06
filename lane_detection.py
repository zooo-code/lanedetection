#!/usr/bin/env python

import rospy
import cv2
import sys
import math
import numpy as np
from lms1xx.msg import cam

speed = 0

L_1 = [0, 120]
L_2 = [0, 340]
L_3 = [320, 340]
L_4 = [320, 120]

R_1 = [320, 120]
R_2 = [320, 450]

R_3 = [640, 450]
R_4 = [640, 120]


# roi 영역을 값을 넣은 후 진행
vertices_L = [
    (L_1[0], L_1[1]),
    (L_2[0], L_2[1]),
    (L_3[0], L_3[1]),
    (L_4[0], L_4[1])
]

vertices_R = [
    (R_1[0], R_1[1]),
    (R_2[0], R_2[1]),
    (R_3[0], R_3[1]),
    (R_4[0], R_4[1])
]
# 왼쪽차선 오른쪽 차선의 위치를 넣을 리스트
mean_data_L = []
mean_data_R = []

line_ = []
line__ = []

sequence = False

def callback_order(msg):
    global sequence
    sequence = msg.second


class Lane_Detection:
    # 왼쪽 ROI
    def region_of_interest_L(self, img):
        global vertices_L
        mask = np.zeros_like(img)

        if len(img.shape) > 2:
            channel_count = img.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        cv2.fillPoly(mask, np.array([vertices_L]), ignore_mask_color)

        ROI_image = cv2.bitwise_and(img, mask)

        return ROI_image
    # 오른쪽 ROI
    def region_of_interest_R(self, img):
        global vertices_R
        mask = np.zeros_like(img)

        if len(img.shape) > 2:
            channel_count = img.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        cv2.fillPoly(mask, np.array([vertices_R]), ignore_mask_color)

        ROI_image = cv2.bitwise_and(img, mask)

        return ROI_image

    # 이미지 전처리
    def img_process(self, img):
        # img_size = cv2.resize(img, dsize=(705, 270), interpolation=cv2.INTER_AREA)

        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_Blur = cv2.medianBlur(img_gray, 7)
        img_canny = cv2.Canny(img_Blur, 80, 180, None, 3)

        return img_canny, img
    # 왼쪽 차선 검출
    def lane_detection_L(self, img, img_size):
        global mean_data_L, L_1, L_2, L_3, L_4, vertices_L

        roi = self.region_of_interest_L(img)
        #cv2.imshow('roi_L', roi)
        linesP = cv2.HoughLinesP(roi, 1, np.pi / 180, 50, None, 10, 10)
        # print(linesP)
        if linesP is not None:
            data = []
            mean_data_L = []
            # 검출된 직선중 일정 조건을 만족하지 못한다면 버린다.
            for i in range(0, len(linesP)):
                l = linesP[i][0]

                if ((float(l[0]) - float(l[2])) != 0):
                    slope = (float(l[1]) - float(l[3])) / (float(l[0]) - float(l[2]))
                    theta__ = (math.atan(slope) * 180 / 3.14)

                    if ((theta__ > -50 and theta__ < 0) or (theta__ > 0 and theta__ < 50)):
                        if (l[0] < 2000):
                            # print(i, "l_l", l)
                            if len(data) < 3:
                                data.append(l)
                            cv2.line(img_size, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)

            # 검출된 직선의 평균을 구해 그 직선을 차선이라 인식한다.
            if len(data) <= 5 and len(data) != 0:
                mean_data_L = np.mean(data, axis=0, dtype=int)
                # print(mean)
                cv2.line(img_size, (mean_data_L[0], mean_data_L[1]), (mean_data_L[2], mean_data_L[3]), (255, 0, 0), 3,
                         cv2.LINE_AA)
            # 만약에 위의 조건을 만족하는 직선이 없을 때 roi를 다시 재조정한다.
            else:
                vertices_L = [
                    (L_1[0], L_1[1]),
                    (L_2[0], L_2[1]),
                    (L_3[0], L_3[1]),
                    (L_4[0], L_4[1])
                ]
        # 이경우에도 roi 재조정
        else:
            vertices_L = [
                (L_1[0], L_1[1]),
                (L_2[0], L_2[1]),
                (L_3[0], L_3[1]),
                (L_4[0], L_4[1])
            ]
            # 그리고 빈배열을 만든다.
            mean_data_L = []
        return img_size, mean_data_L

    def lane_detection_R(self, img, img_size):
        global mean_data_R, R_1, R_2, R_3, R_4, vertices_R
        roi = self.region_of_interest_R(img)
        cv2.imshow('roi_R', roi)
        linesP = cv2.HoughLinesP(roi, 1, np.pi / 180, 50, None, 10, 10)
        if linesP is not None:
            #print("lol")
            data = []
            mean_data_R = []
            for i in range(0, len(linesP)):
                #print("lol")
                l = linesP[i][0]
                # print("l_R", l)
                if ((l[0] - l[2]) != 0):
                    slope = (float(l[1]) - float(l[3])) / (float(l[0]) - float(l[2]))
                    theta__ = (math.atan(slope) * 180 / 3.14)
                    # print(theta__)
                    #print("lol")

                    if ((theta__ > -50 and theta__ < 0) or (theta__ > 0 and theta__ < 50)):
                        if (l[0] < 2000):
                            # print(i, "l_l", l)
                            if len(data) < 3:
                                data.append(l)
                            cv2.line(img_size, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)

            if len(data) <= 5 and len(data) != 0:
                mean_data_R = np.mean(data, axis=0, dtype=int)
                # print(mean)
                cv2.line(img_size, (mean_data_R[0], mean_data_R[1]), (mean_data_R[2], mean_data_R[3]), (255, 0, 0), 3,cv2.LINE_AA)
                # 이부분의 경우  ROI를 인식된 차선의 중심으로 일정범위 증가시켜 인식한다.
                """                
                vertices_R = [
                    (mean_data_R[0] + 30, mean_data_R[1] + 30),
                    (mean_data_R[0] - 30, mean_data_R[1] - 30),
                    (mean_data_R[2] - 30, mean_data_R[3] - 30),
                    (mean_data_R[2] + 30, mean_data_R[3] + 30)
                ]"""

            else:
                vertices_R = [
                    (R_1[0], R_1[1]),
                    (R_2[0], R_2[1]),
                    (R_3[0], R_3[1]),
                    (R_4[0], R_4[1])
                ]
        else:
            vertices_R = [
                (R_1[0], R_1[1]),
                (R_2[0], R_2[1]),
                (R_3[0], R_3[1]),
                (R_4[0], R_4[1])
            ]
            mean_data_R = []
         
        return img_size, mean_data_R
# 정지선
class Stop_Lane:
    def region_of_interest(self, img, color3=(255, 255, 255), color1=255):
        global roi_y_max, roi_y_min
        vertices = np.array([[(0, roi_y_max), (0, roi_y_min), (480, roi_y_min), (480, roi_y_max)]])

        mask = np.zeros_like(img)

        if len(img.shape) > 2:
            channel_count = img.shape[2]
            ignore_mask_color = (255,) * channel_count
        else:
            ignore_mask_color = 255

        cv2.fillPoly(mask, vertices, ignore_mask_color)

        ROI_image = cv2.bitwise_and(img, mask)
        return ROI_image

    def image_process(self, img):



        cgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        frame = cv2.medianBlur(cgray, 5)

        canny = cv2.Canny(frame, 50, 200)

        return canny

    def houghP(self, img):
        global stop_y_data, roi_y_max, roi_y_min
        lines = cv2.HoughLinesP(img, 0.8, np.pi / 180, 90, minLineLength=150, maxLineGap=40)
        # print('t',lines)
        if lines is not None:
            global line_
            global line__
            del line_[:]
            del line__[:]

            for i in lines:
                a = abs(i[0][1] - i[0][3])
                if (a < 25):
                    line_.append(i[0])

            # print('line_', line_)

            for j in range(0, len(line_) - 1):
                for k in range(0, len(line_) - 1 - j):
                    aaa = abs(line_[j][1] - line_[k + 1 + j][1])
                    if (aaa < 40 and aaa > 3):
                        if (abs(line_[j][1] - line_[k + 1 + j][1]) != 0):
                            line__.append(line_[j])
                            line__.append(line_[k + 1 + j])
            # print('p', line__)

            b = len(line__)
            # print('t', b)
            u = 0
            for i in range(0, b):

                k = 0
                p = i + 1
                for j in range(p, b):
                    if (line__[i][0] == line__[j - k][0]):
                        # print('i', i)
                        # print('j-k',j-k)
                        del (line__[j - k])
                        # print(line__)
                        k += 1
                        u += 1
            line_y = []
            if line__ is not None:
                for i in range(len(line__)):
                    if (line__[i][1] < line__[i][3]):
                        line_y.append(line__[i][3])
                    elif (line__[i][1] > line__[i][3]):
                        line_y.append(line__[i][1])
                    elif (line__[i][1] == line__[i][3]):
                        line_y.append(line__[i][1])
                    stop_y_data = max(line_y)
                    y_min = min(line_y)

                    roi_y_max = stop_y_data + 15
                    roi_y_min = y_min - 15


            return line__
        else:
            roi_y_max = 150
            roi_y_min = 30

    def draw_lines(self, img, lines):
        for i in lines:
            cv2.line(img, (i[0], i[1]), (i[2], i[3]), (0, 0, 255), 2)

        return img


def main():
    global fit2

    rospy.init_node("cam_test", anonymous=True)
    pub = rospy.Publisher("cam", cam, queue_size = 1)
    msg = cam()

    # 오른쪽 차선
    cap_R = cv2.VideoCapture(0)
    cap_R.set(3, 640)
    cap_R.set(4, 380)
    # 왼쪽 차선
    cap_L = cv2.VideoCapture(1)
    cap_L.set(3, 640)
    cap_L.set(4, 380)

    ld = Lane_Detection()
    
    while not rospy.is_shutdown():
        ret_L, frame_L = cap_L.read()
        ret_L, frame_R = cap_R.read()

        #cv2.line(frame_L,(320,0),(320,380), (0, 0, 255), 1, cv2.LINE_AA)
        # 왼쪽
        img_pro_L, img_size_L = ld.img_process(frame_L)
        result_L, mean_L = ld.lane_detection_L(img_pro_L, img_size_L)

        # 오른쪽
        img_pro_R, img_size_R = ld.img_process(frame_R)
        result_R, mean_R = ld.lane_detection_R(img_pro_R, img_size_R)

        cv2.imshow("L", result_L)
        cv2.imshow("R", result_R)


        # 보내주는곳
        if mean_L != [] and mean_R !=[]:
            msg.x1_L = mean_L[0]
            msg.y1_L = mean_L[1]
            msg.x2_L = mean_L[2]
            msg.y2_L = mean_L[3]
            
            msg.x1_R = mean_R[0]
            msg.y1_R = mean_R[1]
            msg.x2_R = mean_R[2]
            msg.y2_R = mean_R[3]

            msg.left = 1
            msg.right = 1
            
            print("two")
        elif mean_L !=[] and mean_R==[]:
            msg.x1_L = mean_L[0]
            msg.y1_L = mean_L[1]
            msg.x2_L = mean_L[2]
            msg.y2_L = mean_L[3]
            
            msg.x1_R = 0
            msg.y1_R = 0
            msg.x2_R = 0
            msg.y2_R = 0    
            
            msg.left = 1
            msg.right = 0
            
            print("left")
        elif mean_R !=[] and mean_L==[]:
            msg.x1_R = mean_R[0]
            msg.y1_R = mean_R[1]
            msg.x2_R = mean_R[2]
            msg.y2_R = mean_R[3]
            
            msg.x1_L = 0
            msg.y1_L = 0
            msg.x2_L = 0
            msg.y2_L = 0  
            
            msg.left = 0
            msg.right = 1
            
            print("right")  
        else:
            msg.x1_L = 0
            msg.y1_L = 0
            msg.x2_L = 0
            msg.y2_L = 0 
            msg.x1_R = 0
            msg.y1_R = 0
            msg.x2_R = 0
            msg.y2_R = 0  
            msg.left = 0
            msg.right = 0
            
            print("no lane")
        

        pub.publish(msg)
        #cv2.imshow('result',result_L)

        if cv2.waitKey(1) > 0: break

    cap_L.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
