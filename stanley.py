#!/usr/bin/env python

import rospy
from openpyxl import load_workbook
import matplotlib.pyplot as plt
import math
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import QuaternionStamped
from pyproj import Proj, transform
from sensor_msgs.msg import NavSatFix
from lms1xx.msg import cam, total_msg
import threading
import serial
from time import sleep
import time
from collections import deque


left_cam_frame_x1 = left_cam_frame_y1 = left_cam_frame_x2 = left_cam_frame_y2 = 0
right_cam_frame_x1 = right_cam_frame_y1 = right_cam_frame_x2 = right_cam_frame_y2 = 0
left_lane = right_lane = 0
def callback_cam(msg):
    global left_cam_frame_x1,left_cam_frame_y1,left_cam_frame_x2,left_cam_frame_y2,right_cam_frame_x1,right_cam_frame_y1,right_cam_frame_x2,right_cam_frame_y2 , left_lane , right_lane
    left_cam_frame_x1 = msg.left_cam_frame_x1
    left_cam_frame_y1 = msg.left_cam_frame_y1
    left_cam_frame_x2 = msg.left_cam_frame_x2
    left_cam_frame_y2 = msg.left_cam_frame_y2
    right_cam_frame_x1 = msg.right_cam_frame_x1
    right_cam_frame_y1 = msg.right_cam_frame_y1
    right_cam_frame_x2 = msg.right_cam_frame_x2
    right_cam_frame_y2 = msg.right_cam_frame_y2
    left_lane = msg.left_lane
    right_lane = msg.right_lane
    

class control():
    def __init__(self, port_num):
        S = 83
        T = 84
        X = 88
        AorM = 1
        ESTOP = 0
        ETX0 = 13
        ETX1 = 10

        ALIVE = 0

        self.port_num = port_num

        self.DATA = bytearray(14)

        self.DATA[0] = S
        self.DATA[1] = T
        self.DATA[2] = X
        self.DATA[3] = AorM
        self.DATA[4] = ESTOP
        self.DATA[12] = ETX0
        self.DATA[13] = ETX1

        self.ALIVE = ALIVE

    def open_serial(self):
        self.ser = serial.Serial(
            port = self.port_num,
            baudrate = 115200,
        )

    def send_data(self, SPEED, STEER, BRAKE, GEAR):

        if STEER >= 0:
            self.DATA[8] = STEER // 256
            self.DATA[9] = STEER % 256
        else:
            STEER = -STEER
            self.DATA[8] = 255 - STEER // 256
            self.DATA[9] = 255 - STEER % 256

        self.DATA[5] = GEAR
        self.DATA[6] = SPEED // 256
        self.DATA[7] = SPEED % 256
        self.DATA[10] = BRAKE
        self.DATA[11] = self.ALIVE

        self.ser.write(bytes(self.DATA))

        self.ALIVE = self.ALIVE + 1
        if self.ALIVE is 256:
            self.ALIVE = 0

    def receive_data(self):

        speed = 0.0
        real_speed = 0.0
        a = 0
        line = []
        for i in self.ser.read():
            #print('1')
            line.append(i)
            if i =="\n":
                break
        if(ord(line[0])==83):
            j=1
            i=0
            while j <18:
                for i in self.ser.read():
                    line.append(i)
                    if i =='\n':
                        break
                j = j+1
            speed=(ord(line[6]) + ord(line[7]) * 256)
            robot_now_heading = int(ord(line[8]) + ord(line[9]) * 256)
            if robot_now_heading > 32768:
                robot_now_heading = robot_now_heading - 65537
        #print("ddddd",float(robot_now_heading * -1 % 71*0.0071))       
        robot_now_heading = float(robot_now_heading * -1 / 71) + float(robot_now_heading * -1 % 71 * 0.0071)
        real_speed=(float((speed%10)*0.1)+speed/10)/3.6
        return robot_now_heading ,real_speed



left_lane_x_1 = deque(maxlen=5)
for i in range(len(left_lane_x_1)):
    left_lane_x_1[i] = 0

left_lane_y_1 = deque(maxlen=5)
for i in range(len(left_lane_y_1)):
    left_lane_y_1[i] = 0

left_lane_x_2 = deque(maxlen=5)
for i in range(len(left_lane_x_2)):
    left_lane_x_2[i] = 0    

left_lane_y_2 = deque(maxlen=5)
for i in range(len(left_lane_y_2)):
    left_lane_y_2[i] = 0



# 왼쪽 카메라 실측 데이터
left_x_l = float(586)
left_y_l = float(871-52.5)
left_y_b = float(52.5)
left_h = float(72.5)
left_s_x = float(640/2)
left_s_y = float(380)

def left_calculation_theta(lane_center,left_sum_car_theta,robot_heading,real_speed):
    global left_lane_y , left_x_l , left_y_l , left_y_b , left_h , left_s_x , left_s_y
    sum_left_lane_y = 0.0

    p_1x = float(left_s_x - left_cam_frame_x1)
    p_1y = float(left_s_y - left_cam_frame_y1)
    p_2x = float(left_s_x - left_cam_frame_x2)
    p_2y = float(left_s_y - left_cam_frame_y2)
    
    alpha = float(np.rad2deg(math.atan2(left_h,left_y_b)))
    beta = float(np.rad2deg(math.atan2(left_x_l,left_y_l+left_y_b)))
    theta = float(np.rad2deg(math.atan2(left_h,left_y_l+left_y_b)))

    real_x1 = left_h * math.tan(np.deg2rad((90-alpha)+(p_1y/left_s_y)*(alpha-theta)))
    real_y1 = real_x1 * math.tan(np.deg2rad((p_1x/left_s_x)*beta))
    real_x2 = left_h * math.tan(np.deg2rad((90-alpha)+(p_2y/left_s_y)*(alpha-theta)))
    real_y2 = real_x2 * math.tan(np.deg2rad((p_2x/left_s_x)*beta))

    y_1 = real_x1 + 56.5
    x_1 = (real_y1 * -1) + 124
    y_2 = real_x2 + 56.5
    x_2 = (real_y2 * -1) + 124

    left_lane_x_1.append(x_1)
    left_lane_y_1.append(y_1)
    left_lane_x_2.append(x_2)
    left_lane_y_2.append(y_2)
    sum_left_lane_x_1 = sum_left_lane_y_1 = sum_left_lane_x_2 = sum_left_lane_y_2 =0
    for i in range (len(left_lane_x_1)):
        sum_left_lane_x_1 += left_lane_x_1[i]
    for i in range (len(left_lane_y_1)):
        sum_left_lane_y_1 += left_lane_y_1[i]
    for i in range (len(left_lane_x_2)):
        sum_left_lane_x_2 += left_lane_x_2[i]
    for i in range (len(left_lane_y_2)):
        sum_left_lane_y_2 += left_lane_y_2[i]
    #print("lane_l",x_1,y_1,x_2,y_2)
    alpha = np.rad2deg(math.atan2(sum_left_lane_y_1/5-sum_left_lane_y_2/5,sum_left_lane_x_2/5-sum_left_lane_x_1/5))
    if real_speed == 0:
       real_speed = 0.00001
    


    #print("len",len(left_lane_y))
    #print("sum",sum_left_lane_y)
    #print("speed",real_speed)
    print("left",robot_heading, alpha,real_speed)
    if real_speed >0.1:
        '''
        if lane_center-(sum_left_lane_y/5) < 5 and lane_center-(sum_left_lane_y/5) >-5:
            car_theta =  (robot_heading-alpha)*-1
            print("left_center!!left_center!!left_center!!left_center!!left_center!!left_center!!left_center!!")
        else:
            car_theta = ( (robot_heading-alpha)*-1 + np.rad2deg(math.atan2((0.001*(lane_center-(sum_left_lane_y/5))),real_speed)) )
            print("left_no!!left_no!!left_no!!left_no!!left_no!!left_no!!left_no!!left_no!!left_no!!left_no!!")
        '''
        car_theta = ( (robot_heading-alpha)*-1 + np.rad2deg(math.atan2((0.005*(lane_center-(sum_left_lane_y_1/5))),real_speed)) )
    else:
        car_theta =  (robot_heading-alpha)*-1 + np.rad2deg(math.atan2((0.07*(lane_center-(sum_left_lane_y_1/5))),1))
        #print("left_no_speed!!left_no_speed!!left_no_speed!!left_no_speed!!left_no_speed!!left_no_speed!!")
    car_theta = car_theta *2 /9
    if alpha > 20 or alpha <-20:
        robot_theta = 7*car_theta
    elif alpha > 14 or alpha <-14:
        robot_theta = 3*car_theta
    elif alpha > 10 or alpha <-10:
        robot_theta = 2*car_theta
    elif alpha > 7 or alpha <-7:
        robot_theta = 0.9*car_theta
    else:
        robot_theta = 0.8*car_theta
    print("left_center",lane_center-(float(sum_left_lane_y_1/5)),sum_left_lane_y_1/5)
        
    #robot_theta = 1.5*car_theta #+ 0.0001*left_sum_car_theta # p i 
    #print("robot_theta_l",robot_theta)
    return robot_theta * 71 , sum_left_lane_y_1/5


right_lane_x_1 = deque(maxlen=5)
for i in range(len(right_lane_x_1)):
    right_lane_x_1[i] = 0

right_lane_y_1 = deque(maxlen=5)
for i in range(len(right_lane_y_1)):
    right_lane_y_1[i] = 0

right_lane_x_2 = deque(maxlen=5)
for i in range(len(right_lane_x_2)):
    right_lane_x_2[i] = 0    

right_lane_y_2 = deque(maxlen=5)
for i in range(len(right_lane_y_2)):
    right_lane_y_2[i] = 0


# 오른쪽 실측 데이터
right_x_l = float(586)
right_y_l = float(871-52.5)
right_y_b = float(52.5)
right_h = float(72.5)
right_s_x = float(640/2)
right_s_y = float(380)

def right_calculation_theta(lane_center,right_sum_car_theta,robot_heading,real_speed):
    global right_lane_y , right_x_l , right_y_l , right_y_b , right_h , right_s_x , right_s_y
    sum_right_lane_y = 0.0

    p_1x = float(right_s_x - right_cam_frame_x1)
    p_1y = float(right_s_y - right_cam_frame_y1)
    p_2x = float(right_s_x - right_cam_frame_x2)
    p_2y = float(right_s_y - right_cam_frame_y2)
    
    alpha = float(np.rad2deg(math.atan2(right_h,right_y_b)))
    beta = float(np.rad2deg(math.atan2(right_x_l,right_y_l+right_y_b)))
    theta = float(np.rad2deg(math.atan2(right_h,right_y_l+right_y_b)))

    real_x1 = right_h * math.tan(np.deg2rad((90-alpha)+(p_1y/right_s_y)*(alpha-theta)))
    real_y1 = real_x1 * math.tan(np.deg2rad((p_1x/right_s_x)*beta))
    real_x2 = right_h * math.tan(np.deg2rad((90-alpha)+(p_2y/right_s_y)*(alpha-theta)))
    real_y2 = real_x2 * math.tan(np.deg2rad((p_2x/right_s_x)*beta))
    #print("real",real_x1,real_x2)
    y_1 = (real_x1 + 56.5 )* -1
    x_1 = real_y1 + 124
    y_2 = (real_x2 + 56.5 ) * -1
    x_2 = real_y2 + 124
    x_ = x_1
    x_1 = x_2
    x_2 = x_
    y_ = y_1
    y_1 = y_2
    y_2 = y_
    right_lane_x_1.append(x_1)
    right_lane_y_1.append(y_1)
    right_lane_x_2.append(x_2)
    right_lane_y_2.append(y_2)
    sum_right_lane_x_1 = sum_right_lane_y_1 = sum_right_lane_x_2 = sum_right_lane_y_2 =0
    for i in range (len(right_lane_x_1)):
        sum_right_lane_x_1 += right_lane_x_1[i]
    for i in range (len(right_lane_y_1)):
        sum_right_lane_y_1 += right_lane_y_1[i]
    for i in range (len(right_lane_x_2)):
        sum_right_lane_x_2 += right_lane_x_2[i]
    for i in range (len(right_lane_y_2)):
        sum_right_lane_y_2 += right_lane_y_2[i]
    alpha = np.rad2deg(math.atan2(sum_right_lane_y_1/5-sum_right_lane_y_2/5,sum_right_lane_x_2/5-sum_right_lane_x_1/5))
    #print("right",sum_right_lane_x_1/5,sum_right_lane_y_1/5,sum_right_lane_x_2/5,sum_right_lane_y_2/5)
    print("right",robot_heading, alpha,real_speed)
    if real_speed >0.1:
        '''
        if -lane_center-(sum_right_lane_y/5) < 5 and -lane_center-(sum_right_lane_y/5) >-5:
            car_theta =  (robot_heading-alpha)*-1
            print("right_center!!right_center!!right_center!!right_center!!right_center!!right_center!!")
        else:
            car_theta = ( (robot_heading-alpha)*-1 + np.rad2deg(math.atan2((0.001*(-lane_center-(sum_right_lane_y/5))),real_speed)) )
            print("right_no!!right_no!!right_no!!right_no!!right_no!!right_no!!right_no!!right_no!!v")
        '''
        car_theta = ( (-robot_heading+alpha) + np.rad2deg(math.atan2((0.005*(-lane_center-(sum_right_lane_y_1/5))),real_speed)) )
    else:
        car_theta =  (-robot_heading+alpha) + np.rad2deg(math.atan2((0.07*(-lane_center-(sum_right_lane_y_1/5))),1))
        #print("right_no_speed!!right_no_speed!!right_no_speed!!right_no_speed!!right_no_speed!!right_no_speed!!")
    print("right_center",-lane_center-(float(sum_right_lane_y_1/5)),sum_right_lane_y_1/5)
    car_theta = car_theta *2 /9
    if alpha >20 or alpha <-20:
        robot_theta = 7*car_theta
    elif alpha > 14 or alpha <-14:
        robot_theta = 3*car_theta
    elif alpha > 10 or alpha <-10:
        robot_theta = 2*car_theta
    elif alpha > 7 or alpha <-7:
        robot_theta = 0.9*car_theta
    else:
        robot_theta = 0.8*car_theta
    #robot_theta = 1.0*car_theta #+ 0.0001*right_sum_car_theta # p i 
    #print("robot_theta_r",robot_theta)
    return robot_theta * 71 , sum_right_lane_y_1/5

speed_p_gain = 2
speed_i_gain = 0.02
speed_err_sum = 0
count = 0

def calculation_speed(want_speed,now_speed):
    global speed_p_gain, speed_i_gain, speed_err_sum,count

    err_speed = want_speed - now_speed
    if now_speed < 0.01:
        count = count + 1
    else:
        count = 0

    if count < 5:  
        speed_err_sum += err_speed

    #print(err_speed,speed_err_sum)
    p_output = speed_p_gain * err_speed
    i_output = speed_i_gain * speed_err_sum
    speed = now_speed + p_output + i_output
    return speed * 36

mycar = control('/dev/ttyUSB0')
mycar.open_serial()
left_sum_car_theta = 0
right_sum_car_theta = 0
theta_ = 0
lane_center = 160
right_y = left_y = 0
def thread():
    global left_sum_car_theta,right_sum_car_theta, theta_,lane_center,right_y,left_y
    f = open("/home/control/catkin_ws/src/topic/lane_5.txt", 'a')
    robot_now_heading, real_speed = mycar.receive_data()

    #print(robot_now_heading,real_speed)
    #print("lene",left_lane,right_lane)
    '''
    if  left_lane == 1 and right_lane == 1:
        left_theta , left_y = left_calculation_theta(lane_center,left_sum_car_theta,robot_now_heading,real_speed)
        right_theta , right_y = right_calculation_theta(lane_center,left_sum_car_theta,robot_now_heading,real_speed)
        lane_center = (left_y - right_y) / 2
        theta_ , left_y = left_calculation_theta(lane_center,left_sum_car_theta,robot_now_heading,real_speed)
        left_sum_car_theta += theta_
        right_sum_car_theta = 0
        #print("two")
    elif left_lane == 0 and right_lane == 1:
        theta_ , right_y = right_calculation_theta(lane_center,left_sum_car_theta,robot_now_heading,real_speed)
        right_sum_car_theta += theta_
        left_sum_car_theta = 0
        #print("right")
    elif left_lane == 1 and right_lane == 0:
        theta_ , left_y = left_calculation_theta(lane_center,left_sum_car_theta,robot_now_heading,real_speed)
        left_sum_car_theta += theta_
        right_sum_car_theta = 0
        #print("left")
    else:
        print("no lane")
    '''
    if right_lane != 0 and left_lane != 0:
        theta_r , right_y = right_calculation_theta(lane_center,right_sum_car_theta,robot_now_heading,real_speed)
        theta_l , left_y = left_calculation_theta(lane_center,left_sum_car_theta,robot_now_heading,real_speed)
        #print("dddddd",theta_l, theta_r)
        if abs(theta_r) > abs(theta_l):
            theta_ = theta_r
        else:
            theta_ = theta_l
        lane_center = float((left_y - right_y)/2)
        #print("lane_center",lane_center)
        #print("lane_lane_lane_lane_lane_lane_lane_lane_lane_lane_lane_lane_lane_lane_lane",lane_center)
    elif left_lane != 0 and right_lane == 0:
        theta_ , left_y = left_calculation_theta(lane_center,left_sum_car_theta,robot_now_heading,real_speed)
    elif left_lane == 0 and right_lane != 0:
        theta_ , right_y = right_calculation_theta(lane_center,right_sum_car_theta,robot_now_heading,real_speed)
    print(theta_)
    print("lane_center",lane_center,right_y,left_y)
    f.write(str(lane_center))
    f.write("\t")
    f.write(str(right_y))
    f.write("\t")
    f.write(str(left_y))
    f.write("\t")
    f.write("\n")
    #theta_ = float(theta_ *  2 / 9)
    if theta_ >=2000:
        theta_ = 1999
    elif theta_<-2000:
        theta_ = -1999
    speed = calculation_speed(1,real_speed)
    print("speed_input",speed)
    if speed > 200:
        speed = 199
    mycar.send_data(int(speed),int(theta_),1,0)
    #print(theta_)
    threading.Timer(0.05, thread).start()
    f.close()

def main():
    rospy.init_node('stanley', anonymous=True)
    rospy.Subscriber('total_msg', total_msg, callback_cam) 
    #sleep(3)
    thread()

if __name__ == "__main__":
    main()

