#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
import math
from std_msgs.msg import Float32MultiArray,Float64MultiArray
import matplotlib.pyplot as plt
import time

# set the publish_frequency
freq = 100

# the position of the bell if fixed
# we use inverse-kinetics to get the position of each bell in advance
# adjust the parameters according to the actual situation
# round 1
angle_f0 = [0,0,0,0,0,0]
angle_f1 = [1.0452125332432862, -0.871101392244204, -0.6941446695176527, 1.0452192180606055, 3.138807995917971, -1.5755974915671098]
angle_f2 = [5.664810060227062e-17, -0.42192725855092563, -0.5183907457662237, 0.0, 2.611114331112046, -1.5707963267948968]
angle_f3 = [-1.151412801444313, -1.4747109569263093, 0.07428596381946759, -1.1568102626815078, 3.0724980418585375, -1.414940442512642]
# round 2
angle2_f1 = [0.9452125332432862, -0.871101392244204, -0.7141446695176527, 1.0452192180606055, 3.018807995917971, -1.5755974915671098]
angle2_f2 = [5.664810060227062e-17, -0.42192725855092563, -0.5143907457662237, 0.0, 2.611114331112046, -1.5707963267948968]
angle2_f3 = [-1.151412801444313, -1.4747109569263093, 0.07428596381946759, -1.1568102626815078, 3.0724980418585375, -1.414940442512642]
# round 3
angle3_f1 = [0.94, -0.871101392244204, -0.7941446695176527, 1.0452192180606055, 3.138807995917971, -1.5755974915671098]
angle3_f2 = [5.664810060227062e-17, -0.42192725855092563, -0.5143907457662237, 0.0, 2.611114331112046, -1.5707963267948968]
angle3_f3 = [-1.151412801444313, -1.4747109569263093, 0.07428596381946759, -1.1568102626815078, 3.0724980418585375, -1.414940442512642]
# round 4
angle4_f1 = [0.94, -0.871101392244204, -0.7941446695176527, 1.0452192180606055, 3.138807995917971, -1.5755974915671098]
angle4_f2 = [5.664810060227062e-17, -0.58192725855092563, -0.3043907457662237, 0.0, 2.611114331112046, -1.5707963267948968]
angle4_f3 = [-1.151412801444313, -1.647109569263093, 0.47428596381946759, -1.1568102626815078, 3.0724980418585375, -1.414940442512642]
# round 5
angle5_f1 = [0.97, -0.871101392244204, -0.7441446695176527, 1.0452192180606055, 3.138807995917971, -1.5755974915671098]
angle5_f2 = [5.664810060227062e-17, -0.6192725855092563, -0.15043907457662237, 0.0, 2.611114331112046, -1.5707963267948968]
angle5_f3 = [-0.951412801444313, -1.647109569263093, 0.67428596381946759, -1.1568102626815078, 3.1724980418585375, -1.414940442512642]
# round 6
angle6_f1 = [1.0, -0.871101392244204, -0.7441446695176527, 1.0452192180606055, 3.138807995917971, -1.5755974915671098]
angle6_f2 = [5.664810060227062e-17, -0.6192725855092563, -0.15043907457662237, 0.0, 2.611114331112046, -1.5707963267948968]
angle6_f3 = [-0.951412801444313, -1.647109569263093, 0.67428596381946759, -1.1568102626815078, 3.1724980418585375, -1.414940442512642]
# round 7
angle7_f1 = [1.0, -0.871101392244204, -0.7441446695176527, 1.0452192180606055, 3.138807995917971, -1.5755974915671098]
angle7_f2 = [5.664810060227062e-17, -0.6192725855092563, -0.15043907457662237, 0.0, 2.611114331112046, -1.5707963267948968]
angle7_f3 = [-0.951412801444313, -1.647109569263093, 0.67428596381946759, -1.1568102626815078, 3.1724980418585375, -1.414940442512642]
# round 8
angle8_f1 = [1.0, -0.871101392244204, -0.7441446695176527, 1.0452192180606055, 3.138807995917971, -1.5755974915671098]
angle8_f2 = [5.664810060227062e-17, -0.6192725855092563, -0.15043907457662237, 0.0, 2.611114331112046, -1.5707963267948968]
angle8_f3 = [-0.951412801444313, -1.647109569263093, 0.67428596381946759, -1.1568102626815078, 3.1724980418585375, -1.414940442512642]

# 以下6个函数为插值路径规划内容
# get_a_array: 通过期望运动到某位置所需的时间、起始与结束的速度解算出插值的参数a_n。
# get_v_list: 通过插值参数与时间解算出机械臂某时刻的运动速度。
# get_q_interpolated_debug: 通过解算的a_n与某个时间对应的位置、速度与加速度
def get_v_list(t_array,q_array,v_s,v_e):
    a_array = get_a_array(t_array, q_array, v_s, v_e).T.tolist()[0]
    t,q,v,a = [],[],[],[]
    for ii, jj, kk, ll in get_q_interpolated_debug(a_array, 1/freq, t_array[-1]):
        t.append(ii)
        q.append(jj)
        v.append(kk)
        a.append(ll)
    return q,v,a,max(map(abs,v)),max(map(abs,a))

def get_v_list_2(t_array,q_array,v_s,v_e):
    a_array = get_a_array_2(t_array, q_array, v_s, v_e).T.tolist()[0]
    t,q,v,a = [],[],[],[]
    for ii, jj, kk, ll in get_q_interpolated_debug_2(a_array, 1/freq, t_array[-1]):
        t.append(ii)
        q.append(jj)
        v.append(kk)
        a.append(ll)
    return q,v,a,max(map(abs,v)),max(map(abs,a))

def get_a_array(t_array, q_array, v_s, v_e):
    """
    parameter:
        t_array: [t_0, t_1, t_2, t_3]
        q_array: [q_0, q_1, q_2, q_3]
    """
    t_0, t_1, t_2, t_3 = [t_array[0]], [t_array[1]], [t_array[2]], [t_array[3]]
    for _ in range(6):
        t_0.append(t_0[0]*t_0[-1])
        t_1.append(t_1[0]*t_1[-1])
        t_2.append(t_2[0]*t_2[-1])
        t_3.append(t_3[0]*t_3[-1])

    m1 = np.matrix([
        [1, t_0[0], t_0[1], t_0[2], t_0[3], t_0[4], t_0[5], t_0[6]],
        [0, 1, 2*t_0[0], 3*t_0[1], 4*t_0[2], 5*t_0[3], 6*t_0[4], 7*t_0[5]],
        [0, 0, 2, 6*t_0[0], 12*t_0[1], 20*t_0[2], 30*t_0[3], 42*t_0[4]],
        [1, t_1[0], t_1[1], t_1[2], t_1[3], t_1[4], t_1[5], t_1[6]],
        [1, t_2[0], t_2[1], t_2[2], t_2[3], t_2[4], t_2[5], t_2[6]],
        [1, t_3[0], t_3[1], t_3[2], t_3[3], t_3[4], t_3[5], t_3[6]],
        [0, 1, 2*t_3[0], 3*t_3[1], 4*t_3[2], 5*t_3[3], 6*t_3[4], 7*t_3[5]],
        [0, 0, 2, 6*t_3[0], 12*t_3[1], 20*t_3[2], 30*t_3[3], 42*t_3[4]]
    ], dtype=np.float64)
    q = np.matrix([[q_array[0], v_s, 0, q_array[1], q_array[2], q_array[3], v_e, 0]])
    return np.linalg.inv(m1)*q.T

def get_a_array_2(t_array, q_array, v_s, v_e):
    """
    parameter:
        t_array: [t_0, t_1, t_2, t_3]
        q_array: [q_0, q_1, q_2, q_3]
    """
    t_0, t_3 = [t_array[0]], [t_array[1]]
    for _ in range(6):
        t_0.append(t_0[0]*t_0[-1])
        t_3.append(t_3[0]*t_3[-1])

    m1 = np.matrix([
        [1, t_0[0], t_0[1], t_0[2], t_0[3], t_0[4]],
        [0, 1, 2*t_0[0], 3*t_0[1], 4*t_0[2], 5*t_0[3]],
        [0, 0, 2, 6*t_0[0], 12*t_0[1], 20*t_0[2]],
        [1, t_3[0], t_3[1], t_3[2], t_3[3], t_3[4]],
        [0, 1, 2*t_3[0], 3*t_3[1], 4*t_3[2], 5*t_3[3]],
        [0, 0, 2, 6*t_3[0], 12*t_3[1], 20*t_3[2]]
    ], dtype=np.float64)
    q = np.matrix([[q_array[0], v_s, 0, q_array[1], v_e, 0]])
    return np.linalg.inv(m1)*q.T

def get_q_interpolated_debug(a_array, time_step, time_stop):
    """
        returns : [t, q, q', q'']
    """
    time = 0
    while time <= time_stop:
        time_array = [time]
        for _ in range(6):
            time_array.append(time_array[0]*time_array[-1])
        yield (
            time, 
            a_array[0] + a_array[1]*time_array[0] + a_array[2]*time_array[1] + a_array[3]*time_array[2] + a_array[4]*time_array[3] + a_array[5]*time_array[4] + a_array[6]*time_array[5] + a_array[7]*time_array[6],
            a_array[1] + 2*a_array[2]*time_array[0] + 3*a_array[3]*time_array[1] + 4*a_array[4]*time_array[2]+ 5*a_array[5]*time_array[3]+ 6*a_array[6]*time_array[4] + 7*a_array[7]*time_array[5],
            2*a_array[2] + 6*a_array[3]*time_array[0] + 12*a_array[4]*time_array[1] + 20*a_array[5]*time_array[2]+ 30*a_array[6]*time_array[3] + 42*a_array[7]*time_array[4]
            )
        time += time_step

def get_q_interpolated_debug_2(a_array, time_step, time_stop):
    """
        returns : [t, q, q', q'']
    """
    time = 0
    while time <= time_stop:
        time_array = [time]
        for _ in range(6):
            time_array.append(time_array[0]*time_array[-1])
        yield (
            time, 
            a_array[0] + a_array[1]*time_array[0] + a_array[2]*time_array[1] + a_array[3]*time_array[2] + a_array[4]*time_array[3] + a_array[5]*time_array[4],
            a_array[1] + 2*a_array[2]*time_array[0] + 3*a_array[3]*time_array[1] + 4*a_array[4]*time_array[2]+ 5*a_array[5]*time_array[3],
            2*a_array[2] + 6*a_array[3]*time_array[0] + 12*a_array[4]*time_array[1] + 20*a_array[5]*time_array[2]
            )
        time += time_step

# initiate rospy node
rospy.init_node("control_example",anonymous=True)

# set a extimated circle time
count = 9
for j in range(count):
    if j == 0:
        # step 1
        # move from init_pose to the first bell
        print(j)
        v_list = []
        a_list = []
        q_list = []
        # for every joint
        for i in range(6):
            q_array = [0,angle_f1[i]]
            t_array = [0,12]
            
            q_list_e,v_list_e,a_list_e,v_m,a_m = get_v_list_2(t_array,q_array, 0, 0)
            
            v_list.append(v_list_e)
            a_list.append(a_list_e)
            q_list.append(q_list_e)
    
    # if j>0, do the circle bell1-bell2-bell3-bell1
    elif j == 1:
        print(j)
        v_list = []
        a_list = []
        q_list = []
        # for every joint
        for i in range(6):
            q_array = [angle_f1[i],angle_f2[i],angle_f3[i],angle_f1[i]]
            t_array = [0,4,8,12]
            
            q_list_e,v_list_e,a_list_e,v_m,a_m = get_v_list(t_array,q_array, 0, 0)
            
            v_list.append(v_list_e)
            a_list.append(a_list_e)
            q_list.append(q_list_e)
    elif j == 2:
        print(j)
        v_list = []
        a_list = []
        q_list = []
        # for every joint
        for i in range(6):
            q_array = [angle_f1[i],angle2_f2[i],angle2_f3[i],angle2_f1[i]]
            t_array = [0,4,8,12]
            
            q_list_e,v_list_e,a_list_e,v_m,a_m = get_v_list(t_array,q_array, 0, 0)
            
            v_list.append(v_list_e)
            a_list.append(a_list_e)
            q_list.append(q_list_e)
    elif j == 3:
        print(j)
        v_list = []
        a_list = []
        q_list = []
        # for every joint
        for i in range(6):
            q_array = [angle_f1[i],angle3_f2[i],angle3_f3[i],angle3_f1[i]]
            t_array = [0,4,8,12]
            
            q_list_e,v_list_e,a_list_e,v_m,a_m = get_v_list(t_array,q_array, 0, 0)
            
            v_list.append(v_list_e)
            a_list.append(a_list_e)
            q_list.append(q_list_e)
    elif j == 4:
        print(j)
        v_list = []
        a_list = []
        q_list = []
        # for every joint
        for i in range(6):
            q_array = [angle_f1[i],angle4_f2[i],angle4_f3[i],angle4_f1[i]]
            t_array = [0,4,8.2,12.2]
            
            q_list_e,v_list_e,a_list_e,v_m,a_m = get_v_list(t_array,q_array, 0, 0)
            
            v_list.append(v_list_e)
            a_list.append(a_list_e)
            q_list.append(q_list_e)
    elif j == 5:
        print(j)
        v_list = []
        a_list = []
        q_list = []
        # for every joint
        for i in range(6):
            q_array = [angle_f1[i],angle5_f2[i],angle5_f3[i],angle5_f1[i]]
            t_array = [0,4,8.2,12.2]
            
            q_list_e,v_list_e,a_list_e,v_m,a_m = get_v_list(t_array,q_array, 0, 0)
            
            v_list.append(v_list_e)
            a_list.append(a_list_e)
            q_list.append(q_list_e)
    elif j == 6:
        print(j)
        v_list = []
        a_list = []
        q_list = []
        # for every joint
        for i in range(6):
            q_array = [angle_f1[i],angle6_f2[i],angle6_f3[i],angle6_f1[i]]
            t_array = [0,4,8,12]
            
            q_list_e,v_list_e,a_list_e,v_m,a_m = get_v_list(t_array,q_array, 0, 0)
            
            v_list.append(v_list_e)
            a_list.append(a_list_e)
            q_list.append(q_list_e)
    elif j == 7:
        print(j)
        v_list = []
        a_list = []
        q_list = []
        # for every joint
        for i in range(6):
            q_array = [angle_f1[i],angle7_f2[i],angle7_f3[i],angle7_f1[i]]
            t_array = [0,4,8,12]
            
            q_list_e,v_list_e,a_list_e,v_m,a_m = get_v_list(t_array,q_array, 0, 0)
            
            v_list.append(v_list_e)
            a_list.append(a_list_e)
            q_list.append(q_list_e)
    elif j == 8:
        print(j)
        v_list = []
        a_list = []
        q_list = []
        # for every joint
        for i in range(6):
            q_array = [angle_f1[i],angle8_f2[i],angle8_f3[i],angle8_f1[i]]
            t_array = [0,4,8,12]
            
            q_list_e,v_list_e,a_list_e,v_m,a_m = get_v_list(t_array,q_array, 0, 0)
            
            v_list.append(v_list_e)
            a_list.append(a_list_e)
            q_list.append(q_list_e)
    else:
        # step 2
        print('else',j)
        v_list = []
        a_list = []
        q_list = []
        # for every joint
        for i in range(6):
            q_array = [angle_f1[i],angle8_f2[i],angle8_f3[i],angle8_f1[i]]
            t_array = [0,4,8,12]
            
            q_list_e,v_list_e,a_list_e,v_m,a_m = get_v_list(t_array,q_array, 0, 0)
            
            v_list.append(v_list_e)
            a_list.append(a_list_e)
            q_list.append(q_list_e)

    pub = rospy.Publisher('speed_chatter',Float32MultiArray, queue_size=1000)
    rate = rospy.Rate(freq)

    for i in range(len(v_list[0])):
        msg = Float32MultiArray()
        msg.data = [v_list[0][i]*30*180/math.pi,
                    v_list[1][i]*205*180/math.pi/3,
                    v_list[2][i]*50*180/math.pi,
                    v_list[3][i]*125*180/math.pi/2,
                    v_list[4][i]*125*180/math.pi/2,
                    v_list[5][i]*200*180/math.pi/9]
        pub.publish(msg)
        rate.sleep()
