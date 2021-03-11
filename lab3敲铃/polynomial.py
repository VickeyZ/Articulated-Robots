#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
import math
from std_msgs.msg import Float32MultiArray,Float64MultiArray
import matplotlib.pyplot as plt

# pos
# d_1 = [0.03,-0.01,0]
# d_2 = [0.03,0,0]
# d_3 = [0.02,0,0.01]
# v_se = [0, 0.25, -0.05, 0.1, -0.14, -0.02]
# t_1 = 3.8 
# t_2 = t_1 + 4.8
# t_3 = t_2 + 5.8
# q_e1 = [0.26+d_1[0],0.15+d_1[1],0.20+d_1[2],     0,   0,    math.pi/2]
# q_e2 = [0.36+d_2[0],0+d_2[1],0.3+d_2[2],         0,   0,    math.pi/2]
# q_e3 = [0.28+d_3[0],-0.24+d_3[1],0.08+d_3[2],    0,   0,    math.pi/2]
# t_ss = 8

d_1 = [0.03,-0.01,0]
d_2 = [0.03,0,0]
d_3 = [0.02,0,0.01]
v_se = [0, 0, 0, 0, 0, 0]
t_1 = 3.8 
t_2 = t_1 + 4.8
t_3 = t_2 + 5.8
q_e1 = [0.26+d_1[0],0.15+d_1[1],0.20+d_1[2],     0,   0,    math.pi*8/9]
q_e2 = [0.36+d_2[0],0+d_2[1],0.3+d_2[2],         0,   0,    math.pi/2]
q_e3 = [0.28+d_3[0],-0.24+d_3[1],0.08+d_3[2],    0,   0,    math.pi/4]
t_ss = 6

# vel
# d_1 = [0.03,-0.01,0]
# d_2 = [0.03,0,0]
# d_3 = [0.02,0,0.01]          
# # d_1 = [0,0,0]
# # d_2 = [0,0,0]
# # d_3 = [0,0,0]
# v_se = [0, 0, 0, 0, 0, 0]
# t_1 = 3
# t_2 = t_1 + 5
# t_3 = t_2 + 5
# q_e1 = [0.26+d_1[0],0.15+d_1[1],0.20+d_1[2],     0,   0,    math.pi/2]
# q_e2 = [0.36+d_2[0],0+d_2[1],0.3+d_2[2],         0,   0,    math.pi/2]
# q_e3 = [0.28+d_3[0],-0.24+d_3[1],0.08+d_3[2],    0,   0,    math.pi/2]
# t_ss = 5

# adjust
freq = 200
count = 3

do = 1
do_plot = 0

def forward(angle):
    theta = angle*1
    theta[1] += math.pi/2
    theta[3] += math.pi
    theta[4] += math.pi/2
    dh = [[0,math.pi/2,0.284],
        [0.225,0,0],
        [0,math.pi/2,0],
        [0,math.pi/2,0.2289],
        [0,math.pi/2,0],
        [0,-math.pi/2,0.173]]
    x_p = np.array([0,0,0,1])
    T = np.identity(4)
    for i in range(6):
        T_tmp = np.array([math.cos(theta[i]),-math.cos(dh[i][1])*math.sin(theta[i]),
                    math.sin(dh[i][1])*math.sin(theta[i]),dh[i][0]*math.cos(theta[i]),
                    math.sin(theta[i]),math.cos(dh[i][1])*math.cos(theta[i]),
                    -math.sin(dh[i][1])*math.cos(theta[i]),dh[i][0]*math.sin(theta[i]),
                    0,math.sin(dh[i][1]),math.cos(dh[i][1]),dh[i][2],0,0,0,1]).reshape(4,4)
        T = np.dot(T,T_tmp)
    result = np.dot(T,x_p)
    # rpy
    roll = math.atan2(T[2,1],T[2,2])
    pitch = -math.asin(T[2,0])
    yaw = math.atan2(T[1,0],T[0,0])
    pose = [result[0],result[1],result[2],roll,pitch,yaw]
    return(pose)

def angle_range(angle):
    output = []
    for i in angle:
        while i > math.pi:
            i = i - math.pi*2
        while i <= -math.pi:
            i = i + math.pi*2
        output.append(i)
    return output  

def inverse(pose):
    fx,fy,fz,r,p,y = pose
    answer = []
    l_2 = 0.225
    l_1 = 0.284
    l_3 = 0.2289
    d_6 = 0.173
    fx,fy,fz,r,p,y = pose
    T_70 = np.array([math.cos(p)*math.cos(y),math.sin(r)*math.sin(p)*math.cos(y)-math.cos(r)*math.sin(y),
                    math.sin(r)*math.sin(y)+math.cos(r)*math.sin(p)*math.cos(y),fx,
                    math.cos(p)*math.sin(y),math.cos(r)*math.cos(y)+math.sin(r)*math.sin(p)*math.sin(y),
                    math.cos(r)*math.sin(p)*math.sin(y)-math.cos(y)*math.sin(r),fy,-math.sin(p),
                    math.cos(p)*math.sin(r),math.cos(r)*math.cos(p),fz,0,0,0,1]).reshape(4,4)
    T_76 = np.identity(4)
    T_76[1,3] -= d_6
    T_60 = np.dot(T_70,np.mat(T_76).I)
    R_60 = T_60[0:3,0:3]
    dx,dy,dz = [T_60[0,3],T_60[1,3],T_60[2,3]]

    # solution
    for theta_1 in [math.atan2(dy,dx),math.atan2(dy,dx)+math.pi]:
        a = -2*l_2*(dx*math.cos(theta_1)+dy*math.sin(theta_1))
        b = 2*l_2*(l_1-dz)
        c = l_3**2-((dx*math.cos(theta_1)+dy*math.sin(theta_1))**2+l_1**2-2*l_1*dz+l_2**2+dz**2)
        r = math.sqrt(a**2+b**2)        
        for theta_2 in [math.atan2(c,math.sqrt(r**2-c**2)) - math.atan2(a,b),math.atan2(c,-math.sqrt(r**2-c**2)) - math.atan2(a,b)]:
            theta_3a = math.atan2(dx*math.cos(theta_1)+dy*math.sin(theta_1)-l_2*math.cos(theta_2),l_1+l_2*math.sin(theta_2)-dz)-theta_2
            for theta_3 in [theta_3a,theta_3a+math.pi]:
                R_30 = np.array([math.cos(theta_1)*math.cos(theta_2+theta_3),math.sin(theta_1),
                                math.cos(theta_1)*math.sin(theta_2+theta_3),math.sin(theta_1)*math.cos(theta_2+theta_3),
                                -math.cos(theta_1),math.sin(theta_1)*math.sin(theta_2+theta_3),math.sin(theta_2+theta_3),
                                0,-math.cos(theta_2+theta_3)]).reshape(3,3)
                R_63 = np.dot(np.mat(R_30).I,R_60)

                theta_4_1 = math.atan2(R_63[1,1],R_63[0,1])
                theta_4_2 = theta_4_1 + math.pi
                theta_5_1 = math.atan2(math.sqrt(R_63[0,1]**2+R_63[1,1]**2),R_63[2,1])
                theta_5_2 = theta_5_1 + math.pi
                theta_5_3 = math.atan2(-math.sqrt(R_63[0,1]**2+R_63[1,1]**2),R_63[2,1])
                theta_5_4 = theta_5_3 + math.pi
                theta_6_1 = math.atan2(R_63[2,2],-R_63[2,0])
                theta_6_2 = theta_6_1 + math.pi
                for i in [theta_4_1,theta_4_2]:
                    for j in [theta_5_1,theta_5_2,theta_5_3,theta_5_4]:
                        for k in [theta_6_1,theta_6_2]:
                            if abs(math.cos(k)*math.sin(j)-R_63[2,0]) < 0.001 and abs(-math.cos(i)*math.sin(j)-R_63[0,1]) < 0.001:
                                theta_4 = i
                                theta_5 = j
                                theta_6 = k
                                answer.append([theta_1,theta_2-math.pi/2,theta_3,theta_4-math.pi,theta_5-math.pi/2,theta_6])
    answer_f = []
    for i in range(len(answer)):
        pose_inv = forward(answer[i])
        sum = 0
        for j in range(6):
            sum += (pose[j] - pose_inv[j])**2
        if sum < 0.01:
            angle_r = angle_range(answer[i])
            if -math.pi<=angle_r[0]<=math.pi and -2<=angle_r[1]<=2 \
            and -math.pi<=angle_r[3]<=math.pi and -0.79<=angle_r[4]<=3.93 and -math.pi<=angle_r[5]<=math.pi:
                answer_f.append(angle_range(answer[i]))
    return answer_f

def judge(angle_0,angle_1):
    sum_list = []
    for i in range(len(angle_1)):
        sum = 0
        for j in range(len(angle_0)-2):
            sum += (angle_0[j]-angle_1[i][j])**2
        sum_list.append(sum)
    return sum_list.index(min(sum_list))

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
    ], dtype=np.float)
    q = np.matrix([[q_array[0], v_s, 0, q_array[1], q_array[2], q_array[3], v_e, 0]])
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

def talker(v_list):
    if do:
        rospy.init_node("control_example")
        pub = rospy.Publisher('speed_chatter',Float32MultiArray, queue_size=1)
        # pub = rospy.Publisher('/probot_anno/arm_vel_controller/command',Float64MultiArray, queue_size=10)
        rate = rospy.Rate(freq)

        msg = Float32MultiArray()
        msg2 = Float64MultiArray()
        for i in range(len(v_list[0])):
            msg.data = [v_list[0][i]*30*180/math.pi,
                        v_list[1][i]*205*180/math.pi/3,
                        v_list[2][i]*50*180/math.pi,
                        v_list[3][i]*125*180/math.pi/2,
                        v_list[4][i]*125*180/math.pi/2,
                        v_list[5][i]*200*180/math.pi/9]
            msg2.data = [v_list[0][i],
                        v_list[1][i],
                        v_list[2][i],
                        v_list[3][i],
                        v_list[4][i],
                        v_list[5][i]]
            pub.publish(msg)
            rate.sleep()

def talker_pos(q_list):
    # print('11',q_list)
    rospy.init_node("control_example")
    pub = rospy.Publisher('position_chatter',Float32MultiArray, queue_size=10)
    # pub = rospy.Publisher('/probot_anno/arm_pos_controller/command',Float64MultiArray, queue_size=10)
    rate = rospy.Rate(freq)

    msg = Float32MultiArray()
    msg2 = Float64MultiArray()
    for i in range(len(q_list[0])):
        msg.data = [q_list[0][i]*30*180/math.pi,
                    q_list[1][i]*205*180/math.pi/3,
                    q_list[2][i]*50*180/math.pi,
                    q_list[3][i]*125*180/math.pi/2,
                    q_list[4][i]*125*180/math.pi/2,
                    q_list[5][i]*200*180/math.pi/9]
        msg2.data = [q_list[0][i],
                        q_list[1][i],
                        q_list[2][i],
                        q_list[3][i],
                        q_list[4][i],
                        q_list[5][i]]          
        # print('2',msg.data)
        pub.publish(msg)
        rate.sleep()

def get_v_list(t_array,q_array,v_s,v_e):
    a_array = get_a_array(t_array, q_array, v_s, v_e).T.tolist()[0]
    t,q,v,a = [],[],[],[]
    for ii, jj, kk, ll in get_q_interpolated_debug(a_array, 1/freq, t_array[-1]):
        t.append(ii)
        q.append(jj)
        v.append(kk)
        a.append(ll)
    return q,v,a,max(map(abs,v)),max(map(abs,a))

def plot_v(v_plot,a_plot,q_plot):
    plt.figure(figsize=(24,12))
    x = np.arange(0,len(v_plot[0]))
    for i in range(6):
        plt.subplot(3,6,i+1)
        plt.title('joint_v %d' % int(i+1))
        plt.plot(x,v_plot[i])
        plt.subplot(3,6,i+7)
        plt.title('joint_a %d' % int(i+1))
        plt.plot(x,a_plot[i])
        plt.subplot(3,6,i+13)
        plt.title('joint_q %d' % int(i+1))
        plt.plot(x,q_plot[i])
    plt.show()

def main():
    angle_l = [0,0,0,0,0,0]
    # angle_l = [-0.7086262721276702, -1.4473261315888384, 0.5653481415130802, -4.440892098500626e-16, 0.8819779900757583, -0.7086262721276704]
    t_array = [0,t_1,t_2,t_3]

    angle_1 = inverse(q_e1)
    angle_2 = inverse(q_e2)
    angle_3 = inverse(q_e3)
    print(angle_1)
    print(angle_2)
    print(angle_3)
    angle_f3 = angle_l
    angle_f1 = angle_1[judge(angle_f3,angle_1)]
    angle_f2 = angle_2[judge(angle_f1,angle_2)]
    angle_f3 = angle_3[judge(angle_f2,angle_3)]
    print(' ')
    print(angle_f1)
    print(angle_f2)
    print(angle_f3)

    vvv = []
    aaa = []
    v_plot = [[] for i in range(6)]
    a_plot = [[] for i in range(6)]
    q_plot = [[] for i in range(6)]
    for k in range(count + 1):
        v_s = v_se * 1
        v_e = v_se * 1
        vv,aa = [[] for i in range(6)],[[] for i in range(6)]
        v_list = []
        a_list = []
        q_list = []
        # for every joint
        for i in range(6):
            v_ss = v_s[i]
            v_ee = v_e[i]
            if k == 0:
                q_array = [0,0,0,angle_f1[i]]
                t_array = [0,0.01,0.02,t_ss]
                # t_array = [0,t_1,t_2,t_3]
                v_ss = 0
                v_ee = 0
            else:
                q_array = [angle_f1[i],angle_f2[i],angle_f3[i],angle_f1[i]]
                t_array = [0,t_1,t_2,t_3]
            if k == count:
                v_ee = 0
            
            q_list_e,v_list_e,a_list_e,v_m,a_m = get_v_list(t_array,q_array, v_ss, v_ee)
            v_list.append(v_list_e)
            a_list.append(a_list_e)
            q_list.append(q_list_e)
            vv[i].append(v_m)
            aa[i].append(a_m)
        vvv.append([max(i) for i in vv])
        aaa.append([max(i) for i in aa])

        if do:
            # talker(v_list)
            talker_pos(q_list)

        for i in range(6):
            v_plot[i].extend(v_list[i])
            a_plot[i].extend(a_list[i])
            q_plot[i].extend(q_list[i])

    # find max
    v_max = []
    a_max = []
    for i in range(6):
        v_tmp = []
        a_tmp = []
        for j in vvv:
            v_tmp.append(j[i])
        for j in aaa:
            a_tmp.append(j[i])
        v_max.append(max(v_tmp))
        a_max.append(max(a_tmp))               
    
    print(' ')
    print('v_max',v_max)
    print(' ')
    print('a_max',a_max)
    print(' ')
    
    # plot
    if do_plot:
        plot_v(v_plot,a_plot,q_plot) 

if __name__ == '__main__':
    main()
