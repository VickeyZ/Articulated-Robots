#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Float32MultiArray

cubt = lambda x : np.cbrt(x) #math.pow(x, 1/3)
sqr = lambda x : math.pow(x, 2)
cub = lambda x: math.pow(x, 3)

j_max = 0.2
a_max = [0.3491,0.2618,0.3491,0.4363,0.4363,0.4363]
v_max = [0.7854,0.6981,0.7854,0.9599,0.9599,0.9599]
bs_delta = 0.01
j_vector = [0, j_max, 0, -j_max, 0, -j_max, 0, j_max]

# ending_point
q_e1 = [0.26,0.15,0.08,0,0,0]
q_e2 = [0.36,0,0.3,0,0,0]
q_e3 = [0.28,-0.24,0.08,0,0,0]
q_s = [0.228855,-0.000002,0.453986,1.570801,0,0]

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
        [0,-math.pi/2,0.055]]
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

def get_T(j_acc_max, j_dec_max, a_acc_max, a_dec_max, v_s, v_e, v_max, s_seg, bs_delta):
    T = [0] * 8
    print(j_acc_max, a_acc_max, v_s, v_e, v_max, s_seg)
    # Step 1:
    if v_s < v_e:
        s_acc_min = (v_s+v_e)*math.sqrt((v_e - v_s)/j_acc_max)
        # print(s_acc_min)
        if s_seg < s_acc_min:
            #s_seg = (v_s+v_e)*sqr((v_e-v_s)/j_acc_max)
            x_1 = 4*cub(v_s) + 1.5*(math.sqrt(81*sqr(j_acc_max)*math.pow(s_seg, 4) + 96*cub(v_s)*j_acc_max*sqr(s_seg)) - 8*cub(v_s) - 9*j_acc_max*sqr(s_seg))
            x_2 = 4*cub(v_s) - 1.5*(math.sqrt(81*sqr(j_acc_max)*math.pow(s_seg, 4) + 96*cub(v_s)*j_acc_max*sqr(s_seg)) + 8*cub(v_s) + 9*j_acc_max*sqr(s_seg))
            v_e_p = (-v_s-(cubt(x_1)+cubt(x_2)))/3
            v_max = v_e_p
            v_e = v_e_p
    elif v_s > v_e:
        s_dec_min = (v_s+v_e)*math.sqrt((v_s - v_e)/j_dec_max)
        if s_seg < s_dec_min:
            #s_seg = (v_s+v_e)*sqr((v_s-v_e)/j_dec_max)
            y_1 = 4*cub(v_e) + 1.5*(math.sqrt(81*sqr(j_dec_max)*math.pow(s_seg, 4) + 96*cub(v_e)*j_dec_max*sqr(s_seg)) - 8*cub(v_e) - 9*j_dec_max*sqr(s_seg))
            y_2 = 4*cub(v_e) - 1.5*(math.sqrt(81*sqr(j_dec_max)*math.pow(s_seg, 4) + 96*cub(v_e)*j_dec_max*sqr(s_seg)) + 8*cub(v_e) + 9*j_dec_max*sqr(s_seg))
            v_s_p = (-v_e-(cubt(y_1)+cubt(y_2))) / 3
            v_max = v_s_p
            v_s = v_s_p
    # Step 2:
    if v_s == v_max == v_e:
        T[1] = T[2] = T[3] = 0
        T[4] = s_seg / v_max
        T[5] = T[6] = T[7] = 0
        return T
    # Step 3:
    if (v_max - v_s) > (sqr(a_acc_max) / j_acc_max):
        T[1] = T[3] = a_acc_max / j_acc_max
        T[2] = (v_max - v_s) / a_acc_max - T[1]
    else:
        # print(v_max, '   ', v_s)
        T[1] = T[3] = math.sqrt((v_max - v_s)/j_acc_max)
        T[2] = 0
    
    if (v_max - v_e) > (sqr(a_dec_max) / j_dec_max):
        T[5] = T[7] = a_dec_max / j_dec_max
        T[6] = (v_max - v_e) / a_dec_max - T[5]
    else:
        T[5] = T[7] = math.sqrt((v_max - v_e)/j_dec_max)
        T[6] = 0
    s_a = v_s  *(2*T[1]+T[2]) + 0.5*j_acc_max*sqr(T[1])*(T[1]+T[2]) + 0.5*j_acc_max*T[1]*sqr(T[1]+T[2])
    s_d = v_max*(2*T[5]+T[6]) - 0.5*j_dec_max*sqr(T[5])*(T[5]+T[6]) - 0.5*j_dec_max*T[5]*sqr(T[5]+T[6])
    # Step 4:
    if s_seg > (s_a + s_d):
        T[4] = (s_seg-s_a-s_d)/v_max
        return T
    # Step 5:
    v_max = max(v_s, v_e)
    while True:
        v_max += bs_delta  
        if (v_max-v_s) > (sqr(a_acc_max)/j_acc_max):
            T[1] = T[3] = a_acc_max / j_acc_max
            T[2] = (v_max-v_s)/a_acc_max - T[1]
        else:
            T[1] = T[3] = math.sqrt((v_max-v_s)/j_acc_max)
            T[2] = 0
        
        if (v_max-v_e) > (sqr(a_dec_max)/j_dec_max):
            T[5] = T[7] = a_dec_max / j_dec_max
            T[6] = (v_max-v_e)/a_dec_max - T[5]
        else:
            T[5] = T[7] = math.sqrt((v_max-v_e)/j_dec_max)
            T[6] = 0

        s_a = v_s  *(2*T[1]+T[2]) + 0.5*j_acc_max*sqr(T[1])*(T[1]+T[2]) + 0.5*j_acc_max*T[1]*sqr(T[1]+T[2])
        s_d = v_max*(2*T[5]+T[6]) - 0.5*j_dec_max*sqr(T[5])*(T[5]+T[6]) - 0.5*j_dec_max*T[5]*sqr(T[5]+T[6])

        if s_seg - (s_a + s_d) > bs_delta:
            continue
        else:
            T[4] = 0
            return T

def inverse(pose):
    fx,fy,fz,r,p,y = pose
    answer = []
    l_2 = 0.225
    l_1 = 0.284
    l_3 = 0.2289
    d_6 = 0.055
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
                theta_6_1 = math.atan2(R_63[2,2],-R_63[2,0])
                theta_6_2 = theta_6_1 + math.pi
                for i in [theta_4_1,theta_4_2]:
                    for j in [theta_5_1,theta_5_2]:
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
            if -math.pi<=angle_r[0]<=math.pi and -2<=angle_r[1]<=2 and -0.7<=angle_r[2]<=3.84 \
            and -math.pi<=angle_r[3]<=math.pi and -0.79<=angle_r[4]<=3.93 and -math.pi<=angle_r[5]<=math.pi:
                answer_f.append(angle_range(answer[i]))
    return answer_f

def get_result_vector(j_vector, T_vector, delta_t, fu, v_s):
    # returns (t, j, a, v, q)
    assert(len(j_vector) == len(T_vector))

    t_sum = 0
    t_stage = 1
    t_sum_vector = [0]
    a = 0
    v = v_s

    for i in T_vector[1:]:
        t_sum_vector.append(t_sum_vector[-1] + i)

    while t_sum <= t_sum_vector[-1]:
        if t_sum > t_sum_vector[t_stage]:
            t_stage += 1
        
        t_sum += delta_t
        j = j_vector[t_stage]
        a += delta_t * j
        v += delta_t * a

        if fu < 0:
            yield -v
        else:
            yield v

def judge(angle_0,angle_1):
    sum_list = []
    for i in range(len(angle_1)):
        sum = 0
        for j in range(len(angle_0)):
            sum += (angle_0[j]-angle_1[i][j])**2
        sum_list.append(sum)
    return sum_list.index(min(sum_list))

def fhb(angle,pose,vs,ve):
    angle_0 = angle
    q_e = pose
    angle_1 = inverse(q_e)
    angle_f = angle_1[judge(angle_0,angle_1)]
    print('angle',angle_f)

    s_list = [angle_f[i]-angle_0[i] for i in range(len(angle_0))]
    print(s_list)
    
    v_list = []
    t_list = []
    for s in range(len(s_list)):
        if s_list[s] < 0:
            T = get_T(j_max, j_max, a_max[s], a_max[s], vs, ve, v_max[s], -s_list[s], bs_delta)
            result = list(get_result_vector(j_vector, T, 0.01, s_list[s], vs))
        else:
            T = get_T(j_max, j_max, a_max[s], a_max[s], vs, ve, v_max[s], s_list[s], bs_delta)
            result = list(get_result_vector(j_vector, T, 0.01, s_list[s], vs))
        v_list.append(result)
        t_list.append(len(result))
    t_max = max(t_list) + 1
    # print(t_max)
    return t_max,v_list,t_list,angle_f

def talker(t_max,v_list,t_list):
    rospy.init_node("control_example")
    pub = rospy.Publisher('speed_chatter',Float32MultiArray, queue_size=1)
    rate = rospy.Rate(100)

    msg = Float32MultiArray()
    for i in range(t_max):
        msg.data = []
        if i < t_list[0]:
            msg.data.append(v_list[0][i]*30*180/math.pi)
        else:
            msg.data.append(0)
        if i < t_list[1]:
            msg.data.append(v_list[1][i]*205*180/math.pi/3)
        else:
            msg.data.append(0)
        if i < t_list[2]:
            msg.data.append(v_list[2][i]*50*180/math.pi)
        else:
            msg.data.append(0)
        if i < t_list[3]:
            msg.data.append(v_list[3][i]*125*180/math.pi/2)
        else:
            msg.data.append(0)
        if i < t_list[4]:
            msg.data.append(v_list[4][i]*125*180/math.pi/2)
        else:
            msg.data.append(0)
        if i < t_list[5]:
            msg.data.append(v_list[5][i]*200*180/math.pi/9)
        else:
            msg.data.append(0)     
        print(msg.data,i)
        pub.publish(msg)
        rate.sleep()
    msg.data = [0, 0, 0, 0, 0, 0]
    pub.publish(msg)

def main():
    angle_l = [0,0,0,0,0,0]
    # angle_l = [-0.7086262721276702, -1.4473261315888384, 0.5653481415130802, -4.440892098500626e-16, 0.8819779900757583, -0.7086262721276704]
    count = 5

    vs = [0.1]*count*3
    ve = [0.1]*count*3
    vs[0] = 0
    ve[count-1] = 0   
    q_list = [q_e1,q_e2,q_e3]*count
    # q_list = [q_s]
    for i in range(len(q_list)):
        t_max,v_list,t_list,angle_l = fhb(angle_l,q_list[i],vs[i],ve[i])
        talker(t_max,v_list,t_list)
    print('angle_l',angle_l)

if __name__ == '__main__':
    main()
