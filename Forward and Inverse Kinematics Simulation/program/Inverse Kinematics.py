#!/usr/bin/env python
import math
import numpy as np

def forward(angle):
    theta = angle*1
    theta[1] += math.pi/2
    theta[3] += math.pi
    theta[4] += math.pi/2
    # print(theta)
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

def main():
    # angle = [0.1,0.1,0.1,0.1,0.1,0.1]
    angle = [0.5,0.3,0.8,0.1,0.2,0.5]
    # angle = [0,0,0,0,0,0]

    pose = forward(angle)
    print(pose)
    print('')

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
    # print(T_70)
    T_76 = np.identity(4)
    T_76[1,3] -= d_6
    T_60 = np.dot(T_70,np.mat(T_76).I)
    R_60 = T_60[0:3,0:3]
    dx,dy,dz = [T_60[0,3],T_60[1,3],T_60[2,3]]
    # print(T_60)
    # print(dx,dy,dz)

    # solution
    theta_1 = math.atan2(dy,dx)
    a = -2*l_2*(dx*math.cos(theta_1)+dy*math.sin(theta_1))
    b = 2*l_2*(l_1-dz)
    c = l_3**2-((dx*math.cos(theta_1)+dy*math.sin(theta_1))**2+l_1**2-2*l_1*dz+l_2**2+dz**2)
    r = math.sqrt(a**2+b**2)
    
    for theta_2 in [math.atan2(c,math.sqrt(r**2-c**2)) - math.atan2(a,b),math.atan2(c,-math.sqrt(r**2-c**2)) - math.atan2(a,b)]:
        theta_3 = math.atan2(dx*math.cos(theta_1)+dy*math.sin(theta_1)-l_2*math.cos(theta_2),l_1+l_2*math.sin(theta_2)-dz)-theta_2

        R_30 = np.array([math.cos(theta_1)*math.cos(theta_2+theta_3),math.sin(theta_1),
                        math.cos(theta_1)*math.sin(theta_2+theta_3),math.sin(theta_1)*math.cos(theta_2+theta_3),
                        -math.cos(theta_1),math.sin(theta_1)*math.sin(theta_2+theta_3),math.sin(theta_2+theta_3),
                        0,-math.cos(theta_2+theta_3)]).reshape(3,3)

        R_63 = np.dot(np.mat(R_30).I,R_60)
        # print(R_63)

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

    # check and output
    for i in range(len(answer)):
        pose_inv = forward(answer[i])
        sum = 0
        for j in range(6):
            sum += (pose[j] - pose_inv[j])**2
        if sum < 0.01:
            print('angle',answer[i])
            print(pose_inv)
            print(sum)
            print('')


if __name__ == '__main__':
    main()