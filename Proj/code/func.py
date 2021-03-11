import math,rospy
from std_msgs.msg import Float64MultiArray
from para import *

def move(p, v, ts):
    # return the endpoint of p in interval ts
    p_result = []
    p_result.append(p[0] + v[0] * ts)
    p_result.append(p[1] + v[1] * ts)
    p_result.append(p[2] + v[2] * ts)
    return np.array(p_result)

def projection(v, p):
    # return the projection of v on p
    return p*np.inner(v, p)/math.pow(np.linalg.norm(p), 2)

def forward(angle):
    # from angle get the absolute-pose
    theta = angle*1
    theta[1] += math.pi/2
    theta[3] += math.pi
    theta[4] += math.pi/2
    theta[5] -= math.pi
    # the DH para-list
    dh = [[0,math.pi/2,0.241],
      [0.18,0,0],
      [0.027,math.pi/2,0],
      [0,math.pi/2,0.18296],
      [0,math.pi/2,0],
      [0,0,0.06185]]
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
    rpy = angle_range([roll,pitch,yaw])
    pose = [result[0],result[1],result[2],rpy[0],rpy[1],rpy[2]]
    return(pose)

def inverse(pose):
    # from absolute-pose to get the angle of each joint
    fx,fy,fz,r,p,y = pose
    answer = []
    l_1 = 0.241
    l_2 = 0.18
    l_3 = 0.027
    l_4 = 0.18296
    d_6 = 0.06185
    fx,fy,fz,r,p,y = pose
    T_70 = np.array([math.cos(p)*math.cos(y),math.sin(r)*math.sin(p)*math.cos(y)-math.cos(r)*math.sin(y),
                    math.sin(r)*math.sin(y)+math.cos(r)*math.sin(p)*math.cos(y),fx,
                    math.cos(p)*math.sin(y),math.cos(r)*math.cos(y)+math.sin(r)*math.sin(p)*math.sin(y),
                    math.cos(r)*math.sin(p)*math.sin(y)-math.cos(y)*math.sin(r),fy,-math.sin(p),
                    math.cos(p)*math.sin(r),math.cos(r)*math.cos(p),fz,0,0,0,1]).reshape(4,4)
    T_76 = np.identity(4)
    T_76[1,3] -= d_6
    T_76 = np.dot(T_76,np.array([1,0,0,0,0,0,-1,0,0,1,0,0,0,0,0,1]).reshape(4,4))
    T_60 = np.dot(T_70,np.mat(T_76).I)
    R_60 = T_60[0:3,0:3]
    dx,dy,dz = [T_60[0,3],T_60[1,3],T_60[2,3]]

    # solution
    for theta_1 in [math.atan2(dy,dx),math.atan2(dy,dx)+math.pi]:
        q1 = dx*math.cos(theta_1)+dy*math.sin(theta_1)
        q2 = dz - l_1
        AA = -2*l_3*q2-2*l_4*q1
        BB = 2*l_4*q2-2*l_3*q1
        CC  =l_3**2-l_2**2+l_4**2+q1**2+q2**2
        if AA < 0:
            AA = -AA
            BB = -BB
            CC = -CC
        phi = math.atan2(BB,AA)
        aphi_1 = math.asin(-CC/np.hypot(AA,BB))
        if aphi_1 >= 0:
            aphi_2 = math.pi - aphi_1
        else:
            aphi_2 = -math.pi - aphi_1
        a_list = [aphi_1-phi, aphi_2-phi]     
        for A in a_list:
            theta_2a = math.acos((q1-l_4*math.sin(A)-l_3*math.cos(A))/l_2)
            theta_2b = theta_2a + math.pi
            for theta_2,theta_3 in [theta_2a,A-theta_2a],[theta_2b,A-theta_2b]:
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
                                answer.append([theta_1,theta_2-math.pi/2,theta_3,theta_4-math.pi,theta_5-math.pi/2,theta_6+math.pi])
    # delete the repeat solution
    answerr = []
    [answerr.append(i) for i in answer if not i in answerr]
    answer = answerr   
    # check and output
    answer_f = []
    for i in range(len(answer)):
        pose_inv = forward(angle_range(answer[i]))
        pose_inv[3:6] = angle_range(pose_inv[3:6])
        sum = 0
        for j in range(6):
            sum += (pose[j] - pose_inv[j])**2
        if sum < 0.001:
            angle_r = angle_range(answer[i])
            if -math.pi<=angle_r[0]<=math.pi and -1.57<=angle_r[1]<=1.92 and -1.05<=angle_r[2]<=0.94:
                if -math.pi<=angle_r[3]<=math.pi and 0<=angle_r[4]<=math.pi and -math.pi<=angle_r[5]<=math.pi:
                    answer_f.append(angle_r)
    return answer_f

def judge(angle_0,angle_1):
    # the inverse-method has multiple solutions
    # find the least angle-move to get to the aim-point
    sum_list = []
    for i in range(len(angle_1)):
        sum = 0
        for j in range(len(angle_0)):
            sum += (angle_0[j]-angle_1[i][j])**2
        sum_list.append(sum)
    return sum_list.index(min(sum_list))

def talker_pos(q, pub, catch):
    # send the data to the rostopic
    m = angle2motor(q, catch)
    rate = rospy.Rate(freq)
    msg = Float64MultiArray()
    msg.data = m
    pub.publish(msg)
    rate.sleep()
    
    return m

def angle_range(angle):
    # limit angle in (-pi, pi]
    output = []
    for i in angle:
        while i > math.pi:
            i = i - math.pi*2
        while i <= -math.pi + 0.00001:
            i = i + math.pi*2
        output.append(i)
    return output  

def dynamic_tracking(p_a,p_b,v_a,v_b):
    # decomposition
    # vector:s_*  length:ss_*
    s_ab = p_b - p_a

    # s_ab != 0 and v_b != 0
    s_y = projection(s_ab, v_b)
    s_x = s_ab - s_y
    # find the unit vextor of each direction: ref_x,ref_y,ref_z
    if np.linalg.norm(s_y):
        ref_y = s_y / np.linalg.norm(s_y)
    else:
        ref_y = v_b / np.linalg.norm(v_b)

    if np.linalg.norm(s_x):
        ref_x = s_x / np.linalg.norm(s_x)
    else:
        ref_x = np.cross(s_y, s_y+1) / np.linalg.norm(np.cross(s_y, s_y+1))
    ref_z = np.cross(ref_x, ref_y) / np.linalg.norm(np.cross(ref_x, ref_y))

    # decompose v_a to v_x,v_y,v_z
    v_x = projection(v_a, ref_x)
    v_y = projection(v_a, ref_y)
    v_z = projection(v_a, ref_z)

    # scalar
    ss_x = np.inner(s_x, ref_x)
    ss_y = np.inner(s_y, ref_y)
    vv_x = np.inner(v_x, ref_x)
    vv_y = np.inner(v_y, ref_y)
    vv_z = np.inner(v_z, ref_z)
    vv_b = np.inner(v_b, ref_y)

    # x-axis
    ss_x1 = (v_max**2 - vv_x**2 + v_max**2) / (2 * a_max)
    if ss_x1 < ss_x:
        aa_x = a_max
    else:
        if vv_x >= 0:
            ss_x2 = vv_x**2 / (2 * a_max)
        else:
            ss_x2 = -vv_x**2 / (2 * a_max)
        if ss_x2 < ss_x:
            aa_x = a_max
        else:
            aa_x = -a_max
    a_x = ref_x * aa_x
    if (np.inner(v_x, a_x) > 0 and vv_x + aa_x * ts >= v_max) or (vv_x == 0 and ss_x == 0):
        a_x = v_x * 0

    # y-axis
    ss_y1 = (v_max**2 - vv_y**2 + v_max**2 - vv_b**2) / (2 * a_max)
    t1 = (v_max - vv_y + v_max - vv_b) / a_max
    ss_b1 = vv_b * t1
    if ss_y1 < ss_y + ss_b1:
        aa_y = a_max
    else:
        if vv_y > vv_b:
            ss_y2 = (vv_y**2 - vv_b**2) / (2 * a_max)
            t2 = (vv_y - vv_b) / a_max
            ss_b2 = vv_b * t2
        else:
            ss_y2 = -(vv_y**2 - vv_b**2) / (2 * a_max)
            t2 = (vv_b - vv_y) / a_max
            ss_b2 = vv_b * t2
        if ss_y2 < ss_y + ss_b2:
            aa_y = a_max
        else:
            aa_y = -a_max
    a_y = ref_y * aa_y
    if (np.inner(v_y, a_y) > 0 and vv_y + aa_y * ts >= v_max) or (vv_y == vv_b and ss_y == 0):
        a_y = v_y * 0

    # z-axis
    t_z = vv_z / a_max
    if t_z > ts:
        aa_z = -a_max
    else:
        aa_z = -vv_z / ts
    a_z = ref_z * aa_z

    a = a_x + a_y + a_z
    v_a = v_a + a * ts
    p_a = p_a + v_a * ts
    p_b = move(p_b, v_b, ts)

    return p_a,p_b,v_a

def to_aim(p,angle,pub,catch):
    # move arm to the aim-pose p
    first = 1
    t_start = 3
    angle_list_s = inverse(p)
    angle_t_s = angle_list_s[judge(angle,angle_list_s)] 
    for i in list(np.arange(0, t_start, ts)):
        m = talker_pos(angle_t_s, pub, catch)
    pp = np.array(p[0:3])

    return pp, angle_t_s, m

def angle2motor(angle, catch):
    # convert the angle-value to the motor-value
    angle_n = angle * 1
    angle_n[2] += angle_n[1]
    flag = [-1,1,-1,-1,1,-1,1]
    motor_result = []
    for i in range(len(angle_n)):
        motor_tmp = 2048 + flag[i] * angle_n[i] * 2048 / math.pi
        motor_result.append(motor_tmp)
    # choose different catch-angle
    if catch == 1:
        motor_result.append(2748)
    elif catch == 2:
        motor_result.append(3100)
    else:
        motor_result.append(2048)
    return motor_result

def do_catch(p_a, p_b):
    # judge the distance between arm and object
    # if distance is less than 0.001, do the catch
    catch = 0
    if np.linalg.norm(np.array(p_a)-np.array(p_b)) < 0.001:
        catch = 1
    return catch

def drop(color):
    if color == 0:
        p_end = [0, 0.2, 0.15]
    elif color == 1:
        p_end = [0, 0.32, 0.15]
    elif color == 3:
        p_end = [-0.2, 0.2, 0.15]
    else:
        p_end = [-0.3, 0.1, 0.15]

    return p_end


