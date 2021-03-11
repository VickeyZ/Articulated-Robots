#!/usr/bin/env python
import rospy,math
import numpy as np
from std_msgs.msg import Float64MultiArray


angle = [0,0,0,0,0,0]
# angle = [0.1,0.5,0.3,0.8,0.5,0.2]



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

def main():
    T = np.identity(4)
    for i in range(6):
        T_tmp = np.array([math.cos(theta[i]),-math.cos(dh[i][1])*math.sin(theta[i]),
                    math.sin(dh[i][1])*math.sin(theta[i]),dh[i][0]*math.cos(theta[i]),
                    math.sin(theta[i]),math.cos(dh[i][1])*math.cos(theta[i]),
                    -math.sin(dh[i][1])*math.cos(theta[i]),dh[i][0]*math.sin(theta[i]),
                    0,math.sin(dh[i][1]),math.cos(dh[i][1]),dh[i][2],0,0,0,1]).reshape(4,4)
        T = np.dot(T,T_tmp)
    result = np.dot(T,x_p)
    print('xyz',result[0],result[1],result[2])

    # rpy
    roll = math.atan2(T[2,1],T[2,2])
    pitch = -math.asin(T[2,0])
    yaw = math.atan2(T[1,0],T[0,0])
    print('rpy',roll,pitch,yaw)


def talker():
    pub = rospy.Publisher('/probot_anno/arm_pos_controller/command',Float64MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data = angle
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
