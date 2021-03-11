#!/usr/bin/env python
from func import *
import time
from para import *

running = False
pub = None

def go(data):
    global running, pub
    # we use para 'running' to make sure that only after an object has been caught, does the next catch begin.
    if running == True:
        return
    running = True

    # data contains x-axis pos and color information.
    pos = data.data[0]
    color = data.data[1]

    angle_0 = [0,0,0,0,0,0]
    pose_0 = forward(angle_0)

    p_a = pa_0
    # set the catch-pos, if color is purple, adjust the catch-height
    p_b = np.array([(pos-1)*0.01,-0.19,0.14])
    if color == 4 or color == 2:
        p_b[2] = 0.12
    # initial velocity of robot arm
    v_a = np.array([0]*3)

    first_time = 1
    flag = 0
    count = 0
    # each circle send arm-pos once
    for i in list(np.arange(0, t_max, ts)):
        # use the velocity-decomposition algorithm
        p_a,p_b,v_a = dynamic_tracking(p_a, p_b, v_a, v_b)
    
        pose = list(p_a)
        pose.extend(pose_0[3:6])
        angle_list = inverse(pose)
        # angle_t is the angle each joint will move
        angle_t = angle_list[judge(angle_0,angle_list)]
        
        catch = do_catch(p_a, p_b)
        
        if first_time and catch:
            flag = 1
            first_time = 0

        if color == 4 and flag:
            flag = 2
        
        # flag is used to distinguish different aim-object
        if flag:
            count += 1
            # set the tracking time to be 1s(20*ts)
            # track and catch
            if count > 20:
                m = talker_pos(angle_t, pub, flag)
                angle_0 = angle_t
                if count == 40:
                    break
            # just track
            else:
                m = talker_pos(angle_t, pub, 0)
                angle_0 = angle_t
        # just track
        else:
            m = talker_pos(angle_t, pub, 0)
            angle_0 = angle_t

    time.sleep(0.1)
    # go to aim_pos_1
    p_start = [-0.2, 0.2, 0.3]
    p_start.extend(pose_0[3:6])
    p_a, angle_0, motorr = to_aim(p_start, angle_0, pub, 1)
    time.sleep(3)

    # get the different drop position
    p_end = drop(color)
    p_end.extend(pose_0[3:6])
    # move to aim-position and drop
    p_a, angle_0, motor = to_aim(p_end, angle_0, pub, 1)
    msg = Float64MultiArray()
    motor[6] = 2048
    msg.data = motor
    pub.publish(msg)
    time.sleep(1)
    
    # move back to initial position
    p_start = [0.182959, 0, 0.386151]
    p_start.extend(pose_0[3:6])
    p_a, angle_0, motorr = to_aim(p_start, angle_0, pub, 0)
    
    running = False

def main():
    global pub
    rospy.init_node("controller")
    # send the pos-info to the robot arm
    pub = rospy.Publisher('/fobot_joint_position_controller',Float64MultiArray, queue_size=1000)
    # get the position info of object-to-be-caught from camera
    # once we get infomation, execute function : go
    sub = rospy.Subscriber('cv_position_publisher', Float64MultiArray, queue_size=100, callback=go)
    while rospy.is_shutdown() == False:
        rospy.spin()

if __name__ == '__main__':
    main()
