#!/usr/bin/env python3

import rospy 
import random
import math
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState 
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import GetModelStateRequest

def main():
    rospy.init_node('set_pose')

    random.seed()
    x_pos = [-7.5, 7.5]
    y_pos = [-7.5, 7.5]
    w_pos = [1, 1/math.sqrt(2), 0, -1/math.sqrt(2), 1]
    setx = x_pos[random.randint(0,1)]
    sety = y_pos[random.randint(0,1)]
    if setx > 0 and sety > 0:
        setw = w_pos[random.randint(0,1)]
    elif setx < 0 and sety > 0:
        setw = w_pos[random.randint(1,2)]
    elif setx < 0 and sety < 0:
        setw = w_pos[random.randint(2,3)]
    else:
        setw = w_pos[random.randint(3,4)]

    state_msg = ModelState()
    state_msg.model_name = 'turtlebot3_waffle'
    state_msg.pose.position.x = setx
    state_msg.pose.position.y = sety
    state_msg.pose.position.z = 0.05
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 1 - math.pow(setw,2)
    state_msg.pose.orientation.w = setw
    state_msg.reference_frame = 'world'

    model = GetModelStateRequest()
    model.model_name = 'turtlebot3_waffle'

    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = get_state(model)
        print (resp)
        rate = rospy.Rate(10)
        while not resp.success:
            resp = get_state(model)
            print (resp)
            rate.sleep()

        
    except rospy.ServiceException as e:
        print ("Service call failed:" + e)


    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
        print (resp)

    except rospy.ServiceException as e:
        print ("Service call failed:" + e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
