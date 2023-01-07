#! /usr/bin/env python

"""
.. module:: get_coordinates
   :platform: Unix
   :synopsis: Script to get the rooms coordinates
.. moduleauthor:: Alice Maria Catalano <s5157341@studenti.unige.it>

ROS Service to move the robot to a desired position specified in the request.

Service:
    - name: /get_coordinate 
    - request containing the desired x and y position

Service response:
    - indicating whether the robot reached the target position or not

"""

import rospy
from fsm_patrolling_robot.srv import Get_coordinates
import actionlib
from move_base_msgs.msg import *
from actionlib_msgs.msg import *

def callback(request):
    """
    
    Callback function for the `/get_coordinate` service. Recives as input the desired coordinates and uses the Action client for `move_base`.
    
    """
    x = request.x
    y = request.y
    print('going to: ', x, y)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) #start movebaseaction
    print('subscribed')
    client.wait_for_server() #waiting for response
    
    #create the goal
    goal = MoveBaseGoal()
    #set the goal parameter
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.orientation.w = 1
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    #send the goal
    client.send_goal(goal)
    #wait for result
    wait = client.wait_for_result(timeout=rospy.Duration(100.0))
    if not wait:
        #target not reached, calling cancle goal and return
        print("Mission failed, robot didn't reach goal")
        client.cancel_goal()
        return -1
    return 1 #if reahced the target

def get_coordinates():
    """
    Initializes the `/get_coordinate` service and the `getCoordinate` node.
    """
    rospy.init_node('getCoordinate') #seting up the node
    service= rospy.Service('get_coordinate', Get_coordinates, callback)
    print("service ready")
    rospy.spin()

if __name__=="__main__":
    get_coordinates()
