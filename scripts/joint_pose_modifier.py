#!/usr/bin/env python
"""
.. module:: joint_pose_modifier
   :platform: Unix
   :synopsis: Script to move the arm
.. moduleauthor::Alice Maria Catalano <s5157341@studenti.unige.it>

	The purpose of this script is to control the joint of a robot and move it to a desired position. It does this by publishing commands 
	to the topic "myRob/joint1_position_controller/command" and subscribing to the topic "myRob/joint1_position_controller/state" to monitor the joint's position. This allows the script to issue successive commands based on the current position of the robot's joint.


"""
# Import necessary libraries
import rospy
from control_msgs.msg import JointControllerState
from std_msgs.msg import Float64


process_value=0

def joint_action():
    # Create ROS node
    """"
    This function controls the commands passed to the joints. Joint 1 will rotate 360 degrees to scan the room."
    
    Raises: 
        - ROSInterruptException
    
    """
    rospy.init_node('joint_pose_modifier', anonymous = True)
    joint1_pose_pub = rospy.Publisher('/myRob/joint1_position_controller/command', Float64, queue_size=10)
    rospy.Subscriber("/myRob/joint1_position_controller/state", JointControllerState, callback)
    joint1_pose_pub.publish(-3.14)
    
    while process_value > -3.14:
        joint1_pose_pub.publish(-3.14)
    while process_value < 2.5:
        joint1_pose_pub.publish(2.5)
    joint1_pose_pub.publish(0.0)

def callback(msg):
    """
    Callback function for the joint position.
    
    Args:
        - JointControllerState 
    """
    global process_value
    process_value = msg.process_value

if __name__ == '__main__':
    try:
        joint_action()
    except rospy.ROSInterruptException:
        pass
  
