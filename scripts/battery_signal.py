#!/usr/bin/env python

"""
.. module:: battery_signal
   :platform: Unix
   :synopsis: Python code to randomly change the battery level
.. moduleauthor:: Alice Maria Catalano <s5157341@studenti.unige.it>

ROS Node to pblish the battery level, keeping it charget for a random period of time and keeping it in charge for 10 seconds

Publishes to:
    - /battery_signal a boolean flag to communicate when battery is low and when is totally charged

"""

import rospy
import random
from armor_api.armor_client import ArmorClient
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool

timeTo_recharge = 10 #sec

def cancel_goal():
    """
    Cancels any existing goals using the Action client for the `move_base` action server
 
    """
    # Action client
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # Wait for the action server 
    client.wait_for_server()
    client.cancel_all_goals()

def battery_signal():
    """
    
    Initializes the battery node and publishes a boolean value to the `battery_signal` topic indicating the battery status.
    When the battery is low (`battery_status` is 0), the function cancels any existing goals and waits for `timeTo_recharge` seconds before 
    setting the `battery_status` to 1 and continuing to publish.
    
    Pram:
        - Publisher to the `battery_signal`
        - battery_status: Flag indicating the battery status (0 for low, 1 for charged)
        - time_charged: Random duration for which the battery is charged
   
   """
    
    pub = rospy.Publisher('battery_signal', Bool, queue_size=10)
    rospy.init_node('battery_signal_node', anonymous=True)
    
    while not rospy.is_shutdown():
        
        battery_status = 1
        pub.publish(battery_status)
        time_charged =  random.uniform( 200, 500 )
        rospy.sleep(time_charged)
        battery_status = 0
        print("Battery is low, please charge the robot")
        cancel_goal()
        pub.publish(battery_status)
        rospy.sleep(timeTo_recharge)
        battery_status = 1
        pub.publish(battery_status)

if __name__ == '__main__':
    try:
        battery_signal()
    except rospy.ROSInterruptException:
        pass

