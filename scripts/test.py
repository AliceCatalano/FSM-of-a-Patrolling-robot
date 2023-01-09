#!/usr/bin/env python
import roslib
import random
import math
import time
import rospy
import rospkg
import smach
import smach_ros
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from armor_api.armor_client import ArmorClient
from geometry_msgs.msg import Twist

def find_individual(list):
    """
    Function to rewrite the queried *isIn* Robot's object property, extracting the element and saving it in the returned string

    Args
        - **list** of queried objects section of the Armor service message
        
    Returns
        - **location** the string containing the location name in which the Robot is in
    
    """
    for i in list:
        if "R1" in i:
            return 'R1'
        elif "R2" in i:
            return'R2'
        elif "R3" in i:
            return'R3'
        elif "R4" in i:
            return'R4'
        elif "C1" in i:
            return'C1'
        elif "C2" in i:
            return'C2'
        elif "E" in i:
            return'E'
def find_time(list):
    timestamp = ''
    """
    Function to clean the queried time stamp for both Rooms and Robot's data property.

    Args:
        - *list* the list of queried objects section of the Armor service message.
    Returns:
        all the element between the double quotes
    
    """
    for i in list:
        for element in range(1, 11):
            timestamp=timestamp+i[element]
     
    return timestamp
def find_list(list):
    """
    Function to rewrite the queried *connectedTo* data property list, extracting each element and saving it in the returned list as separate strings

    Args
        - **list** of queried objects section of the Armor service message

    Returns
        - **position_list** the list of strings of locations
    
    """
    position_list = []
    for i in list:
        
        if "R1" in i:
            position_list.append('R1')
        elif "R2" in i:
            position_list.append( 'R2')
        elif "R3" in i:
            position_list.append('R3')
        elif "R4" in i:
            position_list.append( 'R4')
        elif "C1" in i:
            position_list.append('C1')
            
        elif "C2" in i:
            position_list.append('C2')
        elif "E" in i:
            position_list.append( 'E')
    return position_list

def common_connection(list1, list2):
  for string in list1:
    if string in list2:
        return string



def change_position(desPos):
    client = ArmorClient("example", "ontoRef")
    client.call('REASON','','',[''])
    req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
    robPos=find_individual(req.queried_objects)
    print('change position from: ', robPos, 'to: ', desPos)
    robot_can_reach = client.call('QUERY','OBJECTPROP','IND',['canReach', 'Robot1'])
    robot_can_reach = find_list(robot_can_reach.queried_objects)
    print('robot_can_reach: ', robot_can_reach)


    if desPos in robot_can_reach:
        print('Robot can reach the desired position')
        #Update isIn property
        client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1',desPos,robPos])
        client.call('REASON','','',[''])
        
        #Update now data property
        req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
        client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', str(math.floor(time.time())), find_time(req.queried_objects)])
        client.call('REASON','','',[''])
        
        #Update visitedAt data property
        isRoom = client.call('QUERY','CLASS','IND',[desPos, 'true'])
        if isRoom.queried_objects == ['URGENT'] or isRoom.queried_objects == ['ROOM']:
            req=client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
            client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', str(math.floor(time.time())), find_time(req.queried_objects)])
            client.call('REASON','','',[''])



    else:
        print('Robot cannot reach the desired position')
        location_connectedTo = client.call('QUERY','OBJECTPROP','IND',['connectedTo', desPos])
        location_connectedTo = find_list(location_connectedTo.queried_objects)
        print('Desried postion is connected to: ', location_connectedTo)
        common=common_connection(location_connectedTo,robot_can_reach)
        if common == None:
            print('Robot cannot reach the desired position')
            client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1',robot_can_reach[0],robPos])
            client.call('REASON','','',[''])
            req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
            robPos=find_individual(req.queried_objects)
            robot_can_reach = client.call('QUERY','OBJECTPROP','IND',['canReach', 'Robot1'])
            common=common_connection(location_connectedTo,find_list(robot_can_reach.queried_objects))

        
        client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1',common,robPos])
        client.call('REASON','','',[''])
        req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robPos=find_individual(req.queried_objects)
        client.call('REPLACE','OBJECTPROP','IND',['isIn', 'Robot1',desPos,robPos])
        client.call('REASON','','',[''])
        #Update now data property
        req=client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
        client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', str(math.floor(time.time())), find_time(req.queried_objects)])
        client.call('REASON','','',[''])
        #Update visitedAt data property
        isRoom = client.call('QUERY','CLASS','IND',[desPos, 'true'])
        if isRoom.queried_objects == ['URGENT'] or isRoom.queried_objects == ['ROOM']:
            req=client.call('QUERY','DATAPROP','IND',['visitedAt', desPos])
            client.call('REPLACE','DATAPROP','IND',['visitedAt', desPos, 'Long', str(math.floor(time.time())), find_time(req.queried_objects)])
            client.call('REASON','','',[''])

    req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
    print('Robot isIn',find_individual(req.queried_objects))
    req=client.call('REASON','','',[''])
    #print(req)




def main():
    client = ArmorClient("example", "ontoRef")
    client.call('LOAD','FILE','',['/root/ros_ws/src/fsm_patrolling_robot/maps/my_map.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
    while True:
        to_go=input("where to go? (R1, R2, R3, R4, C1, C2, E)")
        change_position(to_go)

if __name__ == '__main__': 
    main()
