#!/usr/bin/env python

"""
.. module:: load_map
   :platform: Unix
   :synopsis: Python code to create the .owl ontology 
.. moduleauthor:: Alice Maria Catalano <s5157341@studenti.unige.it>

ROS Node to create the Ontology map adn initialzie the timestamps for the Rooms, which represents a 2D environment of 4 rooms and 3 corridors

Publishes to:
    - /loader a boolean flag to communicate when the map is totally created.

"""

import random
import rospy
import time
import math
import rospkg
from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool, String
from fsm_patrolling_robot.srv import RoomInformation

motion_status=0
markers_list = []

def extract_aruco(string):
    """
    Call back function that extracts Aruco marker IDs from a string and appends them to the global list of markers if they are not 
    already present.

    Args:
        - String 
    """

    global markers_list
    for word in string.data.split():
        if word.isdigit():
            num = int(word)
            if 10 < num < 18 and num not in markers_list:
                markers_list.append(num)
                print(markers_list)
        

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

def load_std_map():
    """
    Function to initialize the node, to initialize the publisher and to use the `Armor commands <https://github.com/EmaroLab/armor/blob/master/commands.md>`_ to create the ontology.
    It will publish a boolean that will be passed to the state ``Load_map``, advertised by :mod:`load_std_map`

    """
    global markers_list
    print( " Load the standard map", markers_list )
    client = ArmorClient("example", "ontoRef")
    req=client.call('LOAD','FILE','',['/root/ros_ws/src/fsm_patrolling_robot/maps/topological_map.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
    rospy.wait_for_service('/room_info')
    roomInfo_srv = rospy.ServiceProxy('/room_info', RoomInformation)
    individuals = []
    for markers in markers_list: 
        res = roomInfo_srv(markers)
        # getting the informations from the service
        roomID =res.room
        individuals.append(roomID)
        room_X = res.x
        room_Y = res.y

        client.manipulation.add_dataprop_to_ind("X_point", roomID , "float", str(room_X))
        client.manipulation.add_dataprop_to_ind("Y_point", roomID , "float", str(room_Y))
        print(roomID, room_X, room_Y)
    
    #taking the room connections
        for c in res.connections:
            c.through_door
            individuals.append(c.through_door)
            client.call('ADD','OBJECTPROP','IND',['hasDoor', roomID, c.through_door])
            print("the doors are ", individuals)
    
    #set() method is used to convert any iterable to sequence of distinct elements
    client.call('DISJOINT','IND','', list(set(individuals)))
    client.call('REASON','','',[''])
    
    # Initializing the rooms visited_at dataproperty
    for i in list(set(individuals)):
        if i.startswith('R'):
            client.manipulation.add_dataprop_to_ind("visitedAt", i, "Long", str(math.floor(time.time())))
            rospy.sleep(random.uniform(0.0, 1.5))
    client.call('REASON','','',[''])

    #Strat drom room E
    client.call('ADD', 'OBJECTPROP', 'IND', ['isIn', 'Robot1', 'E'])
    client.call('REASON','','',[''])
    
    #Update time property

    rob_time = client.call('QUERY','DATAPROP','IND',['now', 'Robot1'])
    old_rob_time = find_time(rob_time.queried_objects)
    current_time=str(math.floor(time.time()))
    client.call('REPLACE','DATAPROP','IND',['now', 'Robot1', 'Long', current_time, old_rob_time])
   
    client.call('REASON','','',[''])
    
    client.call('SAVE','','',['/root/ros_ws/src/fsm_patrolling_robot/maps/my_map.owl'])

def reader():
"""
    ROS Node to read the data from the subscribed topic '/marker_publisher/target' and extract the markers' ids. 
    When all the markers are detected, the node will shutdown the marker_publisher node, publish a boolean flag to the '/loader' topic 
    and call the function :mod:load_std_map.

    Subscribes to:
        - '/marker_publisher/target' a string containing the detected markers' ids.

    Publishes to:
        - '/loader' a boolean flag to communicate when the map is ready to be built.

"""
    rospy.init_node('loader_node', anonymous=True)
    subscriber=rospy.Subscriber("/marker_publisher/target", String, extract_aruco)
    while not rospy.is_shutdown():
        if len(markers_list)>=7:
            rospy.signal_shutdown('marker_publisher')
            print("markers are all here, build map")
            subscriber.unregister()
            pub = rospy.Publisher('loader', Bool, queue_size=10)
            pub.publish(True)
            load_std_map()
            print("map built")
            pub.publish(False)
            rospy.sleep(2)
            rospy.signal_shutdown('loader')
        
if __name__ == '__main__':

    """
    Entrance point of the code
    """ 

    try:
    	reader()
    except rospy.ROSInterruptException:
    	pass
   