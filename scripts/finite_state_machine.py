#!/usr/bin/env python

"""
.. module:: finite_state_machine
   :platform: Unix
   :synopsis: Python code that defines the finite state machine for the robot
.. moduleauthor:: Alice Maria Catalano <s5157341@studenti.unige.it>

This is finite state machine that uses the `Smach tool <https://wiki.ros.org/smach>`_ to implement it in ROS. 
It menages the behavior  of a surveillance  robot, that stays mostly in corridors and goes to the rooms just when they turn **urgent** after 7 seconds of not visiting them 

Subscribes  to:
    - /loader a Boolean flag to communicate when the map is totally created.
    - /battery_signal a Boolean flag to communicate when battery is low and when is totally charged

"""
import roslib
import random
import time
import math
import rospy
import smach
import smach_ros
from armor_api.armor_client import ArmorClient
from std_msgs.msg import Bool, Float64
from fsm_patrolling_robot.srv import Get_coordinates
from geometry_msgs.msg import Twist
from control_msgs.msg import JointControllerState

pause_time= 1.5         #global variable for the sleeping time
battery_status = 1      # Battery is charged
urgency_status = 0      # Battery flag for the urgent room
loading = True          # True map is not loaded, False map is loaded
shared_connection = ''  # String resulting from the connectedTo data property
are_urgent=''           # String resulting from the query of the individuals in the URGENT class
robot_position=''       # String that contains always one element, which is the robot position in that moment
timestamp = ''          # String that represent the queried timestamp
coordinates = {}        # Dictionary for the rooms coordinates
process_value=0

def callback(data):
    """ 
    Callback function for the map publisher */loader*, that modifies the value of the **global variable loading** and will let the code start.
    
    """
    global loading
    if data.data == True:
        loading = True

    elif data.data == False:
        loading = False

def callback_batt(data):
    """ 
    Callback function for the map publisher */battery_signal*, that modifies the value of the **global variable battery_status** and will let the code start.
    
    """
    global battery_status
    if data.data == 0:
        battery_status = 0

    elif data.data == 1:
        battery_status =1

def go_to_coordinate(coor):
    """
    The :func:go_to_coordinate function moves the robot to the specified coordinates by calling the get_coordinate service. If the service returns that the target coordinates have been reached, the function returns 1. If the target coordinates have not been reached, the function calls itself again to try moving to the target coordinates again.

    Args:
        - coor: A string representing the key for the desired coordinates in the global dictionary coordinates.
    
    Returns:
        - response.return: An integer indicating whether the target coordinates have been reached (1) or not (0).

    """
    get_coordinate = rospy.ServiceProxy('get_coordinate', Get_coordinates)
    response = get_coordinate(coordinates[coor]['X'] , coordinates[coor]['Y'])
    if response.return_ == 1:
        print('Location reached')
    else:
        print('Location not reached')
        go_to_coordinate(coor)
    return response.return_

def find_path(l1, l2, robPos):
    """
    This function is needed to plan the path from one position to another, recursively  scanning the list of reachable point for the robot and the connected area to the desired position

    Args
        - **l1** the list resulting from querying the *connectedTo* data property of the robot position, returning a list of strings

        - **l2** the list resulting from querying the *connectedTo* data property of the desired location, returning a list of strings

        - **robPos** the robot position in the moment the function is called to check if the robot position appears in the *connectedTo* list of the desired location
    
    Returns
        - **shared_connection** the shared variable that will be checked in the calling function :mod:`change_position`
    """
    global shared_connection

    for i in l1:
        for j in l2:
            if i == j :
                shared_connection = i
            elif robPos == j:
                shared_connection=robPos
            else:
                shared_connection=''
    return shared_connection

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

def urgent_rooms():
    """
    Function that recursively checks on the *are_urgent* list and if the list is empty changes the flag *urgency_status* value to false

    Returns
        - **are_urgent** list of the urgent rooms, to be modified in the :mod:`Room_visiting` status.
    """

    global urgency_status
    global are_urgent

    client = ArmorClient("example", "ontoRef")
    client.call('REASON','','',[''])
    
    urgent_list=client.call('QUERY','IND','CLASS',['URGENT'])
    are_urgent = find_list(urgent_list.queried_objects) 
    print('the urgent rooms in urgency: ', are_urgent)
    if are_urgent == []:
        urgency_status =0
    else:
        urgency_status =1
        
    return are_urgent

def change_position(desPos):
    """
    Function used everytime the Robot has to change position, taking into consideration different behaviors based on the different LOCATION's subclasses

        - **ROOM** if the robot moves to a Room the time stap update is taken into cosideration 
        - **URGENT** the behavior is the same as in the above case 
        - **CORRIDOR** the robot will calculate the path to go into the corridor and advertise the State :class:`Corridor_cruise`
    
    This is the main fuction for motion and takes the help of three other functions :func:`find_list`, :func:`find_path`, :func:`extract_value`;
    all of them used to clean the queried string.
    
    Args

        - **robPos** is the robot position in the moment this function is called

        - **desPos** is the desired position that calls this function
    
    Returns

        - **robot_position** the updated robot position at the end of the location change
    """
    client = ArmorClient("example", "ontoRef")
    client.call('REASON','','',[''])
    req=client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
    robPos=find_individual(req.queried_objects)
    print('change position from: ', robPos, 'to: ', desPos)
    robot_can_reach = client.call('QUERY','OBJECTPROP','IND',['canReach', 'Robot1'])
    robot_can_reach = find_list(robot_can_reach.queried_objects)


    if desPos in robot_can_reach:
        #print('Robot can reach the desired position')
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
        #print('Robot cannot reach the desired position')
        location_connectedTo = client.call('QUERY','OBJECTPROP','IND',['connectedTo', desPos])
        location_connectedTo = find_list(location_connectedTo.queried_objects)
        #print('Desried postion is connected to: ', location_connectedTo)
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
    go_to_coordinate(find_individual(req.queried_objects))
    print('Robot isIn',find_individual(req.queried_objects))
    client.call('REASON','','',[''])

def setup_Dictionary():
    """
    This function initializes the global variable *coordinates* as a dictionary containing 
    the X and Y coordinates for each room in the list `rooms`. The function does this by calling 
    the ArmorClient and using the 'QUERY' command to retrieve the X and Y coordinates for each room 
    from the ontology. The function then adds the room's X and Y coordinates to the coordinates dictionary.

    """
    global coordinates
    client = ArmorClient("example", "ontoRef")
    rooms = ['R1', 'R2', 'R3', 'R4', 'C1', 'C2', 'E']
    for i in rooms:
        req=client.call('QUERY','DATAPROP','IND',['X_point', i])
        X=float(extract_value(req.queried_objects))
        req=client.call('QUERY','DATAPROP','IND',['Y_point', i])
        Y=float(extract_value(req.queried_objects))
        coordinates[i] = {'X': X, 'Y': Y}
    
def room_scan():
    """"
    This function controls the commands passed to the joints. Joint 1 will rotate 360 degrees to scan the room."
    
    """
    # Create a publisher for the cmd_vel topic
    print("Scanning..")
    joint1_pose_pub = rospy.Publisher('/myRob/joint1_position_controller/command', Float64, queue_size=10)
    rospy.Subscriber("/myRob/joint1_position_controller/state", JointControllerState, callback_scan)
    joint1_pose_pub.publish(-3.0)
    while process_value > -2.9:
        joint1_pose_pub.publish(-3.0)
    while process_value < 2.9:
        joint1_pose_pub.publish(3.0)
    joint1_pose_pub.publish(0.0)

def callback_scan(msg):
    global process_value
    process_value = msg.process_value

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

def extract_value(input_list):
    """
    Function to rewrite the queried time stamp or coordinate, deleting the not digit part of the string, for both Rooms and Robot's data property

    Args
        - **list** of queried objects section of the Armor service message
    
    Returns
        all the element between the double quotes
    
    """
    # Extract the string value from the list
    value_string = input_list[0]
    # Remove the surrounding quotation marks and the xsd:float type indicator
    stripped_string = value_string[1:-1].split("^^")[0]
    # Remove any surrounding quotation marks
    stripped_string = stripped_string.strip('"')
    # Convert the string to a float and return it
    return float(stripped_string)

def common_connection(list1, list2):
  for string in list1:
    if string in list2:
        return string

class Load_map(smach.State):

    """
    Class that defines  the *Load_map* state, in which the code waits for the map,
    when the map is received the **loading** boolean variable will be False, the robot will enter in the Corridor and
    the outcome *uploaded_map* will make the state end to switch to the other state *Corridor_cruise*

    Args
        - **smachState** State base interface
    
    Returns
        - **waiting_map** transition condition that will keep this state active
        - **uploaded_map** transition condition that will make this state end and go to the new state

    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['waiting_map','uploaded_map'])
    
    def execute(self, userdata):
        global loading
        client = ArmorClient("example", "ontoRef") 
            
        if loading == True:
            rospy.sleep(pause_time)
            return 'waiting_map'
        else:
            client.call('LOAD','FILE','',['/root/ros_ws/src/fsm_patrolling_robot/maps/my_map.owl', 'http://bnc/exp-rob-lab/2022-23', 'true', 'PELLET', 'false'])
            print(' MAp loaded FSM starts the planning')
            setup_Dictionary()
            go_to_coordinate('E')
            client.call('REASON','','',[''])
            
            return 'uploaded_map'

class Corridor_cruise(smach.State):
    """
    Class that defines  the *Corridor_cruise* state, in which the main robot behavior is described.
    The surveillance  robot will cruise in the corridors until  any room gets *urgent*. The logic of the cruising is basic:
    the robot will start from one corridor, stay in it 2 seconds and change to the other corridor with the function ``change_position(robPos, desPos)``
    Moreover this state gets notified by the :mod:`callback_batt` and the :mod:`callback` which are the callback function of the publishers */loader* and */battery_status*
    
    Args
        - **smachState** State base interface
    
    Returns
        - **battery_low** transition condition that change status going to the *Recharging*
        - **urgent_room** transition condition that will make this state end and go to the new state
        - **stay_in_corridor** loop transition that will keep this state active

    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['battery_low','urgent_room', 'stay_in_corridor'])

    def execute(self, userdata):
        global are_urgent
        global robot_position
        rospy.sleep(pause_time)
        
        client = ArmorClient("example", "ontoRef")
        client.call('REASON','','',[''])
        
        query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robot_position = find_individual(query_position.queried_objects)
        
        
        are_urgent = urgent_rooms()
        print('the urgent rooms are : ', are_urgent )

        if battery_status == 1:
            # The outher condition checks on battery, because that task has the highr priority
            if urgency_status == 0 :
                # The second condition to check on is the urgent rooms
                if robot_position == 'C1' :
                    print('the robot is in C1 should go in C2')
                    rospy.sleep(pause_time)

                    change_position('C2')
                    client.call('REASON','','',[''])
                    
                    return 'stay_in_corridor'
                
                elif robot_position == "C2"  :
                    print('the robot is in C2 should go in C1')
                    rospy.sleep(pause_time)
                    
                    change_position('C1')
                    client.call('REASON','','',[''])
                    
                    return 'stay_in_corridor'
                else:
                    change_position('C1')
                    return 'stay_in_corridor'
            else:
                return 'urgent_room'
        else :
            return 'battery_low'

class Recharging(smach.State):
    """
    Class that defines  the *Recharging* state, which manages the behavior when the battery is low, getting advertised by :mod:`callback_batt`, which will modify the shared variable  *battery_status*
    
    Args
        - **smachState** State base interface
    
    Returns
        - **full_battery** transition condition that changes status, going back to the *Corridor_cruise*
        - **on_charge** transition condition that will keep this state active
    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['on_charge', 'full_battery'])

    def execute(self, userdata):
        global robot_position
        
        client = ArmorClient("example", "ontoRef")
        client.call('REASON','','',[''])
        
        query_position = client.call('QUERY','OBJECTPROP','IND',['isIn','Robot1'])
        robot_position = find_individual(query_position.queried_objects)

        rospy.sleep(pause_time)
        
        if battery_status == 0:
            change_position('E')
            
            return 'on_charge'
        else:
            return 'full_battery'

class Room_visiting(smach.State):
    """
    The class *Room_visiting* is a statethat manages the behavior of a robot when it needs to go to a room. It is activated when the *urgency_status* variable is True.
    It calls the :func:`change_position(robPos, desPos)` function to move the robot. It is also advertised by the :func:`callback_batt in case of low battery.

    Args:
        - **smachState** The base interface for a state.
    
    Returns

        - **battery_low** transition condition that change status going to the *Recharging* status

        - **no_urgent_room** transition condition that specifies that no other room are urgent anymore and changes status

        - **stay_in_room** transition condition that will keep this state active
    """
    
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_urgent_room', 'stay_in_room','battery_low'])
        
    def execute(self, userdata):
        global urgency_status
        global are_urgent
        global robot_position

        client = ArmorClient("example", "ontoRef")
        client.call('REASON','','',[''])

        are_urgent = urgent_rooms()
        
        print('the urgent rooms in room visiting are: ', are_urgent)
        rospy.sleep(pause_time) 
        
        if urgency_status == 0:
            return 'no_urgent_room'
        
        elif battery_status == 0:
            return 'battery_low'
        
        elif urgency_status == 1:
            
            for i in are_urgent :
                random_urgent = random.choice(are_urgent)
                change_position(random_urgent)
                room_scan()
        return 'stay_in_room'

def main():
    """
    This is the main function that initializes the node *finite_state_machine*, create the SMACH state machine and specifies the states with the transitions.
    Also initializes the subscription to the publishers.

    """
    rospy.sleep(pause_time)
    rospy.init_node('finite_state_machine')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['Interface'])
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('LOAD_MAP', Load_map(), 
                               transitions={'uploaded_map':'CORRIDOR_CRUISE', 'waiting_map':'LOAD_MAP'})
        smach.StateMachine.add('CORRIDOR_CRUISE', Corridor_cruise(), 
                                transitions={'battery_low' : 'RECHARGING', 'urgent_room': 'ROOM_VISITING', 'stay_in_corridor' : 'CORRIDOR_CRUISE' })
        smach.StateMachine.add('RECHARGING', Recharging(), 
                                transitions={'on_charge':'RECHARGING', 'full_battery':'CORRIDOR_CRUISE'})
        smach.StateMachine.add('ROOM_VISITING', Room_visiting(), 
                                transitions={'no_urgent_room':'CORRIDOR_CRUISE', 'stay_in_room' : 'ROOM_VISITING', 'battery_low': 'RECHARGING' })
    
        # Execute SMACH plan

    # Execute SMACH plan
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    rospy.Subscriber("/loader", Bool, callback)
    rospy.Subscriber("battery_signal", Bool, callback_batt)
    outcome = sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__': 
    main()
