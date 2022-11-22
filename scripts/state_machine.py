#!/usr/bin/env python

"""
.. module::state_machine
   :platform: Ubuntu 20.04
   :snyopsis: This module represents the state machine of the architecture.

.. moduleauthor::Aurora Durante

The state machine is composed of three states:
 1) Charging
 2) Random_move
 3) Wait
All of them refers to the Planner node to execute specific commands.
"""

import roslib
import rospy
import smach
import smach_ros
import time
import random
import param_name_mapper as pnm
import planner as Plan
import controller as Control
import planner_srv as PlanRequest
import controller_srv as ControlRequest
from os.path import dirname, realpath
from std_msgs.msg import Bool
from armor_api.armor_client import ArmorClient

#client = ArmorClient('client','assignment')
plan_client = rospy.ServiceProxy('/planner', Plan)
control_client = rospy.ServiceProxy('/control', Control)
planner = PlanRequest()
controller = ControlRequest()

class Charging(smach.State):
    """
    This models state 'Charging'.

    Outcomes:
    ------
    ready:
      Robot goes in RandomMoving state.
    low_battery:
      Robot stays in Charging state.

    Here, the robot is in location 'E' and is waiting some time to richarge its battery or to load 
    a new ontology based on Boolean variables 'battery_low' and 'ont_needed'.
    """

    def __init__(self):
        """
        Here, Charging state is initialized.
    
        Parameters:
        -------
        ont_needed : bool
            Define if a new ontology is needed to be loaded.
        battery_low : bool
            Define if battery level of the robot is low.
       
        The parameters are initialized to later obtain corresponding values from the Robot State Node
        """
        smach.State.__init__(self, outcomes=['ready','low_battery'])
        self.ont_needed = Bool()
        self.battery_low = Bool()
        rospy.loginfo(pnm.tag_log('Charging state',pnm.NODE_STATE_MACHINE))
            
    def execute(self, userdata):
      """
      Here subscribers to topics 'state/new_ontology' and 'state/battery_low' are defined.
    
      From the subscribers current boolean value of *ont_needed* and *battery_low* are obtained.
      Based on these values, corresponding action from the 'planner' or 'controller' node are defined.
      """
      # load ontology if needed
      pub_load = rospy.Publisher(pnm.TOPIC_LOAD_ONTOLOGY, Bool, queue_size=1, latch=True)
      sub_load = rospy.Subscriber(pnm.TOPIC_LOAD_ONTOLOGY, Bool, self.ontology_state)
      if self.ont_needed:
        #self.load_ontology(pub_load)
        planner.req.command = 'load'
        planner.req.item = self.ont_needed
        plan_client(planner)
      client.utils.sync_buffered_reasoner() # call the Reasoner
      
      # recharge battery if needed
      sub_battery = rospy.Subscriber(pnm.TOPIC_BATTERY_LOW, Bool, self.battery_level)
      if self.battery_low:
        rospy.loginfo(pnm.tag_log('Battery has to recharge',pnm.NODE_STATE_MACHINE))
        #rospy.sleep(5)
        controller.req.loc = 'E'
        control_client(controller)
        return 'low_battery'
      else:
        rospy.loginfo(pnm.tag_log('Battery is fully charged',pnm.NODE_STATE_MACHINE))
        #self.exit_from_E()
        planner.req.command = 'exit'
        plan_client(planner)
        return 'ready'
    
    # Definition of helper functions: 
    def ontology_state(self, data):
      """
      Load ontology subscriber callback function.
    
      Arguments:
      ------
      data : struct
        Retrive state of the ontology.
    
      It retrives the boolean value from Robot State Node representing if a new ontology is needed to be load.
      """
      self.ont_needed = data.data
         
    def battery_level(self,data):
      """
      Battery level subscriber callback function.
    
      Arguments:
      ------
      data : struct
        Retrive state of the battery.
    
      It retrives the boolean value from Robot State Node representing if the battery level is low.
      """
      self.battery_low = data.data
              
#    def load_ontology(self, pub_load):
#      path = dirname(dirname(realpath(__file__)))
#      path = path + "/ontology/assignment_map.owl"
#      client.utils.load_ref_from_file(path, "http://bnc/exp-rob-lab/2022-23", True, "PELLET", True, False)
#      client.utils.mount_on_ref()
#      self.ont_needed = not self.ont_needed
#      pub_load.publish(self.ont_needed)
#      rospy.loginfo(pnm.tag_log('Ontology loaded!',pnm.NODE_STATE_MACHINE))
#      client.manipulation.disj_inds_of_class('LOCATION')
#      client.manipulation.disj_inds_of_class('ROOM')
#      client.manipulation.disj_inds_of_class('CORRIDOR')
#      client.manipulation.disj_inds_of_class('URGENT')
#      client.manipulation.disj_inds_of_class('DOOR')
#      client.manipulation.disj_inds_of_class('IND')
#      client.manipulation.disj_inds_of_class('ROBOT')
      
#    def exit_from_E(self):
#      # Retrive reachable locations
#      reachable_locs = client.query.objectprop_b2_ind('canReach', 'Robot1') # reachable locations
#      reachable_locs = split_str(reachable_locs)
#      # Chose a location which is a CORRIDOR
#      reachable_corridors = ''
#      dim = len(reachable_locs)
#      for i in range(0,dim):
#        cls = client.query.class_of_ind(reachable_locs[i], "false")
#        cls = split_str(cls)
#        if 'CORRIDOR' in cls:
#          reachable_corridors = reachable_corridors+','+reachable_locs[i]
#      reachable_corridors = reachable_corridors.split(',')[1:len(reachable_corridors)]
#      cor = random.choice(reachable_corridors)
#      rospy.loginfo(pnm.tag_log(f'Going in location {cor}',pnm.NODE_STATE_MACHINE))
#      self.move_to(cor)
      
#    def move_to(self, cor):
#      # Replace new location
#      pos = client.query.objectprop_b2_ind('isIn', 'Robot1')
#      print(pos)
#      client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', cor , 'E')
#      client.utils.sync_buffered_reasoner() # call the Reasoner
#      pos = client.query.objectprop_b2_ind('isIn', 'Robot1')
#      print(pos)
#      # Get timestamps
#      current_time = client.query.dataprop_b2_ind('now', 'Robot1') #current time
#      current_time = current_time[0][1:11]
#      last_visit = client.query.dataprop_b2_ind('visitedAt', 'E') # last time E was visited
#      last_visit = last_visit[0][1:11]
#      # Replace new timestamp and new visit to E
#      client.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), current_time)
#      client.manipulation.replace_dataprop_b2_ind('visitedAt', 'E', 'Long', current_time, last_visit)
#      client.utils.sync_buffered_reasoner() # call the Reasoner
#      rospy.loginfo(pnm.tag_log(f'Moved in location {cor}',pnm.NODE_STATE_MACHINE))
      

# state: RANDOM_MOVING
class RandomMoving(smach.State):
    """
    This models state 'RandomMoving'.

    Outcomes:
    ------
    goal_reached:
      Robot goes in Waiting state.
    low_battery:
      Robot goes in Charging state.

    Here, the robot is randomly choosing a corridor to move to. If it find a reachable location which is URGENT, 
    then it calls Planner Node to move to it.
    Moreover, whenever battery level is low, it calls Planner Node to go to location 'E' to recharge the battery.
    """

    def __init__(self):
        """
        Here, RandomMoving state is initialized.
    
        Parameters:
        -------
        battery_low : bool
           Define if battery level of the robot is low.
       
        The parameters are initialized to later obtain corresponding values from the Robot State Node
        """
        smach.State.__init__(self, outcomes=['goal_reached', 'low_battery']) #add outcome 'ready'? It allows to remove while loop
        self.battery_low = Bool()
        rospy.loginfo(pnm.tag_log('RandomMoving state',pnm.NODE_STATE_MACHINE))
        
    def execute(self, userdata):
        """
        Subscribers to topic 'state/battery_low' is defined.
    
        It implements a 'while loop' in which:
         1) From the subscriber it obtaines current boolean value of 'battery_low' and calls the Planner 
            Node to go in Charging state if battery is low.
         2) Defines whether there are URGENT location to visit and calls Planner Node accordingly to go in Waiting state, otherwise
            it moves in a randomply chosen CORRIDOR.
        """
        client.utils.sync_buffered_reasoner() # call the Reasoner
        sub_battery = rospy.Subscriber(pnm.TOPIC_BATTERY_LOW, Bool, self.battery_level)
        while not rospy.is_shutdown():
          if self.battery_low: # battery level is low
            rospy.loginfo(pnm.tag_log('Battery has to recharge',pnm.NODE_STATE_MACHINE))
            #self.go_to('E') # robot moves in E
            control.req.loc = 'E'
            control_client(control)
            return 'low_battery'
#          # Find reachable locations
#          reachable_locs = client.query.objectprop_b2_ind('canReach', 'Robot1') # reachable locations
#          reachable_locs = split_str(reachable_locs)
#          # Look if a loc is URGENT, otherwise choose a location which is a CORRIDOR
#          reachable_corridors = ''
#          reachable_urgent = ''
#          dim = len(reachable_locs)
#          for i in range(0,dim):
#            cls = client.query.class_of_ind(reachable_locs[i],"false")
#            cls = split_str(cls)
#            if 'URGENT' in cls:
#              reachable_urgent = reachable_urgent+','+reachable_locs[i]
#            elif 'CORRIDOR' in cls:
#              reachable_corridors = reachable_corridors+','+reachable_locs[i]
#          reachable_corridors = reachable_corridors.split(',')[1:len(reachable_corridors)]
#          reachable_urgent = reachable_urgent.split(',')[1:len(reachable_urgent)]
          planner.req.command = 'exit'
          plan_client(planner)
          if len(planner.res.reachable_urgent)>0:
            loc = random.choice(planner.res.reachable_urgent)
          else:
            loc = random.choice(planner.res.reachable_corridors)     
          # Move in the location 
          controller.req.loc = loc
          control_client(controller)
#          rospy.loginfo(pnm.tag_log(f'Going in location {loc}',pnm.NODE_STATE_MACHINE))
#          self.go_to(loc)
          return 'goal_reached'
                    
    # Definition of helper function:
    def battery_level(self, data):
      """
      Battery level subscriber callback function.
    
      Arguments:
      ------
      data : struct
        Retrive state of the battery.
    
      It retrives the boolean value from Robot State Node representing if the battery level is low.
      """
      self.battery_low = data.data
                
#    def go_to(self, loc):
#      # Retrive current position
#      current_pos = client.query.objectprop_b2_ind('isIn', 'Robot1') #current position
#      current_pos = str(split_str(current_pos))
#      path = dirname(dirname(realpath(__file__)))
#      path = path + "/ontology/prova.owl"
#      client.utils.save(path)
#      pos = client.query.objectprop_b2_ind('isIn', 'Robot1')
#      print('0: ', pos)
#      # Replace new position
#      client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', loc, current_pos)
#      #client.manipulation.remove_objectprop_from_ind('isIn', 'Robot1', current_pos)
#      client.utils.sync_buffered_reasoner()
#      client.utils.save(path)
#      pos = client.query.objectprop_b2_ind('isIn', 'Robot1')
#      print('1: ', pos)
#      #client.manipulation.add_objectprop_to_ind('isIn', 'Robot1', loc)
#      #client.utils.sync_buffered_reasoner() # call the Reasoner
#      #client.utils.save(path)
#      #pos = client.query.objectprop_b2_ind('isIn', 'Robot1')
#      #print('2: ', pos)
#      # Retrive current timestamp and last time loc was visited
#      current_time = client.query.dataprop_b2_ind('now', 'Robot1') #current time
#      current_time = current_time[0][1:11]
#      last_visit = client.query.dataprop_b2_ind('visitedAt', loc) # last time loc was visited
#      last_visit = last_visit[0][1:11]
#      # Replace new timestamp and new visit to loc
#      client.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), current_time)
#      client.manipulation.replace_dataprop_b2_ind('visitedAt', loc, 'Long', current_time, last_visit)
#      client.utils.sync_buffered_reasoner() # call the Reasoner
#      rospy.loginfo(pnm.tag_log(f'Moved in location {loc}',pnm.NODE_STATE_MACHINE))

# state: WAIT
class Waiting(smach.State):
    """
    This models state 'Waiting'.

    Outcomes:
    ------
    time_out
      Robot goes in RandomMoving state
    low_battery
      Robot goes in Charging state

    Here, the robot is waiting an amount of time in the current location.
    Moreover, whenever battery level is low, it calls Planner Node to go to location 'E' to recharge the battery.
    """

    def __init__(self):
        """
        Here, Waiting state is initialized.
    
        Parameters:
        -------
        battery_low : bool
           Define if battery level of the robot is low.
       
        The parameters are initialized to later obtain corresponding values from the Robot State Node
        """
        smach.State.__init__(self, outcomes=['time_out', 'low_battery'])
        self.battery_low = Bool()
        rospy.loginfo(pnm.tag_log('Waiting state',pnm.NODE_STATE_MACHINE))

    def execute(self,userdata):
        """
        Subscriber 'state/battery_low' is defined.
    
        From the subscriber current boolean value of 'battery_low' is obtained. Based on these values, 
        corresponding action from the Planner Node are defined.
        Otherwise, it waits an amount of time and then calls Planner Node to exit from current location.
        """
        client.utils.sync_buffered_reasoner() # call the Reasoner
        sub_battery = rospy.Subscriber(pnm.TOPIC_BATTERY_LOW, Bool, self.battery_level)
        if self.battery_low: # battery level is low
          rospy.loginfo(pnm.tag_log('Battery has to recharge',pnm.NODE_STATE_MACHINE))
          #self.go_to('E') # robot moves in E
          controller.req.loc = 'E'
          control_client(controller)
          return 'low_battery'
        else:  # robot waits in the room
          rospy.sleep(pnm.WAITING_TIME)
          # Exit from the room and update timestamps
          #self.exit_from_loc()
          planner.req.command = 'exit'
          plan_client(planner)
          return 'time_out'
          
    # Definition of helper function:
    def battery_level(self, data):
      """
      Battery level subscriber callback function.
    
      Arguments:
      ------
      data : bool
        Retrive state of the battery.
    
      It retrives the boolean value from Robot State Node representing if the battery level is low.
      """
      self.battery_low = data.data
    
#    def exit_from_loc(self):
#      # Retrive reachable locations
#      reachable_locs = client.query.objectprop_b2_ind('canReach', 'Robot1') # reachable locations
#      reachable_locs = split_str(reachable_locs)
#      # Chose a location which is a CORRIDOR
#      reachable_corridors = ''
#      dim = len(reachable_locs)
#      for i in range(0,dim):
#        cls = client.query.class_of_ind(reachable_locs[i],"false")
#        cls = split_str(cls)
#        if 'CORRIDOR' in cls:
#          reachable_corridors = reachable_corridors+','+reachable_locs[i]
#      reachable_corridors = reachable_corridors.split(',')[1:len(reachable_corridors)]
#      cor = random.choice(reachable_corridors)
#      rospy.loginfo(pnm.tag_log(f'Exiting from location {cor}',pnm.NODE_STATE_MACHINE))
#      self.go_to(cor)
      
#    def go_to(self, loc):
#      # Retrive current position
#      current_pos = client.query.objectprop_b2_ind('isIn', 'Robot1') #current position
#      current_pos = split_str(current_pos)
#      # Replace new position
#      client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', loc , current_pos)
#      client.utils.sync_buffered_reasoner() # call the Reasoner
#      # Retrive current timestamp and last time loc was visited
#      current_time = client.query.dataprop_b2_ind('now', 'Robot1') #current time
#      current_time = current_time[0][1:11]
#      last_visit = client.query.dataprop_b2_ind('visitedAt', loc) # last time loc was visited
#      last_visit = last_visit[0][1:11]
#      # Replace new timestamp and new visit to loc
#      client.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), current_time)
#      client.manipulation.replace_dataprop_b2_ind('visitedAt', loc, 'Long', current_time, last_visit)
#      client.utils.sync_buffered_reasoner() # call the Reasoner
#      rospy.loginfo(pnm.tag_log(f'Moved in location {loc}',pnm.NODE_STATE_MACHINE))
      

def main():
    """
    The state machine is initialized and started.

    SMACH is used to create the state machine. It has three states and transitions between
    one another are defined.
    The Introspection Server is also cretated for visualization purpouse.
    At last, the state machine is executed and it runs untill the application is stopped.
    """
    rospy.init_node(pnm.NODE_STATE_MACHINE, log_level=rospy.INFO)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('CHARGING', Charging(),
                               transitions={'ready':'RANDOM_MOVING',
                                            'low_battery':'CHARGING'})
        smach.StateMachine.add('RANDOM_MOVING', RandomMoving(),
                               transitions={'goal_reached':'WAITING',
                                            'low_battery':'CHARGING'})
        smach.StateMachine.add('WAITING', Waiting(),
                               transitions={'time_out':'RANDOM_MOVING',
                                            'low_battery':'CHARGING'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ASSIGNMENT1')
    sis.start()
    
    # Initialize clients to planner and controller
    rospy.wait_for_service('/planner')
    rospy.wait_for_service('/controller')

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

#def split_str(string):
#    length = len(string)
#    for i in range(0,length):
#       string[i] = string[i][32:-1]
#    print(string)
#    return string
    

if __name__ == '__main__':
    main()
