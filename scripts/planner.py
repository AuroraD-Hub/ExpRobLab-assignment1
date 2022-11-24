#! /usr/bin/env python

"""
.. module::planner
   :platform: Ubuntu 20.04
   :snyopsis: This module contains architectures parameters.

.. moduleauthor::Aurora Durante

Here Planner Node is defined.
It plans actions based on which command the state machine is giving and calls
the Controller Node to execute it.
"""

import random
import rospy
import param_name_mapper as pnm
from assignment1.srv import Planner_srv
from armor_api.armor_client import ArmorClient

# A tag for identifying logs producer.
LOG_TAG = pnm.NODE_PLANNER

client = ArmorClient('client','assignment')

# An ArmorClient to simulate motion planning.
# It takes in input "command", which is a string defining what action the client have to do, and 
# "item", which is a Boolean value or an empty string based on which command is given.     
def plan():
     """
     Planner node initialized.
     
     Here, service planner is instantiated.
     """
     rospy.init_node('planner_node')
     ser = rospy.Service('/plan', Planner_srv, handle_planner)
     rospy.spin()
     
def handle_planner(req):
    """
    Service callback.
    
    Arguments:
    -----
    req: Plan
    
    The planner executes two different action based on what the state machine needs:
     1) *load*: it loads an available ontology
     2) *exit*: it exits from the current location
    """
    if req.command == 'load': # load the ontology
       rospy.loginfo(pnm.tag_log('Loading ontology...',pnm.NODE_PLANNER))
       pub_load = rospy.Publisher(pnm.TOPIC_LOAD_ONTOLOGY, Bool, queue_size=1, latch=True)
       load_ontology(pub_load, req)
       rospy.loginfo(pnm.tag_log('Ontology loaded!',pnm.NODE_PLANNER))
       client.utils.sync_buffered_reasoner() 
    elif req.command == 'exit': # exit from a location
       client.utils.sync_buffered_reasoner() 
       rospy.loginfo(pnm.tag_log('Getting reachable locations...',pnm.NODE_PLANNER))
       exit_from_loc()
       rospy.loginfo(pnm.tag_log('Reachable locations information retrived!',pnm.NODE_PLANNER))
    res.done = True
   
def load_ontology(self, pub_load, req):
     """
     The planner loads the ontology.
     
     Arguments:
     -----
     pub_load
       It is the publisher to the topic '/state/new_ontology'
     req: Plan
       It is the client request object to the service
     """
     # Retrive ontology file position 
     path = os.path.dirname(os.getcwd())
     path = path + "/ontology/assignment1.owl"
     # Load it and mount on reference
     client.utils.load_ref_from_file(path, "http://bnc/exp-rob-lab/2022-23",True, "PELLET", True, False)
     client.utils.mount_on_ref()
     # Publish that there's no new ontology to load
     ont = not req.item
     pub_load.publish(ont)
     # Disjoint individuals of classes
     client.manipulation.disj_inds_of_class('LOCATION')
     client.manipulation.disj_inds_of_class('ROOM')
     client.manipulation.disj_inds_of_class('CORRIDOR')
     client.manipulation.disj_inds_of_class('URGENT')
     client.manipulation.disj_inds_of_class('DOOR')
      
def exit_from_loc(self):
     """
     The planner queries all reachable locations.
     
     It defines which location are URGENT and which are just CORRIDORS and give the in response
     """
     # Retrive reachable locations
     reachable_locs = client.query.objectprop_b2_ind('canReach', 'Robot1') # reachable locations
     rospy.loginfo(pnm.tag_log('Reachable locations are:',pnm.NODE_PLANNER))
     reachable_locs = split_str(reachable_locs)
     # Divide location in URGENT and CORRIDOR
     res.reachable_corridors = ''
     res.reachable_urgent = ''
     dim = len(reachable_locs)
     for i in range(0,dim):
       cls = client.query.class_of_ind(reachable_locs[i],"false")
       rospy.loginfo(pnm.tag_log(f'{reachable_locs[i]} belong to classes:',pnm.NODE_PLANNER))
       cls = split_str(cls)
       if 'URGENT' in cls:
         res.reachable_urgent = res.reachable_urgent+','+reachable_locs[i]
       elif 'CORRIDOR' in cls:
         res.reachable_corridors = res.reachable_corridors+','+reachable_locs[i]
     res.reachable_corridors = reachable_corridors.split(',')[1:len(reachable_corridors)]
     res.reachable_urgent = reachable_urgent.split(',')[1:len(reachable_urgent)]

def split_str(string):
    """
    Helper function used to get queried object from string.
    
    Arguments:
    -----
    string: str
      String obtained from aRMOR API client.
    """
    length = len(string)
    for i in range(0,length):
       string[i] = string[i][32:-1]
    print(string)
    return string

if __name__ == '__main__':
    # Instantiate the node manager class and wait.
    plan()
