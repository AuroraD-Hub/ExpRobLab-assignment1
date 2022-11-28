#! /usr/bin/env python

"""
.. module::controller
   :platform: Ubuntu 20.04
   :snyopsis: This module defines the controller of the architecture.

.. moduleauthor::Aurora Durante

Here a service defining the controller of the architecture is created.
It controls robot movements in the environment by manipulating the ontology
through the aRMOR API client by EmaroLab.
"""

import random
import rospy
import time
import param_name_mapper as pnm
from assignment1.srv import Controller_srv, Controller_srvResponse
from armor_api.armor_client import ArmorClient

# A tag for identifying logs producer.
LOG_TAG = pnm.NODE_CONTROLLER

client = ArmorClient('client','assignment')

# An ArmorClient to simulate motion controlling.
def control():
   """
   Controller node initialized.
     
   Here, service controller is instantiated.
   """
   rospy.init_node('controller_node')
   ser = rospy.Service('/control', Controller_srv, handle_control)
   rospy.spin()

def handle_control(req):
   """
   Service callback.
   The controller manages the location changes by manipulating the ontology.
   
   :param req: The request of the service
   :type req: Controller_srv
   """
   rospy.loginfo(pnm.tag_log('Going to new location. Just wait a little...',pnm.NODE_CONTROLLER))
   res = Controller_srvResponse()
   move_to(req)
   time.sleep(5) # used to simulate moving time
   pos = client.query.objectprop_b2_ind('isIn', 'Robot1')
   rospy.loginfo(pnm.tag_log('New location reached! Robot is now in:',pnm.NODE_CONTROLLER))
   pos = split_str(pos)
   res.done = True
   return res
  
def move_to(req):
   """
   The controller change position of the robot in the ontology.
   It updates the new location and the timestamps needed to identify current time and
   last visit to the location.
   
   :param req: The request of the service
   :type req: Controller_srv
   """
   # Retrive current position
   current_pos = client.query.objectprop_b2_ind('isIn', 'Robot1') #current position
   rospy.loginfo(pnm.tag_log('Robot1 was in location:',pnm.NODE_CONTROLLER))
   current_pos = str(split_str(current_pos))
   # Replace new position
   client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', req.loc, current_pos)
   rospy.loginfo(pnm.tag_log(f'Robot1 is going in {req.loc}',pnm.NODE_CONTROLLER))
   client.utils.sync_buffered_reasoner()
   # Retrive current timestamp and last time loc was visited
   current_time = client.query.dataprop_b2_ind('now', 'Robot1') #current time
   current_time = current_time[0][1:11]
   last_visit = client.query.dataprop_b2_ind('visitedAt', req.loc) # last time loc was visited
   last_visit = last_visit[0][1:11]
   # Replace new timestamp and new visit to loc
   client.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(int(time.time())), current_time)
   client.manipulation.replace_dataprop_b2_ind('visitedAt', req.loc, 'Long', current_time, last_visit)
   client.utils.sync_buffered_reasoner() # call the Reasoner
   rospy.loginfo(pnm.tag_log(f'Robot1 moved in location {req.loc}',pnm.NODE_CONTROLLER))
   

def split_str(string):
    """
    Helper function used to get queried object from string.
    
    :param string: The string to split to get information needed
    :type string: string
    """
    length = len(string)
    for i in range(0,length):
       string[i] = string[i][32:-1]
    print(string)
    return string

if __name__ == '__main__':
    # Instantiate the node manager service and wait.
    control()
