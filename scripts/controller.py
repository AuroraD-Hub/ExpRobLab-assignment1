#! /usr/bin/env python

import random
import rospy
import param_name_mapper as pnm
import controller_srv
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
   rospy.init_node('controller')
   ser = rospy.Service('/control', controller_srv, handle_control)
   rospy.spin()

def handle_control(req):
   """
   Service callback.
   
   Arguments:
   -----
   req: Plan
   
   The controller manages the location changes by manipulating the ontology.
   """
   rospy.loginfo(pnm.tag_log('Going to new location...',pnm.NODE_CONTROLLER))
   go_to(req.loc)
   rospy.loginfo(pnm.tag_log('New location reached!',pnm.NODE_CONTROLLER))
   res.done = True
  
def go_to(self, req):
   """
   The controller change position of the robot in the ontology.
   
   It updates the new location and the timestamps needed to identify current time and
   last visit to the location.
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
    control()
