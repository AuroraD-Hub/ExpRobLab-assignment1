# ExpRobLab Assignment1
**ROS-based architecture for Experimental Robotics Laboratory first assignment.**  
Author: *Aurora Durante*, MS in Robotics Engineering, UNIGE, Genova  
Contact: aurora.durante@coservizi.it

## Introduction
In this assignment a ROS-based software architecture for a robot with surveillance porpouse is defined.  
It is based on the **OWL-DL** approach to create an ontology of the environment and it uses [SMACH](http://wiki.ros.org/smach) to implement a **Finite State Machine**. The ontology is made with Protégé and the architecture behaviour is based on [ARMOR](https://github.com/EmaroLab/armor).

Related documentation on the code of this solution can be found here:  

### Software tools needed
Use [Protégé](https://protege.stanford.edu/) to directly open and read the ontology used here while with [SMACH viewer](http://wiki.ros.org/smach_viewer) it is possible to see how the architecture behaves in run-time.

In order to correctly execute this possible solution, clone the [ARMOR API Client](https://github.com/EmaroLab/armor_py_api) repository from EmaroLab (UNIGE) in the same workspace where this repository is downloaded. Then, copy and paste the following code in the `armor_query_client.py` file in the API `/scripts` folder:
```
def class_of_ind(self, ind, bottom):
        """
        Query which class an individuals belong to.
    
        Args:
            ind(str): an individual of the ontology
            bottom(bool): 
    
        Returns:
            list(str): the class of the individual
    
        Raises:
            armor_api.exceptions.ArmorServiceCallError: if call to ARMOR fails.
            armor_api.exceptions.ArmorServiceInternalError: if ARMOR reports an internal error.
        """
        try:
            res = self._client.call('QUERY', 'CLASS', 'IND', [ind, bottom])
    
        except rospy.ServiceException:
            raise ArmorServiceCallError(
                "Service call failed upon querying individuals belonging to class {0}".format(ind, bottom))
    
        except rospy.ROSException:
            raise ArmorServiceCallError("Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running.")
    
        if res.success:
            return res.queried_objects
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)
```

## Software robot architecture
It is composed of four nodes:
* *state_machine*: it implements every state of the state machine and defines the behaviour of the robot;
* *robot_state*: it simulates stimuli of the robot regarding battery and ontology states;
* *planner*: this servie plans the necessary action that the robot should perform in a specific state and based on the stimuli;
* *controller*: this service executes actions such that the robot moves in the environment.

There is also a *param_name_mapper* interface that collects all the necessary information regarding names of the topics and services and values of parameters used in all the architecture. Moreover, *planner* and *controller* use the ARMOR API Client from EmaroLab.  
In the following components diagram it can be seen how this nodes interact:
![sw_architecture drawio](https://user-images.githubusercontent.com/72380912/204148762-aabe8d49-2ee9-44b8-8e81-11c249e43a5c.png)

### Robot state
The `robot_state` node is a publisher and it simulates stimuli of the robot as battery level and ontology state.
![rs_component drawio](https://user-images.githubusercontent.com/72380912/204150388-4c89ed43-4687-4f3a-9d24-3ca2b2a410bb.png)  
It creates two topics to which it continuosly publishes related messages:
* */state/battery_low*: if the boolean message is *True*, then battery level is low;
* */state/new_ontology*: if the integer message is *1*, then a new ontology has to be loaded.

Simulation of the battery level is defined by a while-loop that modify the boolean value to publish accordingly to a specific delay. This delay is used to simulate both the charging time and battery usage time by setting different values based on topic message published.

### Planner
The `planner` node is a service and it plans the action that the robot should perform.
![plan_component drawio](https://user-images.githubusercontent.com/72380912/204150391-66596e0a-8fbe-4aee-a003-cb514829ae3e.png)  
The `Planner_srv` message is composed as follows:
* Request:
  * *command*: is a string message that defines the action to plan (load the ontology/exit from current location)
* Response:
  * *done*: is a boolean message that notify when the plan is done
  * *reachable_urgent*: is a string list that queries all the reachable locations that the robot should visit as they are urgent
  * *reachable_corridors*: is a string list that queries all the reachable corridors that the robot can move to if there are not urgent rooms

These tasks are performed by the `planner` throught the ARMOR API Client that uses its *utils* and *query* functions for loading the ontology and exiting the location respectively.

### Controller
The `controller` node is a service and it manages changes in the ontology when the robot moves from one location to another.
![control_component drawio](https://user-images.githubusercontent.com/72380912/204150393-c65e3d55-fd9f-465e-84a2-acc1d6e49a73.png)  
The `Controller_srv` message is composed as follows:
* Request:
  * *loc*: is a string message that defines in which location the robot is moving to
* Response:
  * *done*: is a boolean message that notify when the robot moved in the new location

These tasks are performed by the `controller` throught the ARMOR API Client that uses its *umanipulation* functions for updating the information about location, robot and timestamps in the ontology.

### State machine
The `state_machine` node implements the Finite State Machine that manages the behaviour of the robot.  
![sm_component drawio](https://user-images.githubusercontent.com/72380912/204150375-1ccb2775-2958-43b9-bab8-7af429f13fd6.png)  
It subscribes to the two topics created in `robot_state` node and calls the `planner` and `controller` nodes to manipulate the ontology and move in the environment. To do that it uses the custom service requests.

## Software behaviour
include states diagram and eventually temporal diagram

## Installing and running

To execute the assignment run:
`roslaunch assignment1 assignment1.launch`

## Running code explanation

## Working hypothesis

