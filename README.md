# ExpRobLab Assignment1
**ROS-based architecture for Experimental Robotics Laboratory first assignment.**  
Author: *Aurora Durante*, MS in Robotics Engineering, UNIGE, Genova  
Contact: aurora.durante@coservizi.it

## Introduction
In this assignment a ROS-based software architecture for a robot with surveillance porpouse is defined.  
It is based on the **OWL-DL** approach to create an ontology of the environment and it uses [SMACH](http://wiki.ros.org/smach) to implement a **Finite State Machine**. The ontology is made with Protégé and the architecture behaviour is based on [ARMOR](https://github.com/EmaroLab/armor).

Related documentation on the code of this solution can be found here:  
https://aurorad-hub.github.io/ExpRobLab-assignment1/

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
* *planner*: this service plans the necessary action that the robot should perform in a specific state and based on the stimuli;
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
The state machine is composed of three states: *Charging*, *RandomMoving* and *Waiting*. They are depicted in the following state diagram with the corresponding transitions:  
![state_diagram drawio](https://user-images.githubusercontent.com/72380912/204153843-b1b1b539-1923-48b4-88cc-0e3f4968dc13.png)  
*Charging* state is the starting one of the architecture in which the robot waits in location E for the battery to get fully charged while loading all the information about the ontology. It has two outcomes:
* *battery_low*: state machine stays in *Charging* until `/state/battery_low` topic informs it that battery is fully charged;
* *ready*: state machine goes in *RandomMoving* state as soon as battery is charged.  

In *RandomMoving* state the robot moves randomly in the environment staying mainly in CORRIDORs rather then in ROOMs untill a URGENT location is reachable. It has three outcomes:
* *ready*: state machine stays in *RandomMoving* until the robot is moving between CORRIDORs that are not URGENT;
* *battery_low*: state machine goes in *Charging* if `/state/battery_low` topic informs it that battery is low;
* *goal_reached*: state machine goes in *Waiting* state as soon as a URGENT location is reached.  

In *Waiting* state the robot has reached a URGENT location and, thus, it has to stay there for some time. It has two outcomes:
* *battery_low*: state machine goes in *Charging* if `/state/battery_low` topic informs it that battery is low;
* *time_out*: state machine goes in *RandomMoving* state as soon as waiting time has elapsed.  

## Installing and running
This architecture is based on ROS Noetic and is developped in the Docker environment provided to UNIGE Robotics Engineering students.  
Once that instructions in [Software tools needed section](#software-tools-needed) have been followed, do the following steps:
+ clone this repository in your workspace and source it
+ go in the `script` folder of this repository and run `chmod +x *`
+ go back in your workspace and run `catkin_make`

To run the assignment open a terminal and execute:  
```
roslaunch assignment1 assignment1.launch
```  
Every node will display its log messages in different terminals while the ARMOR Server for the API Client will be automatically launched and a new window displaying the running state diagram with SMACH Viewer will open.

## Running code explanation

## Working hypothesis

