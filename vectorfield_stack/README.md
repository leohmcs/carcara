# vectorfield_stack
This packages were adapted from vectorfield_stack (Navigation control algorithms based on artificial vector fields)


![image](https://github.com/adrianomcr/vectorfield_stack/blob/main/images/field_illustration.png)

The ROS stack provides packages to perform the navigation control of a robot by using an artificial vector field strategy. The image above illustrates the method. Given a desired reference path, the vector field guides the robot towards the curve, which is then followed. The core of the navigation control is a vector field based on the Euclidean distance function, specific packages import the field and perform the control of different robot types.

More information about the stack in: [vectorfield_stack](https://github.com/adrianomcr/vectorfield_stack)

## Sarc-Barinet Simulation

The drones initiate in formation at a high altitude as in the airplane release. Each of them are commanded to depart in different directions and start the search. Search is made by following a spiral trajectory which covers the area until one of them detects the fire. Once fire is detected the swarm reunites above the center of the fire.  There is a wait of 5 seconds in formation and then the swarm starts to performe a joint movement following a closed ellipse curve starting at the center of the fire. Once the swarm is about to conclude the turn, they release the fire-retardant liquid and head to the pre-defined location (here defined as one of the cornes of the map).

The group decided to simulate and test the trajectory in Rviz separeted from the fire detection algorithm due to limitation of resources to operate using the MRS environment. Thereby, robotsim package contains a script called vision which simulates the operation of a sensor. 

## How to use

Clone this repository in your workspace

```bash
$ git clone https://github.com/JoBaiao/carcara
```

Compile and source

```bash
$ catkin build carcara
$ source (YOUR WORKSPACE PATH)/devel/setup.bash
```

For the simulated example with 1, 3 or 5 drones respectively launch one of the following

```bash
$ roslaunch examples drone1.launch
$ roslaunch examples drone3.launch
$ roslaunch examples drone5.launch
```

To run a simulation with 25 drones, launch the following (it's recommended to have better computational resources)

```bash
$ roslaunch examples drone25.launch
```
