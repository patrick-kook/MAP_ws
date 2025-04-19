## Map Controller Extension

This project is based on the `map_control` module from the ForzaETH repository.

An additional script, `map_controller_obs.py`, was developed to extend the functionality:  
When static obstacles are detected, the system switches to a locally generated path using a spline-based planner to safely avoid them.

## Installation

Clone the repository into your ROS workspace and build it:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

git clone https://github.com/patrick-kook/MAP_ws

cd ..
catkin_make
```

## Running the Simulation

After building the workspace, you can run the simulation using:

```bash
roslaunch map_controller sim_MAP.launch
```
