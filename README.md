# Pick and Place
![robot](https://github.com/BrunoEduardoCSantos/Pick-and-Place/blob/master/misc_images/pickobject.PNG)
## Goal

The project goal is compute inverse kinematics for a 6 degree-of-freedom robotic arm in ROS in order to pick an object in the shelf and place it in a cilinder. 

## Setup
Make sure you install [ROS](http://www.ros.org/) on a Ubuntu 16.04 machine.

### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/BrunoEduardoCSantos/Pick-and-Place
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/Pick-and-Place/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```


In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the**spawn_location** argument in `target_description.launch` file under /Pick-and-Place/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot

## Implementation 

For further details on Kinematics implementation and performance of algorithm check out : [writeup](https://github.com/BrunoEduardoCSantos/Pick-and-Place/blob/master/writeup_template.md).
Check [IK_server](https://github.com/BrunoEduardoCSantos/Pick-and-Place/blob/master/kuka_arm/scripts/IK_server.py) to find more details about Inverse Kinematics implementation.


# Disclamer

This project was build from [Robo-ND-Pick-and-place project](https://github.com/udacity/RoboND-Kinematics-Project) in the context of [Robotic Software Engineer Nanodegree](https://in.udacity.com/course/robotics-nanodegree--nd209)
