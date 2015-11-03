# katanafun

Using ROS to play with the katana robot. Most code is taken from the [katana_driver](https://github.com/uos/katana_driver) tutorials and altered. 

## Requirements

I've only tested this with indigo. You can get everything from apt-get. No need to build stuff.

```
ros-indigo-desktop-full
ros-indigo-katana
```

## Installation

In your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):

``` 
$ cd ~/catkin_ws/src/
$ git clone git@github.com:tsagi/katanafun.git
$ . ~/catkin_ws/devel/setup.bash
$ cd ~/catkin_ws/
$ catkin_make
```

## Start arm services

### Real katana arm

If you have a katana arm around start it use:

```
roslaunch katana katana.launch
```

### Simulation

If not, there is a gazebo simulation.

```
roslaunch katana_arm_gazebo katana_arm.launch
```

## Dance
Right now there is only one launch file and it is kind of lame. 
The arm tries to form the YMCA letters and then it goes back to the starting position.

To launch it:

```
roslaunch katanafun ymca.launch
```

***Warning:*** If you do this on a real arm, make sure the arm workspace is empty. There is no object avoidance programmed here.
