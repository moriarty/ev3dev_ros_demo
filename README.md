# ev3dev_ros_demo

A quick demo of ev3dev + ros

Requires [Ubuntu 14.04](http://www.ubuntu.com/) + [ev3dev](http://www.ev3dev.org/) + [Ros](http://www.ros.org) & [ev3dev-lang-cpp](https://github.com/ddemidov/ev3dev-lang-cpp):

See my instructions to install all requirments here: https://github.com/moriarty/ros-ev3

In the above instructions, I recommend python because the steps are much easier. This demo is for roscpp + ev3dev-lang-cpp.

## Install Instructions

Most of the steps are detailed in this document: [brickstrap build steps](https://github.com/moriarty/ros-ev3/blob/master/brickstrap-build-status.md), so I will just mention the high level steps.

### Setup Brickstrap
1. Install Ubuntu 14.04. If running windows or OS X, installing in a virtual machine works. (Instructions can be found online)
3. Install Brickstrap (see doc link above)
4. In the Brickstrap environment for your target device (EV3, rpi, rpi2):
  1. Install ```ros_comm``` dependencies
  2. Install ```ros_comm & common_msgs```
  3. Install ```ev3dev-lang-cpp```
      NOTE: Use instructions above, as I provide a patch to the CMakeLists.txt in ev3dev-lang-cpp for easier use with ros.
5. Create SD Card from brickstrap for EV3

### Install ROS on your "normal" computer.

- The EV3 doesn't have enough CPU/RAM to run ```roscore```, so you'll need a computer on your network with ROS installed and roscore running. This can be the virtual machine from above.
- A RaspberryPi/BrickPi with ev3dev can run roscore, see [notes](https://github.com/moriarty/ros-ev3/blob/master/README.md)

### Use your Brickstrap environment for development

```catkin_make``` will run out of memory on the EV3. The quick fix, is develop inside of brickstrap and dump the whole catkin workspace over with FileZilla. Not an elegant solution but it works.
This instructions are standard to [create a catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

1. Open at least one shell to the brickstrap environment created above.
2. Inside of the brickstrap environment:
  1. Source setup.bash for ROS:
    - ```source /opt/ros/indigo/setup.bash```
  2. Change to the robot home directory and create a catkin workspace
    - ```mkdir -p /home/robot/catkin_ws/src```
    - ```cd /home/robot/catkin_ws/src```
    - ```catkin_init_workspace```
  3. Clone this repository into the catkin workspace
    - ```git clone https://github.com/moriarty/ev3dev_ros_demo.git```
    - ```cd /home/robot/catkin_ws```
    - ```catkin_make```

### Copy catkin_workspace to your ev3

I recommend [FileZilla](https://filezilla-project.org/) for this as it's really easy.

1. Install FileZilla in your Ubuntu environment (not brickstrap)
  - ```sudo apt-get update && sudo apt-get install filezilla```
2. Open Filezilla and use the following connection information:
  - Host: ```ev3dev.local``` - IP or network name of your device. For example mine is ```ev3dev.local``` but not all networks are set up to add the ```.local``` domain.
  - Username: ```robot``` - Default user for ev3dev
  - Password: ```maker``` - Currently the default password for ev3dev
  - Port: ```22``` - use SSH File Transfer Protocol.
3. Click Quickconnect, and accept the ssh key box which might pop up.
4. In ```Local site:```
  - Go to where the brickstrap catkin_ws is located. For example:
  - ```/home/alex/brickstrap-workspace/ev3-rootfs/home/robot/catkin_ws/```
5. In ```Remote site:```
  - Go to ```/home/robot``` (FileZilla likely opened this location by default)
6. Drag ```catkin_ws``` from ```Local site``` to ```/robot/home/``` in ```Remote site:```
  - If you already have a ```catkin_ws``` there from following these steps, FileZilla will ask what to do with duplicate files. 

## About the demo

1. All that is needed is a motor in ```Output B```.
2. ```ev3dev_test_node.cpp``` is the source file.
3. ```ev3dev_test_node_cpp``` is the target defined in the CMakeLists.txt
4. ```ev3test_controller``` is the node name when running ```ev3dev_test_node_cpp```
5. ```ev3test_controller```:
  - Listens for the string ```"start"``` or ```"stop"``` on the topic ```/ev3test_controller/cmd```
  - Publishes the current state ```"RUN"``` or ```"IDLE"``` on the topic ```/ev3test_controller/state```

### Running the demo

1. Start roscore on Ubuntu machine:
  - ```roscore```
2. SSH into ev3, source & setup ROS, run ev3dev_test_node_cpp:
  - ```ssh robot@ev3dev.local```
  - ```source /opt/ros/indigo/setup.bash```
  - ```export ROS_MASTER_URI=http://192.168.0.101:11311``` (replace with IP of machine running roscore)
  - run test node, either directly or with ```rosrun```
    - ```./catkin_ws/devel/lib/ev3dev_ros_demo/ev3dev_test_node_cpp``` or
    - ```rosrun ev3dev_ros_demo ev3dev_test_node_cpp``` (rosrun has some overhead and delays start)
3. Back on Ubuntu machine:
  - Check node is running ```rosnode list```
  - Get node info ```rosnode info```
4. Publish a start command:
  - ```rostopic pub /ev3test_controller/cmd std_msgs/String "data: 'start'" ```
5. Publsih a stop command:
  - ```rostopic pub /ev3test_controller/cmd std_msgs/String "data: 'stop'" ```

### rosnode info

Here is the output of rosnode info. As you can see from the hostnames, it's running on ev3dev.local and my desktop is alex-linux-desktop.local where I'm publishing start and stop.

```
$ rosnode info /ev3test_controller 
--------------------------------------------------------------------------------
Node [/ev3test_controller]
Publications: 
 * /ev3test_controller/state [std_msgs/String]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /ev3test_controller/cmd [std_msgs/String]

Services: 
 * /ev3test_controller/get_loggers
 * /ev3test_controller/set_logger_level


contacting node http://ev3dev.local:32963/ ...
Pid: 2244
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /ev3test_controller/cmd
    * to: /rostopic_17467_1460401560340 (http://alex-linux-desktop.local:38400/)
    * direction: inbound
    * transport: TCPROS
```


### rostopic echo

Here is the output of rostopic echo

```
$ rostopic echo /ev3test_controller/state 
data: RUN
---
data: IDLE
---
```

### rostopic pub

```
$ rostopic pub /ev3test_controller/cmd std_msgs/String "data: 'start'" 
publishing and latching message. Press ctrl-C to terminate
^C
$ rostopic pub /ev3test_controller/cmd std_msgs/String "data: 'stop'" 
publishing and latching message. Press ctrl-C to terminate


```
