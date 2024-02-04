# LEGO Food Tester
----------------------
In this repository, we keep a system to run a device we created with LEGO to measure the texture of food. The system integrates a PC, an Arduino, and a LEGO Mindstorms EV3 into a single system that can control motors using a GUI and check weights obtained from load cells.

## Required equipment
- PC  (OS:  Ubuntu20.04LTS  )
- Arduino
- Loadcell  ( 5kg )
- XFW-HX711
- LEGO Mindstorms EV3
- LEGO Mindstorms EV3 Large Servo Motor(x2)
- Micro SD card (SDHC) 8GB or more

## Setup of PC
### Install Arduino IDE
Explain omitted
### Install Visual Studio Code
Explain omitted
### Install Python modules
``` Bash
sudo apt update
sudo apt install python3-pip
pip3 install matplotlib numpy paho-mqtt
```
### Install MQTT Client
``` Bash
sudo apt install mosquitto-clients
```
### Install ROS Noetic
- Follow the URL below to install the software.
    - http://wiki.ros.org/noetic/Installation/Ubuntu
### Install rosserial dependent packages
``` Bash
sudo apt install ros-noetic-rosserial
sudo apt install ros-noetic-rosserial-python
sudo apt install ros-noetic-rosserial-arduino
```
### Setup of ros_lib
#### Generation of ros_lib
``` Bash
rosrun rosserial_arduino make_libraries.py [Arduino library paths]
```
#### Fixes to ros_lib/ros/msg.h
- cstring -> string.h
- std::memcpy -> memcpy

## Setup of LEGO Mindstorms EV3
### Setup
- Setup according to the following URL
    - https://www.ev3dev.org/docs/getting-started/#step-2-flash-the-sd-card
### Install Python modules
- Connecting PC and EV3
- Remote access to EV3 via ssh (PC side)
    ``` Bash
    ssh robot@ev3dev.local
    ```
    The password is `maker`
- Install dependency packages
    ``` Bash
    sudo apt update
    sudo apt install python3-pip
    sudo apt-get install mosquitto-clients
    ```
- Install Python module
    ``` Bash
    pip3 install paho-mqtt
    ```
### Transfer Python script to EV3
If you do not see `run` in Visual Studio Code, it is possible to run the Python script by transferring it as follows (because EV3 does not support `UTF-8`).
#### Process：
- Connect PC and EV3
- Remote access to EV3 via ssh (PC side)
    ``` Bash
    ssh robot@ev3dev.local
    ```
- Creation of directory(EV3 side)
    ```
    mkdir ev3_ws
    ```
- Transfer Python script with the scp command.
    ``` Bash
    scp ./ev3/lego_food_tester_for_ev3.py robot@ev3dev.local:~/ev3_ws
    ```
- Run Python script(EV3 side)
    ``` Bash
    python3 ~/ev3_ws/lego_food_tester_for_ev3.py
    ```

### How to run LEGO Food Tester System
#### PC
- Connect PC and EV3
- Connect PC and Arduino
- Terminal1
    - Remote access to EV3 via ssh (PC side)
        ``` Bash
        ssh robot@ev3dev.local
        ```
    - Run Python script(EV3 side)
        ``` Bash
        python3 ~/ev3_ws/lego_food_tester_for_ev3.py
        ```
- Terminal2
    ```Bash
    roscore
    ```
- Terminal3
    ```Bash
    cd ~/catkin_ws
    source devel/setup.bash
    roslaunch lego_food_tester_pkg bringup.launch
    ```
- Terminal4
    ```Bash
    cd ~/catkin_ws
    source devel/setup.bash
    roslaunch lego_food_tester_pkg control.launch
    ```
