# GUIDANCE-SDK-ROS-UART 

This is a DJI Guidance ROS node for UART connection, based on the DJI [uart_example](https://github.com/dji-sdk/GuidanceSDK/tree/master/examples/uart_example).

For USB connection the repo can be found on [Guidance-SDK-ROS](https://github.com/dji-sdk/Guidance-SDK-ROS).

# How to use
1. Clone the repo to the catkin workspace source directory `catkin_ws/src` and then 

```
cd ~/catkin_ws/src
git clone this
cd .. &&	catkin_make
```
2. Execution with $roslaunch, by specifying your UART connection port.

For example,

```
roslaunch guidance_uart uart_port.launch uart_port:= "/dev/ttyTHS2" 
```
or by rosrunning the node, with the default UART port, on "/dev/ttyTHS2" (UART3),
```
rosrun guidance_uart guidanceNodeUART
```

# Data Publishers - Topics

There are five Node publishers, 

* /guidance_uart/imu
* /guidance_uart/velocity
* /guidance_uart/ultrasonic
* /guidance_uart/obstacle_avoidance
* /guidance_uart/motion
    
