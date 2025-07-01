# lcm_tools
This is a rosbag for lcm data connection for njust_v2_self_dog.

## How to use
- Download [lcm v1.5.0](https://github.com/lcm-proj/lcm/tree/v1.5.0) and install it.
- Change the network card name **enp88s0** in **star_multicast.sh**;
- Using **star_multicast.sh** to ensure that the lcm of the current host is in broadcast mode;
- Start using the launch file and add the **RobotModel** to rviz for display.
```
source devel/setup.bash
./star_multicast
roslaunch lcm_tools njust_display_rviz.launch 
```

## More detail
- mortors state lcm Type is `motor_state_second_dog`. User can use **lcm-spy** to view the current lcm channel status.