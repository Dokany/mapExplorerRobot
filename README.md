# Map Explorer Robot
A dagu equipped with Jetson TX1 board, LiDAR, IMU and Arduino that enters a large squared field sweeping the surrounding unknown environment for map generation and obstacle detection, thus performing the task of map exploration.
![alt text](https://lh6.googleusercontent.com/l4DTV3r0bFu-QosoNq84wZPw5H0TmxTR8eBNkoJl3ot7-ZAVL0Q9M8uu1xqL4I4R9666D8nPgRogx6HotbF_=w1366-h768-rw "Blue")

## How it Works
The dagu starts from an entry point in the field, moves forward till it detects a colored obstacle with the camera, reads the sign on it (right arrow, left arrow or stop circle) and in turn takes the decision of either making a right or left turn (rotation in 90 degrees), or of stopping. The dagu will keep exploring the field till it stops at a stop sign.

# Demo
<a href="http://www.youtube.com/watch?feature=player_embedded&v=uneVWtG7slM
" target="_blank"><img src="http://img.youtube.com/vi/uneVWtG7slM/0.jpg" 
alt="Map Explorer Robot" width="240" height="180" border="10" /></a>

## How We Built It
Using ROS, we created the vision-ky package that runs several nodes responsible for handling different aspects of the project. The package includes the following nodes:
1. IMU_filter.py
2. IMU_reader.py
3. left_motor.py
4. lidar_obstacle.py
5. master_control.py
6. seg.py
7. sign.py
9. wheels.py

## Dependencies
1. [Sweep SDK for the LiDAR](https://github.com/scanse/sweep-ros)
2. [GSCAM](http://wiki.ros.org/gscam)
3. [Jetson Csi Cam](https://github.com/peter-moran/jetson_csi_cam)
4. [PID](http://wiki.ros.org/pid)
```
sudo apt-get install ros-kinetic-pid
```
