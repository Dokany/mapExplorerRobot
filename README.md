# Map Explorer Robot
A dagu equipped with Jetson TX1 board, LiDAR, IMU and Arduino that enters a large squared field sweeping the surrounding unknown environment for map generation and obstacle detection, thus performing the task of map exploration.

![alt text](https://lh3.googleusercontent.com/K4NFOGF_W8HdMejaXT-gIYBQM-Kk-eDXzCM9H4pH9UoosNK3w8Y5WZ-3_VJuq5fpbr46SiO_xEWw6a2It50WjPc5wJKY8UJgAEBITblfb7w8dnRhapsGBqrrERdTihg64ELRzHw_-JnWdk_pHSmY8KLQK4FIzgNX5p9VyS2lv3m1bQY9OO67Gh_2GahwfFVbJ2XtnnXGZWwd2aSOrT9fl93r3eJtSrjlRuE6o84hu93WPEgwOpqCsILf3pmSGnxePKc_TyJQyclSCS1CgY3gIuoqwB1iQrU21y6eQTC9vFtlmj6CgKZnXdiRDnX6tP-GfGyQSjy-XiVaX6zS2fTFwCoHO_flxERODufagusjH1fxQityjb5r-peLDHj17yqur2tf2pHWSKV1o3lmcqiiz7febHJO1C4hJ7eZpvJOnA3r5gzoCWSsS8zkD9yJq_BOXjTeCL5ZLDP0ODR-LFm4xza7pwdarBlJoqfssr784xhY4yKZ_Fq2yvy0IsKrVs8in0dTohNZ0hClOhecDgNub1fQbJOM_CLgvHb8cpyVc4frctDcxSDZa1NQaaPK-D0K_iYDg0ak1dK6UshFRhZySrqy8riuxi3OD_PrcXZVmNu3lasKGSpHCprXhWB3A0OLeXeFa6MAHRkjv0JsTZW4dRZB_ZNissrG=w1024-h768-no "Blue")

## How it Works
The dagu starts from an entry point in the field, moves forward till it detects a colored obstacle with the camera, reads the sign on it (right arrow, left arrow or stop circle) and in turn takes the decision of either making a right or left turn (rotation in 90 degrees), or of stopping. The dagu will keep exploring the field till it stops at a stop sign.

## Demo
<a href="http://www.youtube.com/watch?feature=player_embedded&v=uneVWtG7slM
" target="_blank"><img src="http://img.youtube.com/vi/uneVWtG7slM/0.jpg" 
alt="Map Explorer Robot" width="720" height="480" border="10" /></a>

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
## Project Team
- [Aley Baracat](https://github.com/alybaracat)
- [John Sourour](https://github.com/johnsourour)
- [Karim Atwa](https://github.com/karimatwa)
- [Sara Seleem](https://github.com/saraseleem)
- [Yasmin ElDokany](https://github.com/Dokany)

#### Project submitted for the CSCE 432/4301 Embedded Systems course, at the American University in Cairo.
