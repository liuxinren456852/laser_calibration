SICK to SICK Lidar Calibration
==============================

Justin Cosentino, Mili Shah, and Roger Eastman
National Institute of Technology
July 9th, 2013

Background 
----------

This repository contains code and data used to calibrate two 3D real-time laser range sensors. Using a specially designed target that allows for the determination of a lidar's full 6DOF pose and the sicktoolbox, we find the coordinate change from one SICK LMS 2xx to another by determining the apex of said target and then using a least-squares fitting algorithm to determine the optimal transformation between each range-finder. This code is currently under development, and is being used to gather statistical data to prove our methods. Upon completing said research, a paper will be submitted to either ICRA or CPVR that details our methods and the target design. 

File System
-----------

