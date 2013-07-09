SICK to SICK Lidar Calibration
==============================

Justin Cosentino, Mili Shah, and Roger Eastman  
National Institute of Technology  
July, 2013  

Background 
----------

This repository contains code and data used to calibrate two 3D real-time laser range sensors. Using a specially designed target that allows for the determination of a lidar's full 6DOF pose and the sicktoolbox, we find the coordinate change from one SICK LMS 2xx to another by determining the apex of said target and then using a least-squares fitting algorithm to determine the optimal transformation between each range-finder. This code is currently under development, and is being used to gather statistical data to prove our methods. Upon completing said research, a paper will be submitted to either ICRA or CPVR that details our methods and the target design. 

File System
-----------

There are three main directories within this repository: Data, Code, and sicktoolbox-1.0.x. All data collected from this project is stored within the subdirectories of Data. These subdirectories contain either the raw laser scan data (overflow data was not recorded, removing the need for background subtraction), the average of the raw data from each scan, and the calculated apex of each scan. There are additional directories contained within these three data types, appearing in the format:

    YYYYMMDD_TRIAL-NUM

This directories signify the date and trial from which the data was collected. Within these directories exist the actual data files, stored as comma-separated values (csv). These files appear in the format:

    LIDAR-NUM_pose_POSE-NUM

Raw data is stored as matrix A of X and Y data from k scans such that

    A = [scan1x ; scan1y; scan2x ; scan2y ; ... ; scankx ; scanky].

Average data is stored as matrix A of X-hat and Y-hat, such that all the X and Y data from the k scans has been averaged and shot data has been ignored. This appears in the format 

    A = [avg-x ; avg-y].

Lastly, Apex data is stored as a point P such that:

    P = [x ; y ; z=0]
    
and P represents the apex point relative to the lidar's coordinate frame (since we assume the lidar sits at the origin, z will always equal 0). The Matlab code used to retrieve scan data, calculate the apex of the target, and optimize a homogenous transformation is stored with the Code directory. See each file for more specific descriptions of their functionality. The SICK to Matlab bridge used was the sicktoolbox, which can be downloaded from http://sicktoolbox.sourceforge.net.

Usage
-----

In order to gather data, first connect both SICK lidars to the computer via serial-to-USB cables. Upon connecting both devices, power on the lidars and then run 

    dmesg
    
to find the path of each lidar. An example path would be '/dev/ttyUSB0'. Once both paths have been determined, select a baud rate at which to run both lidars. Although it is possible to run SICK lasers at 500k, our devices are being run at 38400bps. The sicktoolbox specs used are as follows:

    Sick Type: Sick LMS 200-30106
    Scan Angle: 180 (deg)
    Scan Resolution: 0.5 (deg)
    Measuring Mode: 8m/80m; 3 reflector bits
    Measuring Units: Centimeters (cm)

Once the lasers have been connected, place the target such that both wings and each side are within view of each lidar. At this point, run the following command:

    generate_data('/dev/ttyUSB1','/dev/ttyUSB5',38400,30,1,1)

which is of the form

    generate_data('LIDAR_PATH_1','LIDAR_PATH_2',BAUD_RATE,NUM_SCANS,TRIAL_NUM,POSE_NUM)

This function will collect data and store it in the aforementioned directories. 
