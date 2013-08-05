Lidar Calibration for Use with Mobile Robotics
====================================

Justin Cosentino, Mili Shah, and Roger Eastman  
National Institute of Technology  
Summer, 2013  

Background 
----------------
This repository contains code and data used to calibrate two 3D real-time 
laser range sensors. Using a specially designed target that allows for the
determination of a lidar's full 6DOF pose, we find the coordinate change 
from one SICK LMS 2xx to another by determining the apex of said target 
in various poses and then using a least-squares fitting algorithm to
determine the optimal transformation between each range-finder. This code
is currently under development and is being used to gather statistical
data to prove our methods. Upon completing said research, a paper will be
submitted to either ICRA or CPVR that details our methods and the target
design.

Dependencies
--------------------
- Matlab, which may be purchased from http://www.mathworks.com/products/matlab/.  
- The sicktoolbox, which may be downloaded from http://sicktoolbox.sourceforge.net. View documentaion for installtion instructions. The usb-to-serial adapters used to connect the lidar to the computer are described within the sicktoolbox documentation.
- Specifically designed winged-target
- This guide assumes that one is using Ubuntu 12.04 LTS

Installation
---------------
After installing Matlab and the sicktoolbox, simply clone this repository 
and add the code directory to your Matlab path. At this point, the programs
may be run through Matlab. See the Usage Section for specific instructions on how to use this package.

File System
----------------
Within this repository there are three directories: Data/, Code/, and Figures/. The Data directory contains apex calculations, scan averages, and raw scan data in the sub-directories Apex/, Average/, and Raw/. There exist sub-sub-directories that contain calculations and data for specific lidar transformations within each of the three sub-directories. This sub-sub-directories contain a README that details the ground truth related to the given transformation, as well as various pose directories named in the following format:

    YYYYMMDD_POSE-NUM

such that the data was taken on MM DD, YYYY and corresponds to the POSE-NUM pose of the target. Within these directories there exist 
actual data files, stored as comma-separated values (CSV). The naming 
convention is as follows:

    LIDAR-NUM_pose_SCAN-NUM

Each file corresponds to the SCAN-NUM scan of the lidar LIDAR-NUM for the given pose. For each pose, there should exist five data files per lidar. Here is an example of how data would be stored:

    Data  
    --| Apex  
    ------| transform_1  
    ----------| 20130805_1  
    --------------| l1_pose_1  
    --------------| l2_pose_2  
    --------------| ...  
    --------------| l1_pose_5  
    --------------| l1_pose_5  
    ----------| 20130805_2  
    ----------| ...  
    ----------| 20130805_20  
    ------| transform_2  
    ------| transform_3  
    --| Average  
    ------| ...  
    --| Raw  
    ------| ...  
  
Raw data is stored as matrix A of X and Y data from k scans such that

    A = [scan_1x ; scan_1y; scan_2x ; scan_2y ; ... ; scan_kx ; scan_ky].

Average data is stored as matrix A of X-hat and Y-hat, such that all the X
and Y data from the k scans has been averaged and data that falls outside 
of one standard deviation of the mean has been ignored. This appears in the
format 

    A = [avg_x ; avg_y].

Lastly, Apex data is stored as a point P such that:

    P = [x ; y ; z]
    
and P represents the apex point relative to the lidar's coordinate frame. 

The Matlab code used to retrieve scan data, calculate the apex of the 
target, and optimize a homogenous transformation is stored with the Code/ 
directory. See each file for more specific descriptions of their 
functionality. The Figure/ file contains figures and images used within my presentation.

Usage
---------
First, power on the two SICK LMS 2xx lidars. Once the lidars have booted
up, connect them to the computer via serial-to-USB adapters and run  

    dmesg
    
to find the path of each lidar. An example path would be '/dev/ttyUSB0'.
Once both paths have been determined, select a baud rate at which to run
both lidars. Our tests have been conducted at the 500k baud rate. The 
sicktoolbox specs used are as follows:

    Sick Type: Sick LMS 200-30106
    Scan Angle: 180 (deg)
    Scan Resolution: 0.5 (deg)
    Measuring Mode: 8m/80m; 3 reflector bits
    Measuring Units: Centimeters (cm)

Upon connecting the lidars, place the winged target in front of the SICKs
such that each face of the target is within view of the lidars. In order
to collect data, run the following command: 

    generate_data('/dev/ttyUSB0','/dev/ttyUSB1', 500000, 30, 1, true, transformation_name)

which is of the form

    generate_data(DEV_PATH_1, DEV_PATH_2, BAUD, SCANS, TEST, WRITE_FLAG, TRANSFORMATION_NAME)

This collects raw data, averages the raw data accounting for shot data, 
segments the data, fits lines to the segmented data, and calculated the
apex of the target. If the write_flag is set to true, the raw, average,
and apex data will be saved within the following files, respectively:

    ~/Documents/laser_calibration/Data/Raw/transformation_name/.
    ~/Documents/laser_calibration/Data/Average/transformation_name/.
    ~/Documents/laser_calibration/Data/Apex/transformation_name/.

Within these directories data will be saved in the aforementationed format. 

Once apex data has been collected, one can calculate the 
optimal transformation between the two lidars using:

    calculate_r_t(n, transformation_name)

where n is the max number of apexes and transformation_name is the name of the directory in which the cu

Note:  
- Background subtraction can be changed within the generate_scan function.
Currently, the lidar ignores all data outside of a given region, so merely update
the mask defined within this function to fit a range in which the target may be
found.

- The write_data function assumes that data will be written to 
~/Documents/laser_calibration/Data/*. If you wish to save the data to another 
location, simply change the paths within this function. 

There exists functions that read all average data within the Average directory
and calculate and save new apex data. This can be used to recalculate apex data
after a change to line fitting or data segmentation.

Example Usage
-------------
- Postition lidars
- Power on lidars
- Connect lidars to computer
- Collect apex data, changing the target's pose after each run

    generate_data('/dev/ttyUSB0','/dev/ttyUSB1', 500000, 30,  1, true, transformation_name)
    generate_data('/dev/ttyUSB0','/dev/ttyUSB1', 500000, 30,  2, true, transformation_name)
    ...
    generate_data('/dev/ttyUSB0','/dev/ttyUSB1', 500000, 30, 20, true, transformation_name)

- Calculate the transformation between the two lidars, determining the optimal R and T using 3-N apexes


    calculate_r_t(20, transformation_name)


Included Data
-------------

Currently there exists raw, average, and apex data for the following 
transformations between two SICK LMs 200 Lidars:

- Y Translation  
- X, Y Translation  
- X, Y, Z Translation  
- X, Y Translation & Yaw Rotation  
- X, Y, Z Translation & Yaw Rotation  
- X, Y, Z Translation & Yaw, Pitch Rotation  
- X, Y, Z Translation & Yaw, Pitch, Roll Rotation  
- Distance (Y) Translation Limitation Testing  
- Angle of Target Limitation Testing  
- Height of Target Limitation Testing  

Design of Experiment Example
----------------------------

See the following link for an example of a statistical design of 
experiment used for determining the effectiveness of this method:

https://docs.google.com/document/d/1moAm9RoFyEXCCiOv3lJogkNrMHWG_vOntVbNpSvxDf8/edit?usp=sharing


Contact
-------
Please contact justintcosentino (at) gmail (dot) com for additional information.



