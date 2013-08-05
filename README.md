Lidar Calibration for Use with Mobile Robotics
====================================

Justin Cosentino, Mili Shah, and Roger Eastman  
National Institute of Technology  
July, 2013  

Background 
----------

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
-----------
- Matlab, which may be purchased from http://www.mathworks.com/products/matlab/.  
- The sicktoolbox, which may be downloaded from http://sicktoolbox.sourceforge.net.
- Winged target

Installation
------------
After installing Matlab and the sicktoolbox, simply clone this repository 
and add the code directory to your Matlab path. At this point, the programs
may be run through Matlab. See the Usage Section for instructions on how to
use this package.

File System
-----------

There are two main directories within this repository: Data and Code. The 
Code directory contains all relevant functions for collecting lidar data, 
calculating the optimal transformation between two targets, and performing
statistical analysis. The Data directory contains various subdirectories 
that store raw scans, the average of these scans, and apex calculations.
With these subdirectories additional data relating to specific tests are 
stored. For example, with the Data/Apex/ there exists various 
subdirectories related to translation testing of a known ground truth. 
The ground truth related to each test can be found within these 
directories' READMEs. Data from each pose is stored within the three Data
subdirectories: Apex, Average, and Raw. It is assumed that data within this
each directory relates to a specific transformation between the two lidars,
and the importance of this will be discussed within the Usage section. Data
related to each pose is stored within the following format: 

    YYYYMMDD_POSE-NUM
    
This signifies that the data within this directory was taken on MM DD, YYYY 
and relates to pose number POSE-NUM. Within these directories there exist 
actual data files, stored as comma-separated values (CSV). The naming 
convention is as follows:

    LIDAR-NUM_pose_SCAN-NUM
    
For each pose (which is signified by the parent directory) there should exist
5 of these CSV data files. As mentioned above, each file corresponds to lidar 
LIDAR-NUM and contains data from the SCAN-NUM scan.

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
target, and optimize a homogenous transformation is stored with the Code 
directory. See each file for more specific descriptions of their 
functionality. The SICK to Matlab bridge used was the sicktoolbox, which 
can be downloaded from http://sicktoolbox.sourceforge.net. Installation and
connection instructions can also be found on the aforementioned web page.

Usage
-----

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
to college data, run the following command: 

    generate_data('/dev/ttyUSB0','/dev/ttyUSB1', 500000, 30, 1, true)

which is of the form

    generate_data(DEV_PATH_1, DEV_PATH_2, BAUD, SCANS, TEST, WRITE_FLAG)

This collects raw data, averages the raw data accounting for shot data, 
segments the data, fits lines to the segmented data, and calculated the
apex of the target. If the write_flag is set to true, the raw, average,
and apex data will be saved in the manner described within the File
System section. 

It is assumed that all folders within the Apex, Average, and Raw
directories named in the YYYYMMDD_POSE-NUM manner are associated with
a specifc transformation between the two lidars. If the transformation
between the two lidars is changed, one should move the associated data
into a subdirectory (eg. translation_x). This must be done in all 3
directories, as specific functions will iterate over subfolders and pull
data from all folders named in the YYYYMMDD_POSE-NUM format. In order to
perform additional testing on this data, one should move it from the
associated apex subdirectory to the Apex folder, perform analysis, and then
move the data back into its subdirectory. 

Once apex data has been collected, one can calculate the 
optimal transformation between the two lidars using:

    calculate_r_t()

Note: 
- Running this command will iterate through all apex data. If you have 
data from multiple lidar positions or old data that you do not want to use
within your calculations, move this data into a new folder (eg. 
"Data/Apex/Old") to ignore it.

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



