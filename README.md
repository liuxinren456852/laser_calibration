Lidar Calibration for Use with Mobile Robotics
==============================================

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
Matlab, which may be purchased from http://www.mathworks.com/products/matlab/.
The sicktoolbox, which may be downloaded from http://sicktoolbox.sourceforge.net.

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
connection instructions can also be found on the aforementioned webpage.

Usage
-----

In order to gather data, first connect both SICK lidars to the computer via
serial-to-USB cables. Upon connecting both devices, power on the lidars and
then run 

    dmesg
    
to find the path of each lidar. An example path would be '/dev/ttyUSB0'.
Once both paths have been determined, select a baud rate at which to run
both lidars. Although it is possible to run SICK lasers at 500k, our
devices are being run at 38400bps. The sicktoolbox specs used are as
follows:

    Sick Type: Sick LMS 200-30106
    Scan Angle: 180 (deg)
    Scan Resolution: 0.5 (deg)
    Measuring Mode: 8m/80m; 3 reflector bits
    Measuring Units: Centimeters (cm)

Once the lasers have been connected, place the target such that both wings
and each side are within view of each lidar. At this point, run the
following command:

    generate_data('/dev/ttyUSB1','/dev/ttyUSB5', 38400, 30, 1, true)

which is of the form

    generate_data('LIDAR_PATH_1','LIDAR_PATH_2',BAUD_RATE, NUM_SCANS, ... 
                  TRIAL_NUM, WRITE_FLAG)

This function will collect data and store it in the aforementioned 
directories. Once apex data has been collected, one can calculate the 
optimal transformation between the two lidars using:

    calculate_r_t()

Note: Running this command will iterate through all apex data. If you have
data from multiple lidar positions or old data that you do not want to use
 within your calculations, move this data into a new folder (eg. 
"Data/Apex/Old") to ignore it.


