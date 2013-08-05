Limitations Testing: Rotation (Yaw translation)
===============================================

Data
----

This test data corresponds with the following rotations:

    20130726_1 -> [x = 1.00m +/- 2mm,  y = 0.00mm +/- 2mm, z = 34.50mm +/- 2mm]  
		  [Yaw = 0 +/- 2 degrees]  
    20130726_2 -> [x = 1.00m +/- 2mm,  y = 0.00mm +/- 2mm, z = 34.50mm +/- 2mm]  
		  [Yaw = 5 +/- 2 degrees]  
    20130726_3 -> [x = 1.00m +/- 2mm,  y = 0.00mm +/- 2mm, z = 34.50mm +/- 2mm]  
		  [Yaw = 10 +/- 2 degrees]  
    20130726_4 -> [x = 1.00m +/- 2mm,  y = 0.00mm +/- 2mm, z = 34.50mm +/- 2mm]  
		  [Yaw = 15 +/- 2 degrees]  
    20130726_5 -> [x = 1.00m +/- 2mm,  y = 0.00mm +/- 2mm, z = 34.50mm +/- 2mm]  
		  [Yaw = 20 +/- 2 degrees]  
    20130726_6 -> [x = 1.00m +/- 2mm,  y = 0.00mm +/- 2mm, z = 34.50mm +/- 2mm]  
		  [Yaw = 25 +/- 2 degrees]  
    20130726_7 -> [x = 1.00m +/- 2mm,  y = 0.00mm +/- 2mm, z = 34.50mm +/- 2mm]  
		  [Yaw = 30 +/- 2 degrees]  
    20130726_8 -> [x = 1.00m +/- 2mm,  y = 0.00mm +/- 2mm, z = 34.50mm +/- 2mm]  
		  [Yaw = 35 +/- 2 degrees]  

Limitation
----------
The program was unable to collect data at both 40 degrees yaw. The scanner begins to get split points and the target loses its shape. However, robust fit can still fit a line.

Apex Height Estimations
-----------------------

    Apex Height (Relative to World):  34.50mm +/- 2mm  
    Lidar Height (Relative to World): 21.00mm +/- 2mm  
    Apex Height (Relative to Lidar):  13.50mm +/- 4mm  

Assumptions
-----------
We are assuming that the lidar origin is located at the center of the device and that there is *no* rotation of the lidar.

Further Work
------------
Ensure to check the number of scan points need to calculate an apex.
