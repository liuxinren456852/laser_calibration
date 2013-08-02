Limitations Testing: Height (Z translation)
=============================================

Data
----

This test data corresponds with the following distances (x = N mm where N is the lidar height relative to the base of the target):

    20130726_1 -> [x = 100.00mm +/- 2mm,  y = 0.00mm +/- 2mm, z = 20.5mm +/- .5mm]  
    20130726_2 -> [x = 100.00mm +/- 2mm,  y = 0.00mm +/- 2mm, z = 22.5mm +/- .5mm]  
    20130726_3 -> [x = 100.00mm +/- 2mm,  y = 0.00mm +/- 2mm, z = 24.5mm +/- .5mm]  
    20130726_4 -> [x = 100.00mm +/- 2mm,  y = 0.00mm +/- 2mm, z = 26.5mm +/- .5mm]  
    20130726_5 -> [x = 100.00mm +/- 2mm,  y = 0.00mm +/- 2mm, z = 28.5mm +/- .5mm]  
    20130726_x -> [x = 100.00mm +/- 2mm,  y = 0.00mm +/- 2mm, z = 30.5mm +/- .5mm]  

Limitation
----------
The program was unable to collect data beginning at z=30.5mm (robustfit did not have enough valid coordinates).

Apex Height Estimations
-----------------------

    Apex Height (Relative to World):  34.50mm +/- 2mm  
    Target X Translation (Relative to Lidar): 100.00mm +/- 2mm  
    Target Y Translation (Relative to Lidar):  0mm +/- 2mm  

Assumptions
-----------
We are assuming that the lidar origin is located at the center of the device and that there is *no* rotation of the lidar.

Further Work
------------
Ensure to check the number of scan points need to calculate an apex.
