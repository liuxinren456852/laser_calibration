function [avg_data] =  average_scans(scans_x, scans_y, num_scans, test_no, lidar_no, pose_no)
%==========================================================================
%==========================================================================
%
%  File: average_scans.m
%  Auth: Justin Cosentino
%  Date: 08 July 2013
%
%  In:   num_scans      - Desired number of laser scans
%        test_no        - The test number (for file-writing; > 0)
%        lidar_no       - The laser number (either 'l1' or 'l2')
%        pose_no        - The pose number (for file-writing; > 0)
%
%  Out:  avg_data - a mxn matrix containing the results of the lidar scan,
%                   where m is the desired number of scans and n is the 
%                   number maximum number of points found in any one scan
%
%  Desc:  
%
%        Usage:   generate_scan(SICK_DEV_PATH, SICK_BAUD, NUM_SCANS, TEST, LIDAR, POSE)
%        Example: generate_scan('/dev/ttyUSB0', 38400, 30, 1, 'l1', 1)
%
%==========================================================================

% Check for input params
narginchk(6,6)

if ~(strcmp(lidar_no, 'l1' ) || strcmp(lidar_no, 'l2'))
    error('generate_scan:: lidar_no must be either l1 or l2');
end

% Clear window
clc;

num_points = size(scans_x,2);
if (num_points ~= size(scans_y,2))
    error('average_scans:: invalid scan dimesions')
end

avg_x = []; avg_y = [];

for i = 1:num_points
    current_x = scans_x(:,i);
    current_y = scans_y(:,i);
    
    % Calculate mean and standard deviation of each coordinate col
    mean_x = mean(current_x);
    mean_y = mean(current_y);
    std_x = std(current_x);
    std_y = std(current_y);
    usuable_x = current_x > (mean_x-std_x) & current_x < (mean_x+std_x);
    usuable_y = current_y > (mean_y-std_y) & current_y < (mean_y+std_y);
    avg_x(i) = mean(current_x(usuable_x));
    avg_y(i) = mean(current_y(usuable_y));
end

% Combine coordinates and remove all columns containing NaN elements
avg_data = [avg_x ; avg_y];
avg_data( :, all( isnan( avg_data ), 1 ) ) = [];
avg_data( :, all( isnan( avg_data ), 2 ) ) = [];

file = sprintf('%s_%d', datestr(date,'yyyymmdd'), test_no);
dir = sprintf('~/Documents/NIST/Calibration/Data/Average/%s/', file)
if ~exist(dir,'dir'), mkdir(dir); end
path = sprintf('%s%s_pose_%d', dir, lidar_no, pose_no)
if ~exist(path,'file')
    dlmwrite(path,avg_data,'delimiter', ',','precision', 7);
else
    error('generate_scan:: File %s already exists', path)
end


end % function average_scans