function [avg_data] =  average_scans(scans_x, scans_y, num_scans, ...
    test_num, lidar_num, pose_num, write_flag)
%==========================================================================
%==========================================================================
%
%  File: average_scans.m
%  Auth: Justin Cosentino
%  Date: 08 July 2013
%
%  In:   num_scans      - Desired number of laser scans
%        test_num       - The test number (for file-writing; > 0)
%        lidar_num      - The laser number (either 'l1' or 'l2')
%        pose_num       - The pose number (for file-writing; > 0)
%        write_flag     - Boolean determining if data is written to file
%
%  Out:  avg_data - a mxn matrix containing the results of the lidar scan,
%                   where m is the desired number of scans and n is the 
%                   number maximum number of points found in any one scan
%
%  Desc: Given a series of laser scans, average_scans computes the average
%        scan value for each corresponding point and corrects for data
%        lying outside of the mean+-std
%       
%
%  Usage:   average_scans(SCANS_X, SCANS_Y, NUM_SCANS, TEST,
%                       LIDAR, POSE, WRITE_FLAG)
%  Example: average_scans(scans_x, scans_y, 30, 1, 'l1', 1, true)
%
%==========================================================================

% Check for input params
narginchk(7,7)
if ~(strcmp(lidar_num, 'l1' ) || strcmp(lidar_num, 'l2'))
    error('generate_scan:: lidar_num must be either l1 or l2');
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
    
    % Calculate mean and standard deviation of each coordinate
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

% Write averages to data file if the designated scan does not already exist
if write_flag
    file = sprintf('%s_%d', datestr(date,'yyyymmdd'), test_num);
    dir = sprintf('~/Documents/laser_calibration/Data/Average/%s/', file);
    if ~exist(dir,'dir'), mkdir(dir); end
    path = sprintf('%s%s_pose_%d', dir, lidar_num, pose_num);
    if ~exist(path,'file')
        dlmwrite(path,avg_data,'delimiter', ',','precision', 7);
    else
        error('generate_scan:: File %s already exists', path)
    end
end


end % function average_scans