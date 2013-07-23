function  write_data(lidar_num, test_num, pose_num, num_scans, ...
    raw_x, raw_y, avg_data, apex)
%==========================================================================
%==========================================================================
%
%  File: write_data.m
%  Auth: Justin Cosentino
%  Date: 15 July 2013
%
%  In:   lidar_num - The laser number (either 'l1' or 'l2')
%        test_num  - The test number (for file-writing; > 0)
%        pose_num  - The pose number (for file-writing; > 0)
%        num_scans - Desired number of laser scans
%        raw_x     - Raw scan data of x coordinates
%        raw_y     - Raw scan data of y coordinates
%        avg_data  - Averaged data across all scans
%        apex      - The apex pose calcuated with least squares
%
%  Out:  None
%   
%  Desc: Writes all data to file system
%
%        Usage:   write_data()
%        Example: write_data()
%
%==========================================================================

% Save raw x and y data
nCols = size(raw_x,2);
nRows = size(raw_x,1);
data = zeros(2*nRows,nCols);
data(1:2:end,:) = raw_x;
data(2:2:end,:) = raw_y;

file = sprintf('%s_%d', datestr(date,'yyyymmdd'), test_num);
dir = sprintf('~/Documents/laser_calibration/Data/Raw/%s/', file)
if ~exist(dir,'dir'), mkdir(dir); end
path = sprintf('%s%s_pose_%d', dir, lidar_num, pose_num)
if ~exist(path,'file')
    dlmwrite(path,data,'delimiter', ',','precision', 7);
else
    error('generate_scan:: File %s already exists', path)
end

% Save avg x and y data
file = sprintf('%s_%d', datestr(date,'yyyymmdd'), test_num);
dir = sprintf('~/Documents/laser_calibration/Data/Average/%s/', file);
if ~exist(dir,'dir'), mkdir(dir); end
path = sprintf('%s%s_pose_%d', dir, lidar_num, pose_num);
if ~exist(path,'file')
    dlmwrite(path, avg_data,'delimiter', ',','precision', 7);
else
    error('generate_scan:: File %s already exists', path)
end

% Write apex to data file if the designated scan does not already exist
file = sprintf('%s_%d', datestr(date,'yyyymmdd'), test_num);
dir = sprintf('~/Documents/laser_calibration/Data/Apex/%s/', file);
if ~exist(dir,'dir'), mkdir(dir); end
path = sprintf('%s%s_pose_%d', dir, lidar_num, pose_num);
if ~exist(path,'file')
    dlmwrite(path, apex,'delimiter', ',','precision', 7);
else
    error('calculate_apex:: File %s already exists', path)
end
end