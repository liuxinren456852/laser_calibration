function [scans_x, scans_y] =  generate_scan(sick_dev_path, sick_baud, ...
    num_scans, test_no, lidar_no, pose_no, write_flag)
%==========================================================================
%==========================================================================
%
%  File: generate_scan.m
%  Auth: Justin Cosentino
%  Date: 08 July 2013
%
%  In:   sick_dev_path  - Sick device path
%        sick_baud_rate - Desired baud rate of comm session
%        num_scans      - Desired number of laser scans
%        test_no        - The test number (for file-writing; > 0)
%        lidar_no       - The laser number (either 'l1' or 'l2')
%        pose_no        - The pose number (for file-writing; > 0)
%        write_flag     - Boolean determining if data is written to file
%
%  Out:  scans_x - a mxn matrix containing the results of the lidar scan,
%                 where m is the desired number of scans and n is the 
%                 number maximum number of points found in any one scan
%   
%  Desc: Initializes a SICK LMS 200 lidar and takes n scans, converting the
%        scan data from polar to cartesian coordinates. Overflow data is 
%        discarded from the laser scan, eliminating the need for background
%        subtraction
%
%        Usage:   generate_scan(SICK_DEV_PATH, SICK_BAUD, NUM_SCANS, TEST,
%                   LIDAR, POSE)
%        Example: generate_scan('/dev/ttyUSB0', 38400, 30, 1, 'l1', 1)
%
%==========================================================================
global data_x data_y

% Check for input params
narginchk(7,7)

if ~(strcmp(lidar_no, 'l1' ) || strcmp(lidar_no, 'l2'))
    error('generate_scan:: lidar_no must be either l1 or l2');
end

% Clear window
clc;

% Initialize the SICK LMS 200
init_lidar = sicklms('init', sick_dev_path, sick_baud);
if isempty(init_lidar), error('generate_scan:: sicklms init failed'); end

for i = 1:num_scans
    
    % Grab Lidar data
    data = sicklms('grab');
    if isempty(data), error('generate_scan:: grab data failed'); end
    
    % Ensure range data is returned
    if isempty(data), error('generate_scan:: no range data returned'); end
    
    % For filtering overflow (e.g. max range) values returned from LMS
    valid_indices = data.range < 150; %overflowValue(init_lidar.meas_mode);

    % Keep only valid (non-overflow) measurements
    data.range = data.range(valid_indices);
    
    % If recording reflection data, keep only valid measurements
    if ~isempty(data.reflect) 
        data.reflect = data.reflect(valid_indices);        
    end
    
    % Generate angles corresponding to range measurements
    theta = (data.fov/2:-data.res:-data.fov/2)'*pi/180;
    theta = theta(valid_indices);
    
    % Convert polar to Cartesian coordinates 
    [x_pos y_pos] = pol2cart(theta,data.range);
    
    % Add converted coordinates to data matrix
    if i == 1
        data_x = x_pos'; data_y = y_pos';
    else
        data_x = [data_x ; x_pos'];
        data_y = [data_y ; y_pos'];
    end
    
end

nCols = size(data_x,2);
data = zeros(2*num_scans,nCols);
data(1:2:end,:) = data_x;
data(2:2:end,:) = data_y;

% Save both data_x and data_y to a csv file
if write_flag
    file = sprintf('%s_%d', datestr(date,'yyyymmdd'), test_no);
    dir = sprintf('~/Documents/laser_calibration/Data/Raw/%s/', file)
    if ~exist(dir,'dir'), mkdir(dir); end
    path = sprintf('%s%s_pose_%d', dir, lidar_no, pose_no)
    if ~exist(path,'file')
        dlmwrite(path,data,'delimiter', ',','precision', 7);
    else
        error('generate_scan:: File %s already exists', path)
    end
end 

% Uninitialize the device
clear sicklms;

scans_x = data_x;
scans_y = data_y;

end % function generate_scan

%==========================================================================
%==========================================================================
function overflow = overflowValue(meas_mode)
%==========================================================================
% Func: overflowValue()
% Desc:  This function maps the given measuring mode value to the correct
%        overflow value so that Sick LMS returns can be properly filtered
%==========================================================================
overflow = inf;

% NOTE: These values are from page 124 of LMS Telegram Listing
switch meas_mode
    case {0,1,2};
        overflow = 8183;
    case {3,4,10}
        overflow = 16385;
    case {5,6,7}
        overflow = 32759;
    otherwise
        warning('Unrecognized measuring mode: Using inf for overflow!');
end

end % overflowValue
