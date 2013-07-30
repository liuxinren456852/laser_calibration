function [scans_x, scans_y] =  generate_scan(sick_dev_path, sick_baud, ...
    num_scans, lidar_no)%, bg_x, bg_y)
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
%        lidar_no       - The laser number (either 'l1' or 'l2')
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
%        Usage:   generate_scan(SICK_DEV_PATH, SICK_BAUD, NUM_SCANS, LIDAR)
%        Example: generate_scan('/dev/ttyUSB0', 38400, 30, 'l1')
%
%==========================================================================
global data_x data_y

% Check for input params
%narginchk(4,4)

if ~(strcmp(lidar_no, 'l1' ) || strcmp(lidar_no, 'l2'))
    error('generate_scan:: lidar_no must be either l1 or l2');
end

% Clear window
% clc;

sprintf('Scanning w/Lidar: %s', lidar_no)

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
%     valid_indices = data.range < 200; %overflowValue(init_lidar.meas_mode);
    
    % Keep only valid (non-overflow) measurements
%     data.range = data.range(valid_indices);
    
    % Generate angles corresponding to range measurements
    theta = (data.fov/2:-data.res:-data.fov/2)'*pi/180;
%     theta = theta(valid_indices);
    
    % Convert polar to Cartesian coordinates 
    [x_pos y_pos] = pol2cart(theta,data.range);
    
    mask = (-70 < y_pos) & (y_pos < 70) & (0 < x_pos) & (x_pos < 500);
    x_pos = x_pos(mask);
    y_pos = y_pos(mask);
    
    % Add converted coordinates to data matrix
    if i == 1
        data_x = x_pos';
        data_y = y_pos';
    elseif length(data_x) == length(x_pos')
        data_x = [data_x ; x_pos'];
        data_y = [data_y ; y_pos'];
    end
    
end

% Uninitialize the device
clear sicklms;

scans_x = data_x
scans_y = data_y

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
