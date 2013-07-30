function [bg_x, bg_y] =  background_scan(sick_dev_path, sick_baud, ...
    num_scans)
%==========================================================================
%==========================================================================
%
%  File: background_scan.m
%  Auth: Justin Cosentino
%  Date: 25 July 2013
%
%  In:   sick_path_1    - Sick device path
%        sick_baud_rate - Desired baud rate of comm session
%        num_scans      - Desired number of laser scans
%
%  Out:  bg_l1 - background scan data from lidar 1x
%        bg_l2 - background scan data from lidar 2 
%   
%  Desc:
%
%        Usage: 
%        Example:
%
%==========================================================================
global data_x data_y

% Check for input params
narginchk(3,3)

% Clear window
% clc;

sprintf('Background scan...')

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
    valid_indices = data.range < 600; %overflowValue(init_lidar.meas_mode);

    data.range = data.range(valid_indices);

    % Generate angles corresponding to range measurements
    theta = (data.fov/2:-data.res:-data.fov/2)'*pi/180;
    theta = theta(valid_indices);

    % Convert polar to Cartesian coordinates 
    [x_pos y_pos] = pol2cart(theta,data.range);
    
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

bg_x = data_x;
bg_y = data_y;

figure(1)
grid on;
plot(bg_x, bg_y,'kx'); 

end % function background_scan
