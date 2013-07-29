function [avg_data] =  average_scans(scans_x, scans_y)
%==========================================================================
%==========================================================================
%
%  File: average_scans.m
%  Auth: Justin Cosentino
%  Date: 08 July 2013
%
%  In:   scans_x - All x coordinate scan data
%        scans_y - All y coordinate scan data
%
%  Out:  avg_data - a mxn matrix containing the results of the lidar scan,
%                   where m is the desired number of scans and n is the 
%                   number maximum number of points found in any one scan
%
%  Desc: Given a series of laser scans, average_scans computes the average
%        scan value for each corresponding point and corrects for data
%        lying outside of the mean+-std
%       
%  Usage:   average_scans(SCANS_X, SCANS_Y)
%  Example: average_scans(scans_x, scans_y)
%
%==========================================================================

% Check for input params
narginchk(2,2)

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

clf; figure(1); hold on; grid off;
title('Lidar Scan Data (Average)');
%ylabel('Standard Deviation (mm)');
%rectangle('Position', [-7.5, -7.5, 15, 15], 'FaceColor', 'b')
a=axis; 
plot(avg_y, avg_x, 'ko')
axis square;

end % function average_scans