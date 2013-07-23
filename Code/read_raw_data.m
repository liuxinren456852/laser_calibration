function [x y] =  read_raw_data(file_path)
%==========================================================================
%==========================================================================
%
%  File: read_raw_data.m
%  Auth: Justin Cosentino
%  Date: 10 July 2013
%
%  In:  file_path - path to file containing raw scan data
%
%  Out: x - x coordinate data read from file
%       y - y coordinate data read from file
%   
%  Desc: 
%
%        Usage:   read_raw_data(FILE_PATH)
%        Example: read_raw_data('/home/jtc3/Documents/laser_calibration/
%                                Data/Raw/20130709_1/l1_pose_1')
%
%==========================================================================

% Clear window
clc;

x = []; y = [];

data = dlmread(file_path, ',');

x = data(1 : 2 : end, :);
y = data(2 : 2 : end, :);

nnd = [];
for i = 1:length(x)
    x_res = lillietest(x(:,i));
    y_res = lillietest(y(:,i));
    
    if x_res == 0 || y_res == 0
        nnd = [nnd i];
    end

end
nnd
length(x)


nnd = [];
for i = 1:length(x)
    x_res = jbtest(x(:,i));
    y_res = jbtest(y(:,i));
    
    if x_res == 0 || y_res == 0
        nnd = [nnd i];
    end
end
nnd
length(x)

end % function read_raw_data