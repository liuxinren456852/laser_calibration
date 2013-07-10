function [avg_x avg_y] =  read_avg_data(file_path)
%==========================================================================
%==========================================================================
%
%  File: read_avg_data.m
%  Auth: Justin Cosentino
%  Date: 10 July 2013
%
%  In:  file_path - path to file containing average scan data
%
%  Out: avg_x - x coordinate data read from file
%       avg_y - y coordinate data read from file
%   
%  Desc: 
%
%        Usage:   read_avg_data(FILE_PATH)
%        Example: read_avg_data()
%
%==========================================================================

% Clear window
clc;

avg_x = []; avg_y = [];

avg_data = dlmread(file_path, ',');
avg_x = avg_data(1,:);
avg_y = avg_data(2,:);

end % function read_avg_data
