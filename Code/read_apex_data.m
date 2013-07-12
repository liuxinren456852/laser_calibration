function [l1_apexes, l2_apexes] =  read_apex_data()
%==========================================================================
%==========================================================================
%
%  File: read_apex_data.m
%  Auth: Justin Cosentino
%  Date: 10 July 2013
%
%  In:
%
%  Out:  
%   
%  Desc: 
%
%        Usage:   read_apex_data()
%        Example: read_apex_data()
%
%==========================================================================

% Clear window
clc;

l1_apexes = []; l2_apexes = [];

apex_data_path = '~/Documents/laser_calibration/Data/Apex/';

tests = dir('~/Documents/laser_calibration/Data/Apex/*_*');
for test = tests'
    test_path = strcat(apex_data_path, test.name);
    
    l1_files = dir(strcat(apex_data_path, test.name, '/l1_pose_*'));
    l2_files = dir(strcat(apex_data_path, test.name, '/l2_pose_*'));
    
    for file = l1_files'
        file_name = file.name;
        apex = dlmread(strcat(test_path, '/',file_name), ',');
        l1_apexes = [l1_apexes apex];
    end
    
    for file = l2_files'
        file_name = file.name;
        apex = dlmread(strcat(test_path, '/',file_name), ',');
        l2_apexes = [l2_apexes apex];
    end
end

% Plot Apexes
plot3(l1_apexes(1,:), l1_apexes(2,:), l1_apexes(3,:),'r*'); hold on; grid on;
plot3(l2_apexes(1,:), l2_apexes(2,:), l2_apexes(3,:),'go');

end % function read_apex_data
