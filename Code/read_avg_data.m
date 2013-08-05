function read_avg_data(dir_name)
%==========================================================================
%==========================================================================
%
%  File: read_avg_data.m
%  Auth: Justin Cosentino
%  Date: 10 July 2013
%
%  In:  dir_name - directory containing average scan data
%
%  Out: avg_x - x coordinate data read from file
%       avg_y - y coordinate data read from file
%   
%  Desc: Reads average data and recomputes apex points, saving them to
%        Data/Apex/. Correpsonding apex points for each avg data set must
%        not currently exist within that folder, and should be moved or
%        removed before reading all averages. 
%
%        Usage:   read_avg_data()
%
%==========================================================================

% Clear window
clc;

% Grab apex data paths and dirs
avg_data_path = strcat('~/Documents/laser_calibration/Data/Average/',dir_name,'/');
tests = dir(strcat(avg_data_path,'2013*_*'));

% Iterate through each apex test subdir
for test = tests'
    test_path = strcat(avg_data_path, test.name);

    
    % Fetch all lidar pose files within this dir
    l1_files = dir(strcat(avg_data_path, test.name, '/l1_pose_*'));
    l2_files = dir(strcat(avg_data_path, test.name, '/l2_pose_*'));
    
    
    % Compile lidar one apex data for this test
    intersections = zeros(2,3); count = 0; file_name = 'l1_pose';
    for file = l1_files'
        count = count + 1;
        avg = dlmread(strcat(test_path, '/',file.name), ',');
        [inter] = segment_lines(avg);
        intersections = intersections + inter;
    end
    
    intersections = intersections / count;
    [apex] = calculate_apex(intersections)
    %write_apex(test.name, file_name, apex);
    
    % Compile lidar two apex data for this test
    intersections = zeros(2,3); count = 0; file_name = 'l2_pose';
    for file = l2_files'
        count = count + 1;
        avg = dlmread(strcat(test_path, '/',file.name), ',');
        [inter] = segment_lines(avg);
        intersections = intersections + inter;
    end
    
    intersections = intersections / count;
    [apex] = calculate_apex(intersections)
    %write_apex(test.name, file_name, apex);
    
end


%==========================================================================
%==========================================================================
function write_apex(test_name,file_name,apex)
%==========================================================================
% Func: write_apex()
% Desc: Writes apex data to file system
%==========================================================================

new_dir = sprintf('~/Documents/laser_calibration/Data/Apex/%s/',test_name);
if ~exist(new_dir,'dir'), mkdir(new_dir); end
path = sprintf('%s%s', new_dir, file_name);
if ~exist(path,'file')
    dlmwrite(path, apex,'delimiter', ',','precision', 7);
else
    error('calculate_apex:: File %s already exists', path)
end

end % function write_apex


end % function read_avg_data
