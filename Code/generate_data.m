function generate_data(sick_path_1, sick_path_2, ...
    sick_baud, num_scans, test_num, write_flag)
%==========================================================================
%==========================================================================
%
%  File: generate_data.m
%  Auth: Justin Cosentinum
%  Date: 08 July 2013
%
%  In:   sick_path_1    - Sick device path
%        sick_path_2    - Sick device path
%        sick_baud_rate - Desired baud rate of comm session
%        num_scans      - Desired number of laser scans
%        test_num       - The test number (for file-writing; > 0)
%        write_flag     - Boolean determining if data is written to file
%
%  Out:  None
%  
%  Desc: Given two lidars, generate_data will collect n scans and calculate
%        the apex of a target in regards to each lidars' coordinate frame.
%
%        Usage:   generate_data(DEV_PATH_1, DEV_PATH_2, BAUD, ...
%                   SCANS, TEST, WRITE_FLAG)
%        Example: generate_data('/dev/ttyUSB0', '/dev/ttyUSB1', 38400, ...
%                   30, 1, true)
%
%==========================================================================

clc; clf; close all;

% Check for input params
narginchk(6,6)

% Take 15 pose measurements, with a 5 second pause in between tests to
% allow for target adjustment
for pose_num=1:1

    clc; clf;
    
    % Generate laser scans from each lidar
    [l1_scans_x, l1_scans_y] = generate_scan(sick_path_1, sick_baud, ... 
        num_scans, test_num, 'l1', pose_num, write_flag)
%     [l2_scans_x, l2_scans_y] = generate_scan(sick_path_2, sick_baud, ...
%         num_scans, test_num, 'l2', pose_num, write_flag)

    % Calculate average points from n lidar scans
    l1_avg = average_scans(l1_scans_x, l1_scans_y)
%     l2_avg = average_scans(l2_scans_x, l2_scans_y)

    % Using modified split-merge algorithm to calculate edges of target
    [l1_inter, l1_indices] = segment_lines(l1_avg);
%     [l2_inter, l2_indices] = segment_lines(l2_avg);

    % Calculate apex of target using three inner intersections
    l1_apex = calculate_apex(l1_inter(:,2:4))
%     l2_apex = calculate_apex(l2_inter(:,2:4))
    subplot(1, 2, 1);
    hold on; grid on;
    plot(l1_avg(1,:), l1_avg(2,:),'r*')
    plot([l1_inter(1,:)],[l1_inter(2,:)], 'go-')
    plot(l1_apex(1),l1_apex(2),'g*')
    title('Lidar 1');
    axis([0 150 -75 75]);
    axis normal;

%     subplot(1, 2, 2);
%     hold on; grid on;
%     plot(l2_avg(1,:), l2_avg(2,:),'r*')
%     plot([l2_inter(1,:)],[l2_inter(2,:)], 'go-')
%     plot(l2_apex(1),l2_apex(2),'g*')
%     title('Lidar 2');
%     axis([0 150 -75 75]);
%     axis normal;
   
    % Write all data to csv files
    if write_flag
         write_data('l1', test_num, pose_num, num_scans, ...
                    l1_scans_x, l1_scans_y, l1_avg, l1_apex)
%          write_data('l2', test_num, pose_num, num_scans, ...
%                     l2_scans_x, l2_scans_y, l2_avg, l2_apex)
    end 
    
    pause(5)

end

end % function generate_data

