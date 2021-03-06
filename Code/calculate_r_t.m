function [r t] =  calculate_r_t(maxNum, dir)
%==========================================================================
%==========================================================================
%
%  File: calculate_r_t.m
%  Auth: Justin Cosentino
%  Date: 10 July 2013
%
%  In:  maxNum - The maximum number of apexes used in calcuations
%       dir   - The dirrectory containing apex data
%
%  Out: R - optimal rotation calculated by least squares algortthm
%       T - optimal translation calcuated by least squares algorithm
%   
%  Desc: Reads all apex data and uses this data to calculate the optimal 
%        rotation and translation between two lidar
%
%        Usage:   calculate_r_t()
%        Example: calculate_r_t()
%
%==========================================================================
clc; close all; clf;

% Initialize Variables
[l1_apexes l2_apexes] = read_apex_data(dir);
yaw = []; pitch = []; row=[]; ts= [];

% Rotation and translation std
std_r = []; std_t = []; m_r = []; m_t = [];
% Possible number of poses
numPoses = size(l1_apexes,2)/5;
possible_poses = 1:numPoses;

% Iterate to max num calcuating the std for the optimal r and t for the
% given number of apex points
for numPoints = 3:maxNum
    
    fprintf('%d Apex Point(s)\n', numPoints);
    
    % Select X poses randomly
    poses = possible_poses(randperm(numPoses,numPoints));
    
    % Selecting a random scan from each pose, calcuate the optimal R and T
    % using the X number of poses i t'Standard Deviation'imes
    for i=1:1000
        apex_1 = []; apex_2 = [];

        for j=1:numPoints
            num = randi([1,5]) + (poses(j)-1)*5;
            apex_1 = [apex_1 l1_apexes(:, 1*num)];
            apex_2 = [apex_2 l2_apexes(:, 1*num)];  
        end 

        [R,T] = least_squares_fitting(apex_1, apex_2);
        angles = rot2ang(R);
        yaw = [yaw angles(1)];
        pitch = [pitch angles(2)];
        row = [row angles(3)];
        ts = [ts T];
    end
    
    % Find the standard deviation of the optimal Rs and Ts
    std_yaw = std(yaw)*180/pi;
    std_pitch = std(pitch)*180/pi;
    std_row = std(row)*180/pi;
    std_T = [std(ts(1,:)) ; std(ts(2,:)) ; std(ts(3,:))]';
    std_res = [std_yaw std_pitch std_row];
        
    std_r = [std_r ; std_res];
    std_t = [std_t ; std_T];
    
    m_yaw = mean(yaw)*180/pi;
    m_pitch = mean(pitch)*180/pi;
    m_row = mean(row)*180/pi;
    m_T = [mean(ts(1,:)) ; mean(ts(2,:)) ; mean(ts(3,:))]';
    m_res = [m_yaw m_pitch m_row];

    m_r = [m_r ; m_res];
    m_t = [m_t ; m_T];
    
    if numPoints == 10
        ten_m_r = m_res
        ten_m_t = m_T
    end 
        
    
    yaw = []; pitch = []; row=[]; ts= []; sigmas = [];
        
end

% Plot standard deviation (Rotation)
subplot(2, 2, 1);
hold on; grid on;
plot([3:maxNum], std_r(:,1), 'r-*')
plot([3:maxNum], std_r(:,2), 'b-*')
plot([3:maxNum], std_r(:,3), 'g-*')
legend('yaw', 'pitch', 'roll')
title('Rotation Standard Deviation');
ylabel('Standard Deviation (Degrees)');
xlabel('Apex Points');

% Plot standard deviation (Translation)
subplot(2, 2, 2);
hold on; grid on;
plot([3:maxNum], std_t(:,1), 'r-*')
plot([3:maxNum], std_t(:,2), 'b-*')
plot([3:maxNum], std_t(:,3), 'g-*')
ylabel('Standard Deviation (mm)');
xlabel('Apex Points');
title('Translation Standard Deviation');
legend('x', 'y', 'z');

% Plot mean (Rotation)
subplot(2, 2, 3);
hold on; grid on;
plot([3:maxNum], m_r(:,1), 'r-*')
plot([3:maxNum], m_r(:,2), 'b-*')
plot([3:maxNum], m_r(:,3), 'g-*')
legend('yaw', 'pitch', 'roll')
title('Rotation Mean');
ylabel('Mean (Degrees)');
xlabel('Apex Points');

% Plot mean (Translation)
subplot(2, 2, 4);
hold on; grid on;
plot([3:maxNum], m_t(:,1), 'r-*')
plot([3:maxNum], m_t(:,2), 'b-*')
plot([3:maxNum], m_t(:,3), 'g-*')
ylabel('Mean (mm)');
xlabel('Apex Points');
title('Translation Mean');
legend('x', 'y', 'z')

figure()
hold on; grid on;
[R,T] = least_squares_fitting(l1_apexes, l2_apexes);
l2_transform = R'*(l2_apexes - repmat(T,1,length(l2_apexes)));
plot3(l1_apexes(1,:), l1_apexes(2,:), l1_apexes(3,:),'r*'); hold on; grid on;
plot3(l2_transform(1,:), l2_transform(2,:), l2_transform(3,:),'g*');

end % function calculate_r_t

function ang=rot2ang(R)
ang = zeros(1,3);

% yaw pitch row
ang(1) = atan2(R(2,1),R(1,1));
ang(2) = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
ang(3) = atan2(R(3,2),R(3,3));
end