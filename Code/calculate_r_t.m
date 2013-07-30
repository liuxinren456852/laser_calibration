function [r t] =  calculate_r_t(maxNum)
%==========================================================================
%==========================================================================
%
%  File: calculate_r_t.m
%  Auth: Justin Cosentino
%  Date: 10 July 2013
%
%  In:  none
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
[l1_apexes l2_apexes] = read_apex_data();
yaw = []; pitch = []; row=[]; ts= [];

% Rotation and translation std
r = []; t = [];

% Possible number of poses
numPoses = size(l1_apexes,2)/5;
possible_poses = [1:numPoses];

% Iterate to max num calcuating the std for the optimal r and t for the
% given number of apex points
for numPoints = 3:maxNum
    
    disp(sprintf('%d Apex Point(s)', numPoints));
    
    % Select X poses randomly
    poses = possible_poses(randperm(numPoses,numPoints));
    
    % Selecting a random scan from each pose, calcuate the optimal R and T
    % using the X number of poses i t'Standard Deviation'imes
    for i=1:5000
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
    res = [std_yaw std_pitch std_row];
    % m_yaw = mean(yaw)*180/pi;
    % m_pitch = mean(pitch)*180/pi;
    % m_row = mean(row)*180/pi;
    % m_T = [mean(ts(1,:)) ; mean(ts(2,:)) ; mean(ts(3,:))]
    % res = [m_yaw m_pitch m_row]

    yaw = []; pitch = []; row=[]; ts= [];
    r = [r ; res];
    t = [t ; std_T];
end

% Plot standard deviation
hold on; grid on;
title('Rotation Standard Deviation');
ylabel('Standard Deviation (Degrees)');
xlabel('Apex Points');
plot([3:maxNum], r(:,1), 'r-*')
plot([3:maxNum], r(:,2), 'b-*')
plot([3:maxNum], r(:,3), 'g-*')
legend('yaw', 'pitch', 'roll')

% Plot standard deviation
figure(); hold on; grid on;
title('Translation Standard Deviation');
ylabel('Standard Deviation (mm)');
xlabel('Apex Points');
plot([3:maxNum], t(:,1), 'r-*')
plot([3:maxNum], t(:,2), 'b-*')
plot([3:maxNum], t(:,3), 'g-*')
legend('x', 'y', 'z')

end % function calculate_r_t

function ang=rot2ang(R)
ang = zeros(1,3);

% yaw pitch row
ang(1) = atan2(R(2,1),R(1,1));
ang(2) = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
ang(3) = atan2(R(3,2),R(3,3));
end

% for i=1:10000
%     apex_1 = []; apex_2 = [];
% 
%     for j=1:length(l1_apexes)/5
%         num = randi([1,5]) * j;
%         apex_1 = [apex_1 l1_apexes(:, 1*num)];
%         apex_2 = [apex_2 l2_apexes(:, 1*num)];  
%     end 
% 
%     [R,T] = least_squares_fitting(apex_1, apex_2);
%     angles = rot2ang(R);
%     yaw = [yaw angles(1)];
%     pitch = [pitch angles(2)];
%     row = [row angles(3)];
%     ts = [ts T];
% end
% disp('20 Apex Points')
% m_yaw = mean(yaw)*180/pi
% m_pitch = mean(pitch)*180/pi
% m_row = mean(row)*180/pi
% std_yaw = std(yaw)*180/pi
% std_pitch = std(pitch)*180/pi
% std_row = std(row)*180/pi
% m_T = [mean(ts(1,:)) ; mean(ts(2,:)) ; mean(ts(3,:))]
% std_T = [std(ts(1,:)) ; std(ts(2,:)) ; std(ts(3,:))]
% hold off
% figure(3)
% [R,T] = least_squares_fitting(l1_apexes, l2_apexes);
% l2_transform = R'*(l2_apexes - repmat(T,1,length(l2_apexes)));
% plot3(l1_apexes(1,:), l1_apexes(2,:), l1_apexes(3,:),'r*'); hold on; grid on;
% plot3(l2_transform(1,:), l2_transform(2,:), l2_transform(3,:),'g*');