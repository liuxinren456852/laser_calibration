function [R, T] =  calculate_r_t()
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

% Clear window
clc;

[l1_apexes l2_apexes] = read_apex_data();
rotationVals = []; tVals= [];


for i=1:1000
    apex_1 = []; apex_2 = [];

    for j=1:length(l1_apexes)/5
        num = randi([1,5]) * j;
        apex_1 = [apex_1 l1_apexes(:, 1*num)];
        apex_2 = [apex_2 l2_apexes(:, 1*num)];
    end 

    [R,T] = least_squares_fitting(apex_1, apex_2);
    r = vrrotmat2vec(R);
    rotationVals = [rotationVals r(4)];
    
    tVals = [tVals T];
     
end

m = mean(rotationVals)
s = std(rotationVals);
degrees = s*180/pi

m = [mean(tVals(1,:)) ; mean(tVals(2,:)) ; mean(tVals(3,:))]*10
std_mm = [std(tVals(1,:)) ; std(tVals(2,:)) ; std(tVals(3,:))]*10

hold off
figure(2)
histfit(rotationVals)

hold off
figure(3)
[R,T] = least_squares_fitting(l1_apexes, l2_apexes);
l2_transform = R'*(l2_apexes - repmat(T,1,length(l2_apexes)));
plot3(l1_apexes(1,:), l1_apexes(2,:), l1_apexes(3,:),'r*'); hold on; grid on;
plot3(l2_transform(1,:), l2_transform(2,:), l2_transform(3,:),'g*');

end % function calculate_r_t