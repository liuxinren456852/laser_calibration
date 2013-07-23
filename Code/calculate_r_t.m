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

function dTh = ErrProp3D(X,dX,Th)
% Calculates Error Propogation for the
% 3D Registration Problem
%
% sum||xhati-[R(xi)+t]||^2
%
% Inputs:
% X  = [x1;x2;...;xn;xhat1;xhat2;...;xhatn] 
% dx = uncertainty of X
% Th = [theta;phi;rho;x;y;z]
%
% Outputs:
% dTh = uncertainty of Th
%
% Mili Shah
[m,n] = size(X);
if m<n, X = X';  end
[m,n] = size(dX);
if m<n, dX = dX'; end
n=length(X);

u  = [cos(Th(1))*cos(Th(2));cos(Th(1))*sin(Th(2));sin(Th(1))];
uT = [-sin(Th(1))*cos(Th(2));-sin(Th(1))*sin(Th(2));cos(Th(1))];
uP = [-cos(Th(1))*sin(Th(2));cos(Th(1))*cos(Th(2));0];
uTT= [-cos(Th(1))*cos(Th(2));-cos(Th(1))*sin(Th(2));-sin(Th(1))];
uTP= [sin(Th(1))*sin(Th(2));-sin(Th(1))*cos(Th(2));0];
uPP= [-cos(Th(1))*cos(Th(2));-cos(Th(1))*sin(Th(2));0];
uS = [0 -u(3) u(2);u(3) 0 -u(1);-u(2) u(1) 0];
uST= [0 -uT(3) uT(2);uT(3) 0 -uT(1);-uT(2) uT(1) 0];
uSP= [0 -uP(3) uP(2);uP(3) 0 -uP(1);-uP(2) uP(1) 0];
uSTT= [0 -uTT(3) uTT(2);uTT(3) 0 -uTT(1);-uTT(2) uTT(1) 0];
uSTP= [0 -uTP(3) uTP(2);uTP(3) 0 -uTP(1);-uTP(2) uTP(1) 0];
uSPP= [0 -uPP(3) uPP(2);uPP(3) 0 -uPP(1);-uPP(2) uPP(1) 0];

R  = cos(Th(3))*eye(3) + sin(Th(3))*uS + (1-cos(Th(3)))*u*u';
RT = sin(Th(3))*uST + (1-cos(Th(3)))*(uT*u'+u*uT');
RP = sin(Th(3))*uSP + (1-cos(Th(3)))*(uP*u'+u*uP');
RR = -sin(Th(3))*eye(3) + cos(Th(3))*uS + sin(Th(3))*u*u';
RTT= sin(Th(3))*uSTT + (1-cos(Th(3)))*(uTT*u'+2*uT*uT'+u*uTT');
RTP= sin(Th(3))*uSTP + (1-cos(Th(3)))*(uTP*u'+uT*uP'+uP*uT'+u*uTP');
RTR= cos(Th(3))*uST + (sin(Th(3)))*(uT*u'+u*uT');
RPP= sin(Th(3))*uSPP + (1-cos(Th(3)))*(uPP*u'+2*uP*uP'+u*uPP');
RRP= cos(Th(3))*uSP + (sin(Th(3)))*(uP*u'+u*uP');
RRR= -cos(Th(3))*eye(3) -sin(Th(3))*uS + cos(Th(3))*u*u';

dgdX = zeros(6,n);
dgdTh= zeros(6,6);

for i = 1:n/6
    dgdX(:,3*i-2:3*i) = [...
        2*(Th(4:6)-X(n/2+3*i-2:n/2+3*i))'*RT;...
        2*(Th(4:6)-X(n/2+3*i-2:n/2+3*i))'*RP;...
        2*(Th(4:6)-X(n/2+3*i-2:n/2+3*i))'*RR;...
        2*R];
    dgdX(:,n/2+3*i-2:n/2+3*i) = [...
        -2*(RT*X(3*i-2:3*i))';...
        -2*(RP*X(3*i-2:3*i))';...
        -2*(RR*X(3*i-2:3*i))';...
        -2*eye(3)];
    dgdTh = dgdTh + [...
        2*(RTT*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RTP*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RTR*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RT*X(3*i-2:3*i))';...
        2*(RTP*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RPP*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RRP*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RP*X(3*i-2:3*i))';...
        2*(RTR*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RRP*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RRR*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RR*X(3*i-2:3*i))';...
        2*(RT*X(3*i-2:3*i)) 2*(RT*X(3*i-2:3*i)) 2*(RT*X(3*i-2:3*i)) 2*eye(3)];
end
dTh = -inv(dgdTh)*dgdX*dX;
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