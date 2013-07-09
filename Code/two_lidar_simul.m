function two_lidar_simul(snoise)
%==========================================================================
%==========================================================================
%
%  File: two_lidar_simul.m
%  Auth: Justin Cosentino
%  Date: 03 July 2013
%
%  In:   snoise  - lidar noise scalar
%  
%  Out:  None
%
%  Desc: Generates a target and applies a random transformation to this 
%        target. Then calculates the apex of the target using two different
%        lidar scans that are randomly generated about the target.
%
%        Usage:   two_lidar_simul(snoise)
%        Example: two_lidar_simul(.01)
%
%==========================================================================

% Clear window and figures
clc; clf; close all;

figure('Name','Two Lidar Simulation [Lidars]'); hold on; grid on;

% Generate transformations
[rand_r1, rand_t1] = generateTransformation(pi/8, pi/8, 2*pi);
[rand_r2, rand_t2] = generateTransformation(pi/8, pi/8, 2*pi);

% Apply each lidar transformation to target
[apex1, points1] = generateTargetTransform(rand_r1,rand_t1, 1);
[apex2, points2] = generateTargetTransform(rand_r2,rand_t2, 2);

% Generate lidar scans
scan_one_points = generateLidarScan(apex1,points1);
scan_two_points = generateLidarScan(apex2,points2);

% Add noise to scan results
scan_one = scan_one_points + snoise*rand(size(scan_one_points));
scan_two = scan_two_points + snoise*rand(size(scan_two_points));

% Calcuate apex
[scan_one_apex, inter1] = calculateApex(scan_one, rand_r1, rand_t1);
[scan_two_apex, inter2] = calculateApex(scan_two, rand_r2, rand_t2);
scan_one_apex
scan_two_apex

% Graph apex and scans in lidars' coordinate frames
graphLidar(scan_one, scan_one_apex, inter1, 1)
graphLidar(scan_two, scan_two_apex, inter2, 2)

figure('Name','Two Lidar Simulation [Target]'); hold on; grid on;

% Graph target in target's coordinate frame
drawTarget()

% Graph lidar scans in target's coordinate frame
graphToLidar(scan_one, scan_one_apex, inter1, rand_r1, rand_t1, 1)
graphToLidar(scan_two, scan_two_apex, inter2, rand_r2, rand_t2, 2)

[guess_r, guess_t] = least_squares(scan_one_apex, scan_two_apex)

real_r = rand_r2*rand_r1'
real_t = -real_r*rand_t1+rand_t2

end % function sick_data_process



%==========================================================================
%==========================================================================
function [rand_r rand_t] = generateTransformation(rx,ry,rz)
%==========================================================================
% Func: generateTransformation()
% Desc: Generates a random rotation and translation using the max angles 
%       that are given as rx, ry, and rz
%==========================================================================

% Generate a random rotation for target
rx = rand(1)*pi/8;
ry = rand(1)*pi/8;
rz = rand(1)*2*pi;
Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)];
Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1];
rand_r = Rx*Ry*Rz;

% Generate a random translation for target
rand_t = .1 * rand(3,1);

end % function generateTransformation



%==========================================================================
%==========================================================================
function drawTarget()
%==========================================================================
% Func: drawTarget()
% Desc: Draws the target in its own coordinate plane
%==========================================================================

view([45 45 45])

% Constants used in target generation
a = 1;
x = sqrt(2*a^2)/2;
b = sqrt(a^2/6);
h = sqrt(a^2/3);

% Calculate three points that are originally centered around the origin
% and then apply the random rotation and translation to those points
p1 = [0;sqrt(3/2*a^2)-b;-h];
p2 = [-x;-b;-h];
p3 = [x; -b;-h];

% Apex of the polypod translated and rotated from the origin
apex = [0;0;0];

% Generate and plot lines connecting the polypod legs to the apex
u = linspace(0,1);
plot3([apex(1) p1(1)],[apex(2) p1(2)],[apex(3) p1(3)],'ko-')
plot3([apex(1) p2(1)],[apex(2) p2(2)],[apex(3) p2(3)],'ko-')
plot3([apex(1) p3(1)],[apex(2) p3(2)],[apex(3) p3(3)],'ko-')
plot3(apex(1),apex(2),apex(3),'ko')
plot3(0,0,0,'ko') 

end % function drawTarget



%==========================================================================
%==========================================================================
function [apex, points] = generateTargetTransform(R,T,color)
%==========================================================================
% Func: generateTargetTransform()
% Desc: Generates a random rotation and translation that represent a
%       lidar's coordinate frame relative to the target's coordinate frame
%==========================================================================

view([45 45 45])

% Constants used in target generation
a = 1;
x = sqrt(2*a^2)/2;
b = sqrt(a^2/6);
h = sqrt(a^2/3);

% Calculate three points that are originally centered around the origin
% and then apply the random rotation and translation to those points
p1 = R*[0;sqrt(3/2*a^2)-b;-h]+T;
p2 = R*[-x;-b;-h]+T;
p3 = R*[x; -b;-h]+T;
points = [p1 p2 p3];

% Apex of the polypod translated and rotated from the origin
apex = R*[0;0;0]+T;

if color == 1
    % Generate and plot lines connecting the polypod legs to the apex
    u = linspace(0,1);
    plot3([apex(1) p1(1)],[apex(2) p1(2)],[apex(3) p1(3)],'ro-')
    plot3([apex(1) p2(1)],[apex(2) p2(2)],[apex(3) p2(3)],'ro-')
    plot3([apex(1) p3(1)],[apex(2) p3(2)],[apex(3) p3(3)],'ro-')
    plot3(apex(1),apex(2),apex(3),'ro')
else
    % Generate and plot lines connecting the polypod legs to the apex
    u = linspace(0,1);
    plot3([apex(1) p1(1)],[apex(2) p1(2)],[apex(3) p1(3)],'bo-')
    plot3([apex(1) p2(1)],[apex(2) p2(2)],[apex(3) p2(3)],'bo-')
    plot3([apex(1) p3(1)],[apex(2) p3(2)],[apex(3) p3(3)],'bo-')
    plot3(apex(1),apex(2),apex(3),'bo')
end 

end % function generateTargetTransform



%==========================================================================
%==========================================================================
function [intersections] = generateLidarScan(apex,points)
%==========================================================================
% Func: generateLidarScan()
% Desc: Given the apex and points that represent the target, randomly
%       select a point along each of the legs of the target to represent a 
%       lidar scan
%==========================================================================

p1 = points(:,1);
p2 = points(:,2);
p3 = points(:,3);

% Generate series of points along each side
side1 = [linspace(apex(1),p1(1)); 
         linspace(apex(2),p1(2));
         linspace(apex(3),p1(3))];
     
side2 = [linspace(apex(1),p2(1));
         linspace(apex(2),p2(2));
         linspace(apex(3),p2(3))];
     
side3 = [linspace(apex(1),p3(1));
         linspace(apex(2),p3(2));
         linspace(apex(3),p3(3))];

% Randomly generate points along each line
num = ceil(abs(rand*100));
x = side1(1,num); 
y = side1(2,num);
z = side1(3,num);

num = ceil(abs(rand*100));
x = [x side2(1,num)];
y = [y side2(2,num)];
z = [z side2(3,num)];

num = ceil(abs(rand*100));
x = [x side3(1,num)];
y = [y side3(2,num)];
z = [z side3(3,num)];

intersections = [x;y;z];

end % function generateScan



%==========================================================================
function graphToLidar(points, apex, I, R, T, color)
%==========================================================================
% Func: graphToLidar()
% Desc: Translate the target from its own coordinate frame to a lidar's and
%       plot the result
%==========================================================================


p1 = R\(points(:,1)-T);
p2 = R\(points(:,2)-T);
p3 = R\(points(:,3)-T);
I = R\(I-T);
apex = R\(apex-T);


if color == 1 
    plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'r-*');
    plot3([p2(1) p3(1)],[p2(2) p3(2)],[p2(3) p3(3)],'r-*');
    plot3([p1(1) p3(1)],[p1(2) p3(2)],[p1(3) p3(3)],'r-*');
else 
    plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'b-*');
    plot3([p2(1) p3(1)],[p2(2) p3(2)],[p2(3) p3(3)],'b-*');
    plot3([p1(1) p3(1)],[p1(2) p3(2)],[p1(3) p3(3)],'b-*');
end

plot3(apex(1),apex(2),apex(3),'g*');
plot3(I(1),I(2),I(3),'mo');


end % function graphToLidar



%==========================================================================
function graphLidar(points, apex, I, color)
%==========================================================================
% Func: graphToLidar()
% Desc: Graphs the lidar's coordinate frame
%==========================================================================

p1 = points(:,1);
p2 = points(:,2);
p3 = points(:,3);

if color == 1 
    plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'r-*');
    plot3([p2(1) p3(1)],[p2(2) p3(2)],[p2(3) p3(3)],'r-*');
    plot3([p1(1) p3(1)],[p1(2) p3(2)],[p1(3) p3(3)],'r-*');
else 
    plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'b-*');
    plot3([p2(1) p3(1)],[p2(2) p3(2)],[p2(3) p3(3)],'b-*');
    plot3([p1(1) p3(1)],[p1(2) p3(2)],[p1(3) p3(3)],'b-*');
end

plot3(apex(1),apex(2),apex(3),'g*');
plot3(I(1),I(2),I(3),'mo');


end % function graphToLidar



%==========================================================================
%==========================================================================
function [apex, i2] = calculateApex(scan, R, T)
%==========================================================================
% Func: calculateApex()
% Desc: Given the scan, calculate the apex of the target
%==========================================================================

% calculate magnitude of each vector of lidar scan (distance between the
% base points)

scan 
base1 = norm(scan(:,1)-scan(:,2))
base2 = norm(scan(:,2)-scan(:,3))
base3 = norm(scan(:,3)-scan(:,1))

% Calculate the distance between the apex and each base point
sides = [ 1/2*(2*base3^2-2*base2^2+2*base1^2)^(1/2), 1/2*(2*base2^2-2*base3^2+2*base1^2)^(1/2),  1/2*(2*base2^2+2*base3^2-2*base1^2)^(1/2)];

% Intersection of three spheres to find apex
% http://www.mathworks.com/matlabcentral/newsreader/view_thread/239659
p21 = scan(:,2)-scan(:,1); 
p31 = scan(:,3)-scan(:,1); 
c = cross(p21,p31);
c2 = sum(c.^2);
u1 = cross(((sum(p21.^2)+sides(1)^2-sides(2)^2)*p31 - ...
    (sum(p31.^2)+sides(1)^2-sides(3)^2)*p21)/2,c)/c2;
v = sqrt(sides(1)^2-sum(u1.^2))*c/sqrt(c2);
p1 = scan(:,1);
i1 = p1+u1+v; % intersection point above plane
i2 = p1+u1-v; % intersection point below plane

% Plot the two possible apex points
C = real(i1);
TC = [T C];

no = zeros(3,1);
no(1) = norm(scan(:,1)-C);
no(2) = norm(scan(:,2)-C);
no(3) = norm(scan(:,3)-C);

minNo = min(no);
newscan = zeros(3,3);
newscan(:,1) = (scan(:,1)-C)/no(1)*minNo+C;
newscan(:,2) = (scan(:,2)-C)/no(2)*minNo+C;
newscan(:,3) = (scan(:,3)-C)/no(3)*minNo+C;

m1 = [0 sqrt(2) -1]';
m2 = [-sqrt(6)/2 -1/sqrt(2) -1]';
m3 = [sqrt(6)/2 -1/sqrt(2) -1]';

X =    newscan;
Xhat = [m1 m2 m3];

for i = 1:3
    X(:,i) = X(:,i)-C;
end

[u,s,v] = svd(X*Xhat');
R = v*u';
if det(R)<0, R = v*diag([1 1 -1])*u'; end
R';

apex = C;

end % function calculateApex


%==========================================================================
%==========================================================================
function [guess_r, guess_t]= least_squares(p, p_hat)
%==========================================================================
% Func: least_squares()
% Desc: Using a least squares algorithm, determine the optimal rotation and
%       trasnlation between the two lidars. 
%==========================================================================

% Count the number of data points
num_points = size(p,2);

% Calculate data centroids
centroid_a = sum(p,2)./num_points;
centroid_b = sum(p_hat,2)./num_points;

% Subtract centroids from data
q = p - repmat(centroid_a, 1, num_points);
q_hat = p_hat - repmat(centroid_b, 1, num_points);

% Calculate 3x3 matrix and find rotation
H = q * q_hat'; [U,S,V] = svd(H); guess_r = V*U';
if det(guess_r) < 0, guess_r = V*diag([1, 1, -1])*U'; end
guess_r;

% Calculate translation
guess_t = centroid_b - guess_r*centroid_a;

end % function least_squares



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OLD FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%==========================================================================
%==========================================================================
function [intersections] = generateScan(perpV, point, apex, points)
%==========================================================================
% Func: generateScan()
% Desc: 
% Generate and graph first lidar scan
%scan_one_points = generateScan([0 .1 -1], [0 0 -.2], apex, points);
%scan_one = scan_one_points + snoise*rand(size(scan_one_points));
% Generate and graph second lidar scan
%scan_two_points = generateScan([0 .05 -2], [0 0 -.3], apex, points);
%scan_two = scan_two_points + snoise*rand(size(scan_two_points));
%==========================================================================

% Graph plane
x = -1:.05:1; y = -1:.05:1;
[X,Y] = meshgrid(x,y);
d = perpV(1)*point(1) + perpV(2)*point(2) + perpV(3)*point(3);
Z = ( d - perpV(1)*X - perpV(2)*Y)/perpV(3);
mesh(X,Y,Z)

% Find intersections
[i1, check] = plane_line_intersect(perpV, point, apex', points(:,1)');
[i2, check] = plane_line_intersect(perpV, point, apex', points(:,2)');
[i3, check] = plane_line_intersect(perpV, point, apex', points(:,3)');

% Graph intersections
plot3(i1(1),i1(2),i1(3),'k*')
plot3(i2(1),i2(2),i2(3),'k*')
plot3(i3(1),i3(2),i3(3),'k*')

intersections = [i1 ; i2 ; i3]';

end % function generateScan