%==========================================================================
%==========================================================================
function [apex, i2] = calculate_apex(points)
%==========================================================================
%==========================================================================
%
%  File: calculate_apex.m
%  Auth: Justin Cosentinum
%  Date: 08 July 2013
%
%  In:   points     - the points of the lidar scan that intersect with the
%
%  Out:  apex - the calculated apex of the target given the intersections
%        i2   - the other possible apex of the targer - ignored because we
%               assume that the target is in an upright position
%  
%  Desc: Given two lidars, generate_data will collect n scans and calculate
%        the apex of a target in regards to each lidars' coordinate frame.
%
%  Usage: calculate_apex(points, test_num, lidar_num, pose_num, write_flag)
%
%==========================================================================

points = [points ; 0 0 0];

% calculate magnitude of each vector of lidar points (distance between the
% base points)

base1 = norm(points(:,1)-points(:,2));
base2 = norm(points(:,2)-points(:,3));
base3 = norm(points(:,3)-points(:,1));

% Calculate the distance between the apex and each base point
sides = [ 1/2*(2*base3^2-2*base2^2+2*base1^2)^(1/2), ...
          1/2*(2*base2^2-2*base3^2+2*base1^2)^(1/2), ...
          1/2*(2*base2^2+2*base3^2-2*base1^2)^(1/2)];

% Intersection of three spheres to find apex
% http://www.mathworks.com/matlabcentral/newsreader/view_thread/239659
p21 = points(:,2)-points(:,1); 
p31 = points(:,3)-points(:,1); 
c = cross(p21,p31);
c2 = sum(c.^2);
u1 = cross(((sum(p21.^2)+sides(1)^2-sides(2)^2)*p31 - ...
    (sum(p31.^2)+sides(1)^2-sides(3)^2)*p21)/2,c)/c2;
v = sqrt(sides(1)^2-sum(u1.^2))*c/sqrt(c2);
p1 = points(:,1);
i1 = p1+u1+v; % intersection point above plane
i2 = p1+u1-v; % intersection point below plane

C = real(i1);
apex = C;
apex(3) = abs(apex(3));

end % function calculate_apex