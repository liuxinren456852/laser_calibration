%==========================================================================
%==========================================================================
function [apex, i2] = calculate_apex(points, R, T)
%==========================================================================
% Func: calculate_apex()
% Desc: 
%==========================================================================

% calculate magnitude of each vector of lidar points (distance between the
% base points)

base1 = norm(points(:,1)-points(:,2));
base2 = norm(points(:,2)-points(:,3));
base3 = norm(points(:,3)-points(:,1));

% Calculate the distance between the apex and each base point
sides = [ 1/2*(2*base3^2-2*base2^2+2*base1^2)^(1/2), 1/2*(2*base2^2-2*base3^2+2*base1^2)^(1/2),  1/2*(2*base2^2+2*base3^2-2*base1^2)^(1/2)];

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

% Plot the two possible apex points
C = real(i1);
TC = [T C];

no = zeros(3,1);
no(1) = norm(points(:,1)-C);
no(2) = norm(points(:,2)-C);
no(3) = norm(points(:,3)-C);

minNo = min(no);
newscan = zeros(3,3);
newscan(:,1) = (points(:,1)-C)/no(1)*minNo+C;
newscan(:,2) = (points(:,2)-C)/no(2)*minNo+C;
newscan(:,3) = (points(:,3)-C)/no(3)*minNo+C;

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

end % function calculate_apex