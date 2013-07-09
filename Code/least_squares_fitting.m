%==========================================================================
%==========================================================================
function [guess_r, guess_t]= least_squares_fitting(p, p_hat)
%==========================================================================
%
%  File: least_squares_fitting.m
%  Auth: Justin Cosentino
%  Date: 08 July 2013
%
%  In: p -     A  number of [x, y, z] points representing the target's apex
%              at various poses
%      p_hat - The corresponding [x, y, z] points that represent the same
%              apex in the second lidar's coordinate frame 
%
%  Out: guess_r - The optimal rotation matrix calculated by the least
%                 squares method
%       guess_t - The optimal translation vector calculated by the least
%                 squares method
%
%  Desc: Using a least squares algorithm, calculate an optimal homogeneous
%        transformation from one lidar's coordinate system to the other.
%
%  Usage: least_squares_fitting(p, p_hat)
%
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

end % function least_squares_fitting