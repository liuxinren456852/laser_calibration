%==========================================================================
%==========================================================================
function [guess_r, guess_t]= least_squares_fitting(p, p_hat)
%==========================================================================
% Func: least_squares()
% Desc: 
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