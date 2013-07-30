function least_squares_fitting(p, snoise)

% Clear current window and figures
clc
%clf

% Generate random rotation matrix
rand_x = rand(3,3);
[u,s,v]=svd(rand_x); rand_r = v*u';
if det(rand_r) < 0,  rand_r = v*diag([1 1 -1])*u; end
rand_r

% Generate random translation vector
rand_t = rand(3,1)

% Count the number of data points
num_points = size(p,2)

% Use random matrix and vector to calculate second point set
p_hat = rand_r*p + repmat(rand_t,1,num_points) + snoise*rand(size(p));

% Calculate data centroids
centroid_a = sum(p,2)./num_points;
centroid_b = sum(p_hat,2)./num_points;

% Subtract centroids from data
q = p - repmat(centroid_a, 1, num_points);
q_hat = p_hat - repmat(centroid_b, 1, num_points);

% Calculate 3x3 matrix and find rotation
H = q * q_hat'; [U,S,V] = svd(H); guess_r = V*U';
if det(guess_r) < 0, guess_r = V*diag([1, 1, -1])*U'; end
guess_r

% Calculate translation
guess_t = centroid_b - guess_r*centroid_a

end

