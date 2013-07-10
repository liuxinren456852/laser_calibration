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
[R,T] = least_squares_fitting(l1_apexes, l2_apexes);

end % function calculate_r_t
