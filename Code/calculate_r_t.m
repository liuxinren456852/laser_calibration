function [R, T] =  calculate_r_t()
%==========================================================================
%==========================================================================
%
%  File: calculate_r_t.m
%  Auth: Justin Cosentino
%  Date: 10 July 2013
%
%  In:
%
%  Out:  
%   
%  Desc: 
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
