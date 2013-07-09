function [intersections, indices] = segment_lines(points)
%==========================================================================
%==========================================================================
%
%  File: segment_lines.m
%  Auth: Justin Cosentino
%  Date: 08 July 2013
%
%  In:
%
%  Out: 
%
%  Desc:
%
%==========================================================================

% Error Checking
if ~(nargin == 1)
    help splitAndMerge
    return
end

% Make first split
num_points = size(points,2);
first = points(:,1);
last  = points(:,num_points);
index1 = splitIndex(first, last, points)
split1 = points(:,index1);

% Make second splits
index2 = splitIndex(first, split1, points(:, 1:index1))
split2 = points(:,index2);

index3 = splitIndex(split1, last, points(:,index1:num_points)) + index1 -1;
split3 = points(:,index3);

indices = [1 ; index2 ; index1 ; index3 ; num_points];
intersections = [first split2 split1 split3 last];

%==========================================================================
%==========================================================================
function [index] = splitIndex(first,last, points)
%==========================================================================
% Func: ()
% Desc: 
%==========================================================================

% Fit line to first and last point
v = last-first
v_perp = [-v(2),v(1)]
line = v_perp/norm(v_perp)

n = size(points,2);

maxDistance = 0; index = 0;
for i=1:n
    currentPoint = points(:,i);
    distance = dot(line, currentPoint-first);
    if abs(distance) > maxDistance
        maxDistance = abs(distance);
        index = i;
    end
end
 
end % function splitIndex

end % function segment_lines