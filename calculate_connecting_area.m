function area = calculate_connecting_area(points, X)
% Calculates the total area you get when you add the areas of the triangles
% you get by connecting point X to two consecutive points of the shape
% described in the points argument, and that for all possible consecutive
% pairs. If the point X lies inside the described shape, the calculated
% area is equal to the area of the shape. If it lies outside of it, the
% area will be bigger than that area.
%
% Arguments:
%   points: a 2xN matrix with N>=2 (N=2 defines a line so the returned area
%           will only be nonzero if 'X' doesn't lie on that line). Each
%           column gives the x and y coordinate of a corner of a polygon. 
%           The points have to be provided in consecutive order as they are 
%           encountered while traveling along the outline of the polygon. 
%           The polygon must be convex. Required argument.
%   X:      a 2x1 vector describing the base point from which the connected
%           area will be calculated. Optional argument: if it isn't given,
%           the first point of 'points' will be taken, such that the
%           function automatically returns the area of the polygon.

    if nargin < 2
        X = points(:,1);
    end

    nb_points = size(points,2);
    area = zeros(1, size(X,2));
    
    for i=1:nb_points
        point_1 = points(:,i);
        point_2 = points(:,mod(i,nb_points)+1); % always gives the next point in the list, except for when i is the lest element in the list: then it returns the first one
        area = area + .5 * abs((X(1,:)-point_2(1)).*(point_1(2)-X(2,:)) - (X(1,:)-point_1(1)).*(point_2(2)-X(2,:)));
    end
    
end