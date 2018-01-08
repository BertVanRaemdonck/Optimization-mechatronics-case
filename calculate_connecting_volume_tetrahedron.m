function volume = calculate_connecting_volume_tetrahedron(points, X)
% Calculates the total volume you get when you add the volumes of the
% tetrahedrons formed by connecting point X to four consecutive points of 
% the tetrahedron described in the points argument, and that for all 
% possible consecutive points. If the point X lies inside the described 
% shape, the calculated volume is equal to the volume of the shape. If it 
% lies outside of it, the calculated volume will be bigger than the volume 
% of the shape.
%
% Arguments:
%   points: a 4x3 matrix with each column gives the x, y and z coordinate 
%           of a corner of the tetrahedon.
%   X:      a 1x3 vector describing the base point from which the connected
%           volume will be calculated. Optional argument: if it isn't 
%           given, the first point of 'points' will be taken, such that the
%           function automatically returns the volume of the tetrahedon.

    a_point = points(1,:);
    b_point = points(2,:);
    c_point = points(3,:);
    d_point = points(4,:);
    
    if nargin < 2
        % only return the volume of the given tetrahedron
        volume = abs(det([a_point-d_point;
                        b_point-d_point;
                        c_point-d_point]))/6;
    else
        % give the volume that the point X has in regard to the tetrahedron
        volume = zeros(1, 1);
        
        volume = volume + abs(dot(a_point-X, ...
                                (cross(b_point-X,c_point-X))))/6;
        volume = volume + abs(dot(a_point-X, ...
                                (cross(b_point-X,d_point-X))))/6;  
        volume = volume + abs(dot(a_point-X, ...
                                (cross(c_point-X,d_point-X))))/6;
        volume = volume + abs(dot(b_point-X, ...
                                (cross(c_point-X,d_point-X))))/6;        
    end
end