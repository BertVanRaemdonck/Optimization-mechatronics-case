function volume = calculate_connecting_volume_tetrahedon(points, X)
% Calculates the total volume you get when you add the volumes of the
% tetrahedeons you get by connecting point X to four consecutive points of the 
% tetrahedon described in the points argument, and that for all possible consecutive
% points. If the point X lies inside the described shape, the calculated
% volume is equal to the volume of the shape. If it lies outside of it, the
% volume will be bigger than that volume.
%
% volume tetrahedon (a,b,c,d) = det(a-d,b-d,c-d)/6
%
% Arguments:
%   points: a 4x3 matrix with each column gives the x, y and z coordinate 
%           of a corner of the tetrahedon.
%   X:      a 1x3 vector describing the base point from which the connected
%           volume will be calculated. Optional argument: if it isn't given,
%           the first point of 'points' will be taken, such that the
%           function automatically returns the volume of the tetrahedon.

    a_point = points(1,:);
    b_point = points(2,:);
    c_point = points(3,:);
    d_point = points(4,:);
    if nargin < 2
        % X = points(:,1);
        volume = abs(det([a_point-d_point;
                        b_point-d_point;
                        c_point-d_point]))/6;
    else
        
%         nb_points = 4;
        volume = zeros(1, 1);
        
        volume = volume + abs(dot(a_point-X, ...
                                (cross(b_point-X,c_point-X))))/6;
        volume = volume + abs(dot(a_point-X, ...
                                (cross(b_point-X,d_point-X))))/6;  
        volume = volume + abs(dot(a_point-X, ...
                                (cross(c_point-X,d_point-X))))/6;
        volume = volume + abs(dot(b_point-X, ...
                                (cross(c_point-X,d_point-X))))/6;        

%         volume = volume + abs(det([a_point-X;
%                         b_point-X;
%                         c_point-X]))/6;
%         volume = volume + abs(det([a_point-X;
%                         b_point-X;
%                         d_point-X]))/6;
%         volume = volume + abs(det([a_point-X;
%                         c_point-X;
%                         d_point-X]))/6;
%         volume = volume + abs(det([b_point-X;
%                         c_point-X;
%                         d_point-X]))/6;
                    
                    
%         for i=1:nb_points
%             point_1 = points(:,i);
%             point_2 = points(:,mod(i,nb_points)+1); % always gives the next point in the list, except for when i is the lest element in the list: then it returns the first one
%             volume = volume + .5 * abs((X(1,:)-point_2(1)).*(point_1(2)-X(2,:)) - (X(1,:)-point_1(1)).*(point_2(2)-X(2,:)));
%         end
    end
end