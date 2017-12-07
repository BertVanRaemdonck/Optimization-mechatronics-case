function [x,y] = tracking_method_2( data )

    sum_x = sum(data, 1);  % sum datapoints over all columns
    sum_y = sum(data, 2)';  % sum datapoints over all rows
    [max_of_column,x] = max(sum_x);  % column with max value
    [max_of_row,y] = max(sum_y);  % row with max value
    
end
