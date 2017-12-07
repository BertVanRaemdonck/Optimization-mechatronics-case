function [x,y] = tracking_method_1( data )

    [max_of_column,index_in_column] = max(data);  % get max values for each column
    [max_overall,index_in_row] = max(max_of_column);  % get max values for each row, given the column with max value
    x = index_in_row;  % x-value of max data point
    y = index_in_column(index_in_row);  %  y-value of max datapoint

end
