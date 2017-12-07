function dx = calc_der1( x, N, h) %First order finite difference

dx = (x(:,2:N)-x(:,1:N-1))/h;    % finite difference
dx = [dx ,dx(:,end)];          % set last element equal to second last element,
                             % to obtain vectors of equal length

end

