function varargout = spline_vertcat(varargin)
    %SPLINE_VERTCAT 
    %
    %  Function = SPLINE_VERTCAT([Function] args)
    %
    %
  [varargout{1:nargout}] = splinesMEX(384, varargin{:});
end
