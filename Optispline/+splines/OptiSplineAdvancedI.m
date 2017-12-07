classdef  OptiSplineAdvancedI < SwigRef
    %OPTISPLINEADVANCEDI 
    %
    %
    %
  methods
    function this = swig_this(self)
      this = splinesMEX(3, self);
    end
    function varargout = set_initial(self,varargin)
    %SET_INITIAL 
    %
    %  SET_INITIAL(self, MTensor t, DTensor d)
    %  SET_INITIAL(self, Coefficient c, DTensor d)
    %  SET_INITIAL(self, Function f, Function g)
    %
    %
      [varargout{1:nargout}] = splinesMEX(353, self, varargin{:});
    end
    function varargout = set_value(self,varargin)
    %SET_VALUE 
    %
    %  SET_VALUE(self, MTensor t, DTensor d)
    %  SET_VALUE(self, Coefficient c, DTensor d)
    %  SET_VALUE(self, Function f, Function g)
    %
    %
      [varargout{1:nargout}] = splinesMEX(354, self, varargin{:});
    end
    function varargout = Function(self,varargin)
    %FUNCTION 
    %
    %  Function = FUNCTION(self, TensorBasis b, [int] shape, char attribute)
    %
    %
      [varargout{1:nargout}] = splinesMEX(355, self, varargin{:});
    end
    function self = OptiSplineAdvancedI(varargin)
    %OPTISPLINEADVANCEDI 
    %
    %  new_obj = OPTISPLINEADVANCEDI()
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = splinesMEX(356, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        splinesMEX(357, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
