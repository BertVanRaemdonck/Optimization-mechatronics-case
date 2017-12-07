classdef  Polynomial < splines.Function
    %POLYNOMIAL 
    %
    %
    %
  methods
    function varargout = type(self,varargin)
    %TYPE 
    %
    %  char = TYPE(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(339, self, varargin{:});
    end
    function varargout = to_string(self,varargin)
    %TO_STRING 
    %
    %  char = TO_STRING(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(340, self, varargin{:});
    end
    function self = Polynomial(varargin)
    %POLYNOMIAL 
    %
    %  new_obj = POLYNOMIAL()
    %  new_obj = POLYNOMIAL([double] coef)
    %  new_obj = POLYNOMIAL([double] coef, std::string & a)
    %  new_obj = POLYNOMIAL([double] coef, char name)
    %
    %
      self@splines.Function(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = splinesMEX(341, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        splinesMEX(342, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
