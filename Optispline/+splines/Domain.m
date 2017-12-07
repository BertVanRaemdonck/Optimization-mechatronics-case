classdef  Domain < splines.SharedObject
    %DOMAIN 
    %
    %
    %
  methods
    function varargout = data(self,varargin)
    %DATA 
    %
    %  [AnyScalar] = DATA(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(176, self, varargin{:});
    end
    function varargout = type(self,varargin)
    %TYPE 
    %
    %  char = TYPE(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(177, self, varargin{:});
    end
    function varargout = intersection(self,varargin)
    %INTERSECTION 
    %
    %  Domain = INTERSECTION(self, Domain dom)
    %  Domain = INTERSECTION(self, interval dom)
    %
    %
      [varargout{1:nargout}] = splinesMEX(178, self, varargin{:});
    end
    function varargout = eq(self,varargin)
    %EQ 
    %
    %  bool = EQ(self, Domain other)
    %  bool = EQ(self, interval other)
    %
    %
      [varargout{1:nargout}] = splinesMEX(179, self, varargin{:});
    end
    function self = Domain(varargin)
    %DOMAIN 
    %
    %  new_obj = DOMAIN()
    %
    %
      self@splines.SharedObject(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = splinesMEX(180, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        splinesMEX(181, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
