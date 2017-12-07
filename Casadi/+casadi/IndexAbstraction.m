classdef  IndexAbstraction < SwigRef
    %INDEXABSTRACTION 
    %
    %   = INDEXABSTRACTION()
    %
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function v = start(self)
      v = casadiMEX(1125, self);
    end
    function v = stop(self)
      v = casadiMEX(1126, self);
    end
    function self = IndexAbstraction(varargin)
    %INDEXABSTRACTION 
    %
    %  new_obj = INDEXABSTRACTION()
    %
    %
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1127, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        casadiMEX(1128, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
