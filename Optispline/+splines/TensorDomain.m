classdef  TensorDomain < splines.SharedObject
    %TENSORDOMAIN 
    %
    %
    %
  methods
    function varargout = arguments(self,varargin)
    %ARGUMENTS 
    %
    %  [char] = ARGUMENTS(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(186, self, varargin{:});
    end
    function varargout = argument(self,varargin)
    %ARGUMENT 
    %
    %  char = ARGUMENT(self, int index)
    %
    %
      [varargout{1:nargout}] = splinesMEX(187, self, varargin{:});
    end
    function varargout = indexArgument(self,varargin)
    %INDEXARGUMENT 
    %
    %  int = INDEXARGUMENT(self, char a)
    %
    %
      [varargout{1:nargout}] = splinesMEX(188, self, varargin{:});
    end
    function varargout = hasArguments(self,varargin)
    %HASARGUMENTS 
    %
    %  bool = HASARGUMENTS(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(189, self, varargin{:});
    end
    function varargout = type(self,varargin)
    %TYPE 
    %
    %  char = TYPE(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(190, self, varargin{:});
    end
    function varargout = n_domains(self,varargin)
    %N_DOMAINS 
    %
    %  int = N_DOMAINS(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(191, self, varargin{:});
    end
    function varargout = domains(self,varargin)
    %DOMAINS 
    %
    %  [Domain] = DOMAINS(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(192, self, varargin{:});
    end
    function varargout = domain(self,varargin)
    %DOMAIN 
    %
    %  Domain = DOMAIN(self)
    %  Domain = DOMAIN(self, index ind)
    %  Domain = DOMAIN(self, char a)
    %
    %
      [varargout{1:nargout}] = splinesMEX(193, self, varargin{:});
    end
    function varargout = intersection(self,varargin)
    %INTERSECTION 
    %
    %  TensorDomain = INTERSECTION(self, TensorDomain other)
    %
    %
      [varargout{1:nargout}] = splinesMEX(194, self, varargin{:});
    end
    function varargout = eq(self,varargin)
    %EQ 
    %
    %  bool = EQ(self, TensorDomain other)
    %
    %
      [varargout{1:nargout}] = splinesMEX(195, self, varargin{:});
    end
    function self = TensorDomain(varargin)
    %TENSORDOMAIN 
    %
    %  new_obj = TENSORDOMAIN()
    %  new_obj = TENSORDOMAIN(Domain allSubDomain)
    %  new_obj = TENSORDOMAIN([Domain] allDomain)
    %  new_obj = TENSORDOMAIN([Domain] allDomain, [char] args)
    %
    %
      self@splines.SharedObject(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = splinesMEX(196, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        splinesMEX(197, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
  end
end
