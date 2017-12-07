classdef  Coefficient < splines.SharedObject
    %COEFFICIENT 
    %
    %
    %
  methods
    function varargout = shape(self,varargin)
    %SHAPE 
    %
    %  [int] = SHAPE(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(271, self, varargin{:});
    end
    function varargout = dimension(self,varargin)
    %DIMENSION 
    %
    %  [int] = DIMENSION(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(272, self, varargin{:});
    end
    function varargout = type(self,varargin)
    %TYPE 
    %
    %  char = TYPE(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(273, self, varargin{:});
    end
    function varargout = uminus(self,varargin)
    %UMINUS 
    %
    %  Coefficient = UMINUS(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(274, self, varargin{:});
    end
    function varargout = data(self,varargin)
    %DATA 
    %
    %  AnyTensor = DATA(self)
    %  AnyTensor = DATA(self, index k)
    %
    %
      [varargout{1:nargout}] = splinesMEX(275, self, varargin{:});
    end
    function varargout = transform(self,varargin)
    %TRANSFORM 
    %
    %  AnyTensor = TRANSFORM(self, AnyTensor T)
    %  AnyTensor = TRANSFORM(self, AnyTensor T, index direction)
    %  AnyTensor = TRANSFORM(self, AnyTensor T, [index] direction)
    %  AnyTensor = TRANSFORM(self, [AnyTensor] T, [index] direction)
    %
    %
      [varargout{1:nargout}] = splinesMEX(276, self, varargin{:});
    end
    function varargout = transpose(self,varargin)
    %TRANSPOSE 
    %
    %  Coefficient = TRANSPOSE(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(277, self, varargin{:});
    end
    function varargout = rm_direction(self,varargin)
    %RM_DIRECTION 
    %
    %  Coefficient = RM_DIRECTION(self, [int] indices)
    %
    %
      [varargout{1:nargout}] = splinesMEX(278, self, varargin{:});
    end
    function varargout = reshape(self,varargin)
    %RESHAPE 
    %
    %  Coefficient = RESHAPE(self, [int] shape)
    %
    %
      [varargout{1:nargout}] = splinesMEX(279, self, varargin{:});
    end
    function varargout = trace(self,varargin)
    %TRACE 
    %
    %  Coefficient = TRACE(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(280, self, varargin{:});
    end
    function varargout = to_matrix_valued(self,varargin)
    %TO_MATRIX_VALUED 
    %
    %  Coefficient = TO_MATRIX_VALUED(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(281, self, varargin{:});
    end
    function varargout = is_true_scalar(self,varargin)
    %IS_TRUE_SCALAR 
    %
    %  bool = IS_TRUE_SCALAR(self)
    %
    %
      [varargout{1:nargout}] = splinesMEX(283, self, varargin{:});
    end
    function self = Coefficient(varargin)
    %COEFFICIENT 
    %
    %  new_obj = COEFFICIENT()
    %  new_obj = COEFFICIENT([double] v)
    %  new_obj = COEFFICIENT(AnyTensor t)
    %
    %
      self@splines.SharedObject(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = splinesMEX(284, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
    function delete(self)
      if self.swigPtr
        splinesMEX(285, self);
        self.swigPtr=[];
      end
    end
  end
  methods(Static)
    function varargout = cat(varargin)
    %CAT 
    %
    %  Coefficient = CAT(index index, [Coefficient] coefs)
    %
    %
     [varargout{1:nargout}] = splinesMEX(282, varargin{:});
    end
  end
end
