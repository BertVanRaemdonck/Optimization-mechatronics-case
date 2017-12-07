classdef  XmlFile < casadi.SharedObject & casadi.PrintableCommon
    %XMLFILE XML parser Can be used for parsing XML files into CasADi data structures.
    %
    %
    %
    %Joel Andersson
    %
    %C++ includes: xml_file.hpp 
    %
  methods
    function this = swig_this(self)
      this = casadiMEX(3, self);
    end
    function delete(self)
      if self.swigPtr
        casadiMEX(1087, self);
        self.swigPtr=[];
      end
    end
    function self = XmlFile(varargin)
    %XMLFILE 
    %
    %  new_obj = XMLFILE()
    %  new_obj = XMLFILE(char name)
    %
    %
      self@casadi.SharedObject(SwigRef.Null);
      self@casadi.PrintableCommon(SwigRef.Null);
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = casadiMEX(1090, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.swigPtr = [];
      end
    end
  end
  methods(Static)
    function varargout = type_name(varargin)
    %TYPE_NAME 
    %
    %  char = TYPE_NAME()
    %
    %
     [varargout{1:nargout}] = casadiMEX(1086, varargin{:});
    end
    function varargout = load_plugin(varargin)
    %LOAD_PLUGIN 
    %
    %  LOAD_PLUGIN(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(1088, varargin{:});
    end
    function varargout = doc(varargin)
    %DOC 
    %
    %  char = DOC(char name)
    %
    %
     [varargout{1:nargout}] = casadiMEX(1089, varargin{:});
    end
  end
end
