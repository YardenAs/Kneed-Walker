classdef Controller < handle & matlab.mixin.Copyable
    
    properties
        Period  = [];
        Amp     = [];
        Phase   = [];
        omega   = [];
        nEvents = 1;
        Order   = 1;
    end
    
    methods
        % %%%%%% % Class constructor % %%%%%% %
        function C = Controller(varargin)
        C.omega  = varargin{1};
        C.Amp    = varargin{2}; 
        C.Phase  = varargin{3};
        C.Period = varargin{4};
        end  
        
        function [Xdot] = Derivatives(C, t, X) %#ok
        Xdot = C.omega;
        end
        
        function [value, isterminal, direction] = Events(C, X) %#ok
        isterminal = 1;
        direction  = 1;
        value = 1 - X;
        end
        
        function [C, Xa] = HandleEvents(C)
        Xa = 0;
        end
        
        function Torques = Output(C, ~, X)
        Torques = (X >= C.Phase).*C.Amp + (X <= X.Phase + C.Period).*C.Amp; 
        end
    end
end