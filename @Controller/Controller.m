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
        % assuming that the phase, amplitude and period are symmetrical
        function C = Controller(varargin)
        C.omega  = abs(varargin{1});
        C.Amp    = varargin{2}; 
        C.Phase  = abs(varargin{3});
        C.Period = abs(varargin{4});
        end  
        
        function [Xdot] = Derivative(C, t, X) %#ok
        Xdot = C.omega;
        end
        
        function [value, isterminal, direction] = Events(C, X) %#ok
        isterminal = 1;
        direction  = -1;
        value = 1 - X;
        end
        
        function [Xa] = HandleEvent(C,iEvent,Xi,ConEv) %#ok
        if iEvent == ConEv(1)
            Xa = 0;
        else
            Xa = Xi;
        end
        end
        
        function Torques = Output(C, ~, X)
        % if phase + period > 1, wrap.
        T = C.Amp.*(X >= C.Phase).*(X <= C.Phase + C.Period)...
            + C.Amp.*((C.Phase + C.Period - 1) >= X);
        Torques = T(1,:)+T(2,:);
        end
    end
end