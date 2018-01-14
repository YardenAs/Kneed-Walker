classdef Controller < handle & matlab.mixin.Copyable
    
    properties
        Period  = [];
        Amp     = [];
        Phase   = [];
        omega   = [];
        nEvents = 1;
        Order   = 1;
        Kp      = 0;
        Kd      = 0;
    end
    
    methods
        % %%%%%% % Class constructor % %%%%%% %
        % assuming that the phase, amplitude and period are symmetrical
        function C = Controller(varargin)
        C.omega  = abs(varargin{1});
        C.Amp    = varargin{2}; 
        C.Amp    = [C.Amp(1), C.Amp(1), C.Amp(2), C.Amp(2), C.Amp(3), C.Amp(3);
                    C.Amp(4), C.Amp(4), C.Amp(5), C.Amp(5), zeros(1,2)];
        C.Phase  = abs(varargin{3});
        C.Phase  = [C.Phase(1), C.Phase(1) + 0.5, C.Phase(2), C.Phase(2) + 0.5, C.Phase(3), C.Phase(3) + 0.5;
                    C.Phase(4), C.Phase(4) + 0.5, C.Phase(5), C.Phase(5) + 0.5, zeros(1,2)];
        C.Period = abs(varargin{4});
        C.Period = [C.Period(1), C.Period(1), C.Period(2), C.Period(2), C.Period(3), C.Period(3);
                    C.Period(4), C.Period(4), C.Period(5), C.Period(5), zeros(1,2)];
        C.Kp     = varargin{5}(1);
        C.Kd     = varargin{5}(2);
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
        
        function Torques = Output(C, ~, X, Xkw, support)
        % if phase + period > 1, wrap.
        T = C.Amp.*(X >= C.Phase).*(X <= C.Phase + C.Period)...
            + C.Amp.*((C.Phase + C.Period - 1) >= X);
        Torques = T(1,:)+T(2,:);
        Torques(1) = Torques(1) - strcmp(support,'Left')*(C.Kp*Xkw(3) + C.Kd*Xkw(10));
        Torques(2) = Torques(2) - strcmp(support,'Right')*(C.Kp*Xkw(3) + C.Kd*Xkw(10));
        end
    end
end