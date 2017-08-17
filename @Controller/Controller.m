classdef Controller < handle & matlab.mixin.Copyable
    
    properties
        CycleTime = [];
        Amp = [];
        Phase = [];
    end
    
    methods
        % %%%%%% % Class constructor % %%%%%% %
        function C = Controller(varargin)
        C.CycleTime = varargin{1};
        C.Amp = varargin{2}; 
        C.Phase = varargin{3};
        end  
        
        function Torques = Output(C, t, Xm) %#ok
        Torques(1) = C.Amp(1).*sin(2*pi/C.CycleTime(1)*t + C.Phase(1));
        Torques(2) = C.Amp(2).*sin(2*pi/C.CycleTime(2)*t + C.Phase(2));
        Torques(3) = C.Amp(3).*sin(2*pi/C.CycleTime(3)*t + C.Phase(3));
        Torques(4) = C.Amp(4).*sin(2*pi/C.CycleTime(4)*t + C.Phase(1));
        end
    end
end