classdef Controller < handle & matlab.mixin.Copyable
    
    properties
        CycleTime = [];
        Amp = [];
        Phase = [];
        Ctime = [];
        CT_hip = [];
        CT_ank1 = [];
        CT_ank2 = [];
    end
    
    methods
        % %%%%%% % Class constructor % %%%%%% %
        % assuming that the phase, amplitude and period are symmetrical
        function C = Controller(varargin)
        load('CB_torques.mat');
        C.CT_ank1 = T_ank1;
        C.CT_ank2 = T_ank2;
        C.CT_hip  = T_hip;
        C.Ctime   = time;
        end  
        
        function Torques = Output(C, t, Xm) %#ok
        %         Torques(1) = C.Amp(1).*sin(2*pi/C.CycleTime(1)*t + C.Phase(1));
        %         Torques(2) = C.Amp(2).*sin(2*pi/C.CycleTime(2)*t + C.Phase(2));
        %         Torques(3) = C.Amp(3).*sin(2*pi/C.CycleTime(3)*t + C.Phase(3));
        %         Torques(4) = C.Amp(4).*sin(2*pi/C.CycleTime(4)*t + C.Phase(1));
        %         Torques(5) =
        %         Torques(6) =
        ind = find(abs(t-C.Ctime)<1e-2);
        Torques(1) = C.CT_hip(ind(1));
        Torques(2) = C.CT_ank1(ind(1));
        Torques(3) = C.CT_ank2(ind(1));
        end
    end
end