classdef ControllerOrd2Seg < handle & matlab.mixin.Copyable
    % defines the controller parameters
    
    
    properties
        poly1 = [];
        poly2 = [];
        poly3 = [];
        poly4 = [];
        EndTime = [];
        tau_max = 10;
    end
    
    methods
        % Class constructor
        function [C] = ControllerOrd2Seg(varargin)
            C.EndTime = varargin{1};
            C.poly1 = varargin{2};
            C.poly2 = varargin{3};
            C.poly3 = varargin{4};
            C.poly4 = varargin{5};
        end
        
        function tau = Output(t, Xc, Xm)
            if t < C.EndTime(1)
                tau = polyval(C.poly1,t);
            elseif t >= C.EndTime(1) && t < C.EndTime(2)
                tau = polyval(C.poly2,t-C.EndTime(1));
            elseif t >= C.EndTime(2) && t < C.EndTime(3)
                tau = polyval(C.poly3,t-C.EndTime(2));                
            elseif t >= C.EndTime(3)
                tau = polyval(C.poly4,t-C.EndTime(3));
            end
        end
        
    end
end