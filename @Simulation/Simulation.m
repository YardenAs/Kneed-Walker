classdef Simulation < handle & matlab.mixin.Copyable
    % Version 0.2 - 10/05/2014
    % This simulation integrates a system over time until
    % an event occurs, then it performs some calculations
    % and continues integrating until the next event or
    % it runs out of time
    
    properties
        Mod; % Model
        Con; % Controller
        Env; % Environment
    
        % State params
        stDim; ModCo; ConCo;
        % Event params
        nEvents; ModEv; ConEv;
    
        % Simulation parameters
        IC;
        EndCond = 0;
    end
    
    methods
        % %%%%%% % Class constructor % %%%%%% %
        function sim = Simulation(varargin)
            switch nargin
                case 3
                    sim.Mod = varargin{1};
                    sim.Con = varargin{2};
                    sim.Env = varargin{3};
                    sim.ModEv = 1:sim.Mod.nEvents;
                    sim.ModCo = 1:sim.Mod.Order;
                otherwise
                    sim.Mod = KneedWalker();
                    sim.Con = Controller();
                    sim.Env = Terrain();
            end            
        end

        function [Xt] = Derivative(sim,t,X)
            sim.Mod.Torques = sim.Con.Output(t, X(sim.ModCo));
            Xt = sim.Mod.Derivative(t,X(sim.ModCo));
        end

        function [value, isterminal, direction] = Events(sim, t, X) %#ok<INUSL>
            value = zeros(sim.nEvents,1);
            isterminal = ones(sim.nEvents,1);
            direction = zeros(sim.nEvents,1);
            [value(sim.ModEv), isterminal(sim.ModEv), direction(sim.ModEv)] = ...
                sim.Mod.Events(X(sim.ModCo), sim.Env);
        end
    end
end


