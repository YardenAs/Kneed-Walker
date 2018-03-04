function [ fit ] = GA_Sim_KW(genome)
%GA_Fit_KW Is the fitness function for the Kneed Walker Genetic Algorithm
%   Simulates the kneed walker with a controller and calculates the fitness
%   based on the robot's performance in the simulation.
C  = Controller(genome(1), genome(2:4), genome(5:7), genome(8:10));
KW = KneedWalker;
KW.to = [5 0 0]; % set the torso as a point mass
Floor = Terrain(0,0);
Sim = Simulation(KW, C, Floor);
%%% State Space %%%
% X1 = x
% X2 = y
% X3 = support thigh angle
% X4 = non support thigh angle
% X5 = support shank angle
% X6 = non support shank angle
% X7-12 respective velocities (X7 = dx...)
% thigh and shank angles and angular velocities must be equal to satisfy
% the locked knees constraint
Sim.IC = [0 0 17.63/18*pi 17/18*pi 17.63/18*pi 17/18*pi 0 0 0 0 0 0];

opt = odeset('reltol', 1e-7, 'abstol', 1e-7, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, ~, Ie] = ode45(@Sim.Derivative, [0 10], Sim.IC, opt);
Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:),Sim.Env); 
if Ie(end) >= 3 || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
    EndCond = 1;
end
while ~EndCond
    [tTime, tX, tTe, ~,tIe] = ode45(@Sim.Derivative, [Time(end) 10], Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
    Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:),Sim.Env); 
    if Ie(end) >= 3 || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
        EndCond = 1;
    end       
end
fit = GetFit(Sim.Mod, X, Time, Ie);
toc;
end

