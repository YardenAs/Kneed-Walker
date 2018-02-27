function [ fit ] = GA_Sim_KW(genome)
%GA_Fit_KW Is the fitness function for the Kneed Walker Genetic Algorithm
%   Simulates the kneed walker with a controller and calculates the fitness
%   based on the robot's performance in the simulation.


net = feedforwardnet(5);
net = configure(net,rand(4,10),rand(2,10));
net = setwb(net, genome.');
C = nnController;
C.net = net;
KW               = KneedWalker;            
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


opt = odeset('reltol', 1e-8, 'abstol', 1e-9, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, ~, Ie] = ode45(@Sim.Derivative, 0:1e-3:10, Sim.IC, opt);
Xf = Sim.Mod.HandleEvent(Ie(end), X(end,Sim.ModCo),Sim.Env);
if Ie(end) >= Sim.ModEv(2) || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
    EndCond = 1;
end

while ~EndCond
    [tTime, tX, tTe, ~,tIe] = ode45(@Sim.Derivative, Time(end):1e-3:10, Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
    Xf = Sim.Mod.HandleEvent(Ie(end), X(end,Sim.ModCo),Sim.Env);
    if Ie(end) >= Sim.ModEv(2) || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
        EndCond = 1;
    end
end
fit = GetFit(Sim.Mod, X, Time, Ie);
end

