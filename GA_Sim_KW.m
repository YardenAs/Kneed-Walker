function [ fit ] = GA_Sim_KW(genome)
%GA_Fit_KW Is the fitness function for the Kneed Walker Genetic Algorithm
%   Simulates the kneed walker with a controller and calculates the fitness
%   based on the robot's performance in the simulation.

hidden_sizes    = 5;
C               = nnController(hidden_sizes,{'tansig'});
KW              = KneedWalker;
C.net.numInputs  = 8;
C.net.numOutputs = 2;
for ii = 1:hidden_sizes(1)
    C.net.IW{1}   = [C.net.IW{1};
        genome((ii*C.net.numInputs - C.net.numInputs + 1):ii*C.net.numInputs)];
end

Floor = Terrain(0,0);
Sim = Simulation(KW, C, Floor);
Sim.IC = [0 0 0/180*pi 190/180*pi 170/180*pi 190/180*pi 170/180*pi 0 0 0 0 0 0 0 0];

opt = odeset('reltol', 1e-8, 'abstol', 1e-9, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, ~, Ie] = ode45(@Sim.Derivative, 0:1e-3:10, Sim.IC, opt);


Xf = [Sim.Mod.HandleEvent(Ie(end), X(end,Sim.ModCo),Sim.Env),...
      Sim.Con.HandleEvent(Ie(end), X(end,Sim.ConCo),Sim.ConEv)];
if Ie(end) >= Sim.ModEv(2) && Ie(end) <  Sim.ConEv(1) || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
    EndCond = 1;
end

while ~EndCond
    [tTime, tX, tTe, ~,tIe] = ode45(@Sim.Derivative, Time(end):1e-3:10, Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
    Xf = [Sim.Mod.HandleEvent(Ie(end), X(end,Sim.ModCo),Sim.Env),...
          Sim.Con.HandleEvent(Ie(end), X(end,Sim.ConCo),Sim.ConEv)];
  
    if Ie(end) >= Sim.ModEv(2) && Ie(end) <  Sim.ConEv(1) || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
        EndCond = 1;
    end
end

fit = GetFit(Sim.Mod, X, Time, Ie);

end

