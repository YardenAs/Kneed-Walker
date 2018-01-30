function [ fit ] = GA_Sim_KW(Control_Params)
%GA_Fit_KW Is the fitness function for the Kneed Walker Genetic Algorithm
%   Simulates the kneed walker with a controller and calculates the fitness
%   based on the robot's performance in the simulation.


omega      = Control_Params(1);
Amplitudes = Control_Params(2:4);
Phases     = Control_Params(5:7);
Periods    = Control_Params(8:10);
Control    = Controller(omega,Amplitudes,Phases,Periods);

KW = KneedWalker;
Floor = Terrain(0,0);
Sim = Simulation(KW, Control, Floor);
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

