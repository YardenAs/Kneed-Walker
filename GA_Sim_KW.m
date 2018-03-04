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
KW.to = [5 0 0]; % set the torso as a point mass
Floor = Terrain(0,0);
Sim = Simulation(KW, Control, Floor);
Sim.IC = [0 0 17.63/18*pi 17/18*pi 17.63/18*pi 17/18*pi 0 0 0 0 0 0 0];
opt = odeset('reltol', 1e-7, 'abstol', 1e-7, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, Xe, Ie] = ode45(@Sim.Derivative, 0:1e-3:10, Sim.IC, opt);


Xf = [Sim.Mod.HandleEvent(Ie(end), X(end,Sim.ModCo),Sim.Env),...
      Sim.Con.HandleEvent(Ie(end), X(end,Sim.ConCo),Sim.ConEv)];
if (Ie(end) >= Sim.ModEv(3) && Ie(end) <  Sim.ConEv(1)) || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
    EndCond = 1;
end

while ~EndCond
    [tTime, tX, tTe, tXe,tIe] = ode45(@Sim.Derivative, Time(end):1e-3:10, Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
    Xe = [Xe; tXe]; %#ok
    Xf = [Sim.Mod.HandleEvent(Ie(end), X(end,Sim.ModCo),Sim.Env),...
          Sim.Con.HandleEvent(Ie(end), X(end,Sim.ConCo),Sim.ConEv)];
  
    if (Ie(end) >= Sim.ModEv(3) && Ie(end) <  Sim.ConEv(1)) || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
        EndCond = 1;
    end
end

fit = GetFit(Sim.Mod, Xe, Ie, X);

end

