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
omega      = rand(1,1);
Amplitudes = rand(1,3);
Phases     = rand(1,3);
Periods    = rand(1,3);
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
figure()
for ii = 1:length(Time)-1
    Sim.RenderSim(X(ii,:),-1,5);
    dt = 1e-3;
    drawnow;
    pause(dt);
end
