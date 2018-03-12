KW = KneedWalker; 
Control_Params = rand(1,16);
omega      = Control_Params(1);
Amplitudes = Control_Params(2:6);
Phases     = Control_Params(7:11);
Periods    = Control_Params(12:16);
Control    = Controller(omega,Amplitudes,Phases,Periods);
Floor = Terrain(0,0);
Sim = Simulation(KW, Control, Floor);
Sim.IC = [0 0 0/180*pi 190/180*pi 170/180*pi 190/180*pi 170/180*pi 0 0 0 0 0 0 0 0];

opt = odeset('reltol', 1e-8, 'abstol', 1e-9, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, Xe, Ie] = ode45(@Sim.Derivative, 0:1e-3:10, Sim.IC, opt);

Xf = [Sim.Mod.HandleEvent(Ie(end), X(end,Sim.ModCo),Sim.Env),...
      Sim.Con.HandleEvent(Ie(end), X(end,Sim.ConCo),Sim.ConEv)];
if Ie(end) >= Sim.ModEv(2) && Ie(end) <  Sim.ConEv(1) %|| %~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
    EndCond = 1;
end

while ~EndCond
    [tTime, tX, tTe, tXe,tIe] = ode45(@Sim.Derivative, Time(end):1e-3:10, Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
    Xf = [Sim.Mod.HandleEvent(Ie(end), X(end,Sim.ModCo),Sim.Env),...
          Sim.Con.HandleEvent(Ie(end), X(end,Sim.ConCo),Sim.ConEv)];
  
    if Ie(end) >= Sim.ModEv(2) && Ie(end) <  Sim.ConEv(1) %|| ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
        EndCond = 1;
    end
end

for ii = 1:length(Time)-1
    Sim.RenderSim(X(ii,:),-1,5);
    drawnow;
end