KW = KneedWalker; 
Control = Controller(2,10*ones(1,3),zeros(1,3), 0.5*ones(1,3));
Floor = Terrain(0,0);
Sim = Simulation(KW, Control, Floor);
Sim.IC = [0 0 -30/180*pi 190/180*pi 170/180*pi pi 16/18*pi 0 0 0 0 0 0 0 0];

opt = odeset('reltol', 1e-8, 'abstol', 1e-9, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, Xe, Ie] = ode45(@Sim.Derivative, 0:1e-3:10, Sim.IC, opt);
% for ii = 1:length(Time)
%     F(ii,:) = KW.GetReactionForces(X(ii,:).');
% end

Xf = [Sim.Mod.HandleEvent(Ie(end), X(end,Sim.ModCo),Sim.Env),...
      Sim.Con.HandleEvent(Ie(end), X(end,Sim.ConCo),Sim.ConEv)];
if Ie(end) >= Sim.ModEv(2) && Ie(end) <  Sim.ConEv(1) %|| %~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
    EndCond = 1;
end

while ~EndCond
    [tTime, tX, tTe, tXe,tIe] = ode45(@Sim.Derivative, Time(end):1e-3:10, Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
%     for ii = 1:length(tTime)
%         tF(ii,:) = KW.GetReactionForces(tX(ii,:).');
%     end
%     F = [F;tF];
%     Fn = F(:,1)*sin(15/180*pi) + F(:,2)*cos(15/180*pi);
%     tF = [];
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
E = [];
for ii = 1:length(Time)
    E(ii) = Sim.Mod.GetEnergy(X(ii,:));
end
figure()
plot(Time,E)