%% Optimization
nParams = 16;
AmpL = [5 0 0 -20 -20];
AmpU = [20 20 20 0 -2];
PhaseL = [0.5 0 0 0 0.5];
PhaseU = [0.6 0.2 0.2 0.2 0.7];
PeriodL = [0.2 0 0 0 0.1];
PeriodU = 0.5*ones(1,5);
omega = 0.6;
LB = [0.4, AmpL, PhaseL, PeriodL];
UB = [omega, AmpU, PhaseU, PeriodU];

options = gaoptimset('UseParallel',true,'PlotFcns',{@gaplotbestf,@gaplotbestindiv}...
    ,'CrossoverFraction',0.8,'PopulationSize',10000);
[GAsol, fit] = ga(@GA_Sim_KW,nParams,[],[],[],[],LB,UB,[],[],options);
c = clock;
save(['Workspaces/GAsol_fit' num2str(fit) '_d' num2str(c(3)) '_h' num2str(c(4)) '_m' num2str(c(5)) '.mat'],'GAsol');

%% Simulation
Control_Params = GAsol;
omega      = Control_Params(1);
Amplitudes = Control_Params(2:6);
Phases     = Control_Params(7:11);
Periods    = Control_Params(12:16);
Control    = Controller(omega,Amplitudes,Phases,Periods);

KW = KneedWalker;
Floor = Terrain(0,0);
Sim = Simulation(KW, Control, Floor);
Sim.IC = [0 0 0/180*pi 190/180*pi 170/180*pi pi 160/180*pi 0 0 0 0 0 0 0 0];

opt = odeset('reltol', 1e-8, 'abstol', 1e-9, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, Xe, Ie] = ode45(@Sim.Derivative, 0:1e-3:10, Sim.IC, opt);
Xf = [Sim.Mod.HandleEvent(Ie(end), X(end,Sim.ModCo),Sim.Env),...
      Sim.Con.HandleEvent(Ie(end), X(end,Sim.ConCo),Sim.ConEv)];
if (Ie(end) >= Sim.ModEv(2) && Ie(end) <  Sim.ConEv(1)) || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
    EndCond = 1;
end
while ~EndCond
    [tTime, tX, tTe, tXe,tIe] = ode45(@Sim.Derivative, Time(end):1e-3:10, Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
    Xf = [Sim.Mod.HandleEvent(Ie(end), X(end,Sim.ModCo),Sim.Env),...
          Sim.Con.HandleEvent(Ie(end), X(end,Sim.ConCo),Sim.ConEv)];
  
    if (Ie(end) >= Sim.ModEv(2) && Ie(end) <  Sim.ConEv(1)) || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
        EndCond = 1;
    end
end
figure(1); clf;
for ii = 1:length(Time)-1
    Sim.RenderSim(X(ii,:),-1,5);
    pause(1e-2);
    drawnow;
end
E = [];
T = [];
for ii = 1:length(Time)
    E(ii) = Sim.Mod.GetEnergy(X(ii,:));
    T(:,ii) = Control.Output(Time(ii),X(ii,15));
end
figure(2)
plot(Time,E)
figure(3)
plot(Time,T(1:3,:).',Time,T(4:6,:).',[1/GAsol(1) 1/GAsol(1)],[-15,15],'--k')
% ylim([-15,15])
legend('LHip','RHip','LKnee','RKnee','LAnkle','RAnkle')
figure(4)
phi = linspace(0,1,1000);
T = [];
for ii = 1:1000
    T(:,ii) = Control.Output(phi(ii),phi(ii));
end
plot(phi,T(1:3,:).',phi,T(4:6,:).')
% ylim([-15,15])
legend('LHip','RHip','LKnee','RKnee','LAnkle','RAnkle')