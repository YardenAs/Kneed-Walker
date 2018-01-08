%% Optimization
nParams = 13;
Amp = 20*ones(1,4);
Phase = ones(1,4);
Period = 0.5*ones(1,4);
omega = 2;
LB = [0, -Amp, 0*Phase,0*Period];
UB = [omega, Amp, Phase, Period];

options = gaoptimset('UseParallel',true,'PlotFcns',{@gaplotbestf,@gaplotbestindiv}...
    ,'MutationFcn',{@mutationuniform, 0.1},'CrossoverFraction',0.6,'PopulationSize',800);
[GAsol, fit] = ga(@GA_Sim_KW,nParams,[],[],[],[],LB,UB,[],[],options);
c = clock;
save(['Workspaces/GAsol_fit' num2str(fit) '_d' num2str(c(3)) '_h' num2str(c(4)) '_m' num2str(c(5)) '.mat'],'GAsol');

%% Simulation
KW = KneedWalker; 
Control = Controller(GAsol(1),GAsol(2:5),GAsol(6:9),GAsol(10:13));
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
figure(1)
for ii = 1:length(Time)-1
    Sim.RenderSim(X(ii,:),-1,5);
    pause(1e-3);
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
ylim([-15,15])
legend('LHip','RHip','LKnee','RKnee','LAnkle','RAnkle')