% nParams must be equal to the number of weights - therefore check that the
% number of weights in GA_Sim_KW matches this size.
nParams = 12;

% LB = [0.2, -10*ones(1,3), [0 0 0.4] ,[0.1 0.1 0.1],[16/18*pi 14/18*pi]];
% UB = [0.6, 10*ones(1,3), [0.6 0.2 1] , ones(1,3),[20/18*pi 18/18*pi]];
load('Workspaces\GAsol_fit-7.031_d14_h16_m36.mat');
LB = GAsol - 0.3*abs(GAsol);
UB = GAsol + 0.3*abs(GAsol);
options = gaoptimset('UseParallel',true,'PlotFcns',{@gaplotbestf,@gaplotbestindiv}...
    ,'PopulationSize',3000);
[GAsol, fit] = ga(@GA_Sim_KW,nParams,[],[],[],[],LB,UB,[],[],options);
c = clock;
save(['Workspaces/GAsol_fit' num2str(fit) '_d' num2str(c(3)) '_h' num2str(c(4)) '_m' num2str(c(5)) '.mat'],'GAsol');

%% Simulate Results %%
Control_Params = GAsol;
mega      = Control_Params(1);
Amplitudes = Control_Params(2:4);
Phases     = Control_Params(5:7);
Periods    = Control_Params(8:10);
dt         = 1e-3;
StopTime   = 20;
Control    = Controller(omega,Amplitudes,Phases,Periods);
KW = KneedWalker;
KW.to = [10 0 0]; % set the torso as a point mass
Floor = Terrain(0,0);
Sim = Simulation(KW, Control, Floor);
Sim.IC = [0 0 Control_Params(11) Control_Params(12) Control_Params(11) Control_Params(12) 0 0 0 0 0 0 0];
opt = odeset('reltol', 1e-8, 'abstol', 1e-9, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, Xe, Ie] = ode45(@Sim.Derivative, 0:dt:StopTime, Sim.IC, opt);

Xf = [Sim.Mod.HandleEvent(Ie(end), X(end,Sim.ModCo),Sim.Env),...
    Sim.Con.HandleEvent(Ie(end), X(end,Sim.ConCo),Sim.ConEv)];
if (Ie(end) >= Sim.ModEv(3) && Ie(end) <  Sim.ConEv(1)) || Time(end) >= StopTime-dt% || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
    EndCond = 1;
end

while ~EndCond
    [tTime, tX, tTe, tXe,tIe] = ode45(@Sim.Derivative, Time(end):dt:StopTime, Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
    Xe = [Xe; tXe]; %#ok
    Xf = [Sim.Mod.HandleEvent(Ie(end), X(end,Sim.ModCo),Sim.Env),...
        Sim.Con.HandleEvent(Ie(end), X(end,Sim.ConCo),Sim.ConEv)];
    if (Ie(end) >= Sim.ModEv(3) && Ie(end) <  Sim.ConEv(1)) || Time(end) >= StopTime-dt% || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
        EndCond = 1;
    end
end
GetFit(Sim.Mod,Xe,Ie,X)
figure()
for ii = 1:length(Time)-1
    Sim.RenderSim(X(ii,:),-1,5);
    dt = Time(ii+1) - Time(ii);
    drawnow;
    pause(dt);
end
T = [];
for ii = 1:length(Time)
    T(ii,:) = Control.Output(Time(ii),[],X(ii,13));
end
% figure()
% plot(Time, [X(:,3) - X(:,5), X(:,4) - X(:,6)]);
% xlabel('Time [sec]'); ylabel('\Delta\theta [rad]');
figure()
plot(Time, T.')
legend('Hip','Ankle')