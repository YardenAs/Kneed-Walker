% nParams must be equal to the number of weights - therefore check that the
% number of weights in GA_Sim_KW matches this size. 
nParams = 37;

% LB = [0, -Amp, zeros(1,3),zeros(1,3)];
% UB = [omega, Amp, Phase, Period];
options = gaoptimset('UseParallel',true,'PlotFcns',{@gaplotbestf,@gaplotbestindiv}... 
    ,'MutationFcn',{@mutationuniform, 0.1},'CrossoverFraction',0.6,'PopulationSize',1000); 
% options = gaoptimset('UseParallel',true,'PlotFcns',{@gaplotbestf,@gaplotbestindiv},'CrossoverFraction',0.6);
[GAsol, fit] = ga(@GA_Sim_KW,nParams,[],[],[],[],[],[],[],[],options);
c = clock;
save(['Workspaces/GAsol_fit' num2str(fit) '_d' num2str(c(3)) '_h' num2str(c(4)) '_m' num2str(c(5)) '.mat'],'GAsol');


%% Simulate Results %%

KW = KneedWalker;
KW.to = [5 0 0]; % set the torso as a point mass
net = feedforwardnet(5);
net = configure(net,rand(4,10),rand(2,10));
net = setwb(net, GAsol.');
C = nnController(5);
C.net = net;
Floor = Terrain(0,0);
Sim = Simulation(KW, C, Floor);
Sim.IC = [0 0 17.63/18*pi 17/18*pi 17.63/18*pi 17/18*pi 0 0 0 0 0 0];

opt = odeset('reltol', 1e-8, 'abstol', 1e-9, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, Xe, Ie] = ode45(@Sim.Derivative, 0:1e-3:10, Sim.IC, opt);
Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:),Sim.Env); 
if Ie(end) >= 3 || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
    EndCond = 1;
end
while ~EndCond
    [tTime, tX, tTe, tXe,tIe] = ode45(@Sim.Derivative, Time(end):1e-3:10, Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
    Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:),Sim.Env); 
    if Ie(end) >= 3 || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
        EndCond = 1;
    end       
end
figure()
for ii = 1:length(Time)-1
    Sim.RenderSim(X(ii,:),-1,5);
    dt = Time(ii+1) - Time(ii);
    drawnow;
    pause(dt);
end
figure()
plot(Time, [X(:,3) - X(:,5), X(:,4) - X(:,6)]);
xlabel('Time [sec]'); ylabel('\Delta\theta [rad]');