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

KW = KneedWalker;
KW.to = [5 0 0]; % set the torso as a point mass
net = feedforwardnet(5);
net = configure(net,rand(4,10),rand(2,10));
net = setwb(net, rand(37,1));
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

for ii = 1:length(Time)-1
    Sim.RenderSim(X(ii,:),-1,5);
    dt = Time(ii+1) - Time(ii);
    drawnow;
    pause(dt);
end
E = []; Pos = [];
for ii = 1:length(Time)
    E(ii) = Sim.Mod.GetEnergy(X(ii,:));        %#ok
    Pos(ii,:) = KW.GetPos(X(ii,:), 'NSankle'); %#ok
end
figure()
plot(Time,E)
xlabel('Time [sec]'); ylabel('Energy [J]');
figure()
plot(Time, [X(:,3) - X(:,5), X(:,4) - X(:,6)]);
xlabel('Time [sec]'); ylabel('\Delta\theta [rad]');
