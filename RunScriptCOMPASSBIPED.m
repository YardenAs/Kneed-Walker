KW = KneedWalker;
KW.to = [5 0 0]; % set the torso as a point mass
Control = Controller(0.5*ones(1,4),zeros(1,4),[0 pi/2 0 pi/2]);
Floor = Terrain(0,0);
Sim = Simulation(KW, Control, Floor);
Sim.IC = [0 0 20/18*pi 14/18*pi 20/18*pi 14/18*pi 0 0 0 0 0 0];

opt = odeset('reltol', 1e-8, 'abstol', 1e-9, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, Xe, Ie] = ode45(@Sim.Derivative, [0 inf], Sim.IC, opt);
% for ii = 1:length(Time)
%     F(ii,:) = KW.GetReactionForces(X(ii,:).');
% end

Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:),Sim.Env);
if Ie(end) >= 2 || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
    EndCond = 1;
end


while ~EndCond
    [tTime, tX, tTe, tXe,tIe] = ode45(@Sim.Derivative,[Time(end) inf], Xf, opt);
    Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
    X  = [X; tX]; Time = [Time; tTime]; %#ok
%     for ii = 1:length(tTime)
%         tF(ii,:) = KW.GetReactionForces(tX(ii,:).');
%     end
%     F = [F;tF];
%     Fn = F(:,1)*sin(15/180*pi) + F(:,2)*cos(15/180*pi);
%     tF = [];
    Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:),Sim.Env);
    if Ie(end) >= 2 || ~isempty(KW.BadImpulse) || ~isempty(KW.BadLiftoff)
        EndCond = 1;
    end
end

for ii = 1:length(Time)-1
    Sim.RenderSim(X(ii,:),-1,5);
    dt = Time(ii+1) - Time(ii);
    drawnow;
    pause(dt*10);
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
