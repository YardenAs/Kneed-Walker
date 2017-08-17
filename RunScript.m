KW = KneedWalker; 
Control = Controller(0.5*ones(1,4),ones(1,4),[0 pi/2 0 pi/2]);
Sim = Simulation(KW, Control, Terrain);
Sim.IC = zeros(1,14);
opt = odeset('reltol', 1e-9, 'abstol', 1e-9, 'Events', @Sim.Events);
EndCond = 0;
[Time, X, Te, Xe, Ie] = ode45(@Sim.Derivative, [0 inf], Sim.IC, opt);
Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:));
if Ie(end) >= 2
    EndCond = 1;
end

while ~EndCond
    [tTime, tX, tTe, tXe,tIe] = ode45(@Sim.Derivative,[Time(end) inf], Xf, opt);
        Ie = [Ie; tIe]; Te = [Te; tTe]; %#ok
        X  = [X; tX]; Time = [Time; tTime]; %#ok
        Xf = Sim.Mod.HandleEvent(Ie(end), X(end,:));
        if Ie(end) >= 2
            EndCond = 1;
        end
end

for ii = 1:length(Time)-1
    KW.Render(Time(ii),X(ii,:),1);
    dt = Time(ii+1) - Time(ii);
%     addpoints(path, COM(ii,1), COM(ii,2));
%     drawnow;
    pause(dt*11);
end

