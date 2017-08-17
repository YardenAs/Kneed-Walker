KW = KneedWalker;
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