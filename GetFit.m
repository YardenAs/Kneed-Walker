function [fitness] = GetFit(KW, X, Time, Ie)
%GetFIt calculates the fitness of a single Kneed walker specimen
Leg1x = KW.GetPos(X(end,:),'Sankle');
Leg2x = KW.GetPos(X(end,:), 'NSankle');
rearLeg = min([Leg1x(1), Leg2x(1)]);

nSteps = length(find(Ie == 1));

fitness = 1/2*(-nSteps.^2 - pi^2*rearLeg.^2);


end

