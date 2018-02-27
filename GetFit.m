function [fitness] = GetFit(KW, Xe, Ie, X)
%GetFIt calculates the fitness of a single Kneed walker specimen
% Leg1x = KW.GetPos(X(end,:),'Lankle');
% Leg2x = KW.GetPos(X(end,:), 'Rankle');
% rearLeg = min([Leg1x(1), Leg2x(1)]);


ind = find(Ie == 1);
Xstep = Xe(ind,:); 
rearLeg = [];
for ii = 1:length(ind)
    Leg1x = KW.GetPos(Xstep(ii,:),'Sankle');
    rearLeg = [rearLeg, Leg1x(1)]; %#ok
end
delta = diff(rearLeg);
if ~isempty(delta)
    fitness = -delta*delta.'
else 
    fitness = 0;
end
end