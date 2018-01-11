function [fitness] = GetFit(KW, Xe, Ie, X)
%GetFIt calculates the fitness of a single Kneed walker specimen
% Leg1x = KW.GetPos(X(end,:),'Lankle');
% Leg2x = KW.GetPos(X(end,:), 'Rankle');
% rearLeg = min([Leg1x(1), Leg2x(1)]);


ind = find(Ie == 1);
Xstep = Xe(ind,:); 
rearLeg = [];
for ii = 1:length(ind)
    Leg1x = KW.GetPos(Xstep(ii,:),'Lankle');
    Leg2x = KW.GetPos(Xstep(ii,:), 'Rankle');
    rearLeg = [rearLeg, min([Leg1x(1), Leg2x(1)])]; %#ok
end
delta = diff(rearLeg);
if ~isempty(delta)
    fitness = -delta*delta.';
else 
    fitness = 0;
end
meanA = mean(X(:,3));
fitness = fitness*(1-abs(meanA/(pi/2))/10); % up to 10% penalty based on torso angle
end