function [fitness] = GetFit(KW, Xe, Ie, X)
%GetFIt calculates the fitness of a single Kneed walker specimen
% Leg1x = KW.GetPos(X(end,:),'Lankle');
% Leg2x = KW.GetPos(X(end,:), 'Rankle');
% rearLeg = min([Leg1x(1), Leg2x(1)]);
punishment = 1;
if any(Ie == 5)
    punishment = 0.8;
end
ind = find(Ie == 1);
Xstep = Xe(ind,:); 
rearLeg = [];
for ii = 1:length(ind)
    Leg1x = KW.GetPos(Xstep(ii,:),'Sankle');
    Leg2x = KW.GetPos(Xstep(ii,:),'NSankle');
    rearLeg = [rearLeg, min([Leg1x(1) Leg2x(1)])]; %#ok
end
delta = diff(rearLeg);

if ~isempty(delta)
    fitness = -sum(delta);
    n_steps_thresh = 6;
    if length(ind) > n_steps_thresh
        steps_states = Xe(ind((n_steps_thresh-2):2:length(ind)),3:end);
        difference   = diff(steps_states);
        for ii = 1:size(difference,2)
            norm_vec(ii) = norm(difference(:,ii));
        end
        weights = [1 1 0 0 0 0 0.1 0.1 0 0 0.1].';
        fitness = fitness + norm_vec*weights;
    end
else
    fitness = 0;
end
if fitness > -1e-3
    fitness = 0;
end
fitness = punishment*fitness;
end