% nParams must be equal to the number of weights - therefore check that the
% number of weights in GA_Sim_KW matches this size. 
nParams = 37;

% LB = [0, -Amp, zeros(1,3),zeros(1,3)];
% UB = [omega, Amp, Phase, Period];

options = gaoptimset('UseParallel',true,'PlotFcns',{@gaplotbestf,@gaplotbestindiv},'CrossoverFraction',0.6);
[GAsol, fit] = ga(@GA_Sim_KW,nParams,[],[],[],[],[],[],[],[],options);
c = clock;
save(['Workspaces/GAsol_fit' num2str(fit) '_d' num2str(c(3)) '_h' num2str(c(4)) '_m' num2str(c(5)) '.mat'],'GAsol');