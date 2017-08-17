function [M, W] = EventMat(KW, X)

% Decides what are the right impact matrices given the robot's Phase and 
% event index. For further information about the derivation of the         
% matrices look at DynamicEqsDerivation.m script.

% note: if an impact occurres when one of the knees is locked, it is
% assumed that the knee continous to be locked through the impact.


mt = KW.to(1); lt = KW.to(2); It = KW.to(3);
msh = KW.sh(1); lsh = KW.sh(2); Ish = KW.sh(3);
mth = KW.th(1); lth = KW.th(2); Ith = KW.th(3);


x = X(1); %#ok
y = X(3); %#ok
a = X(5);
gs = X(7);
gns = X(9);
bs = X(11);
bns = X(13);



M = [            2*msh + mt + 2*mth,                             0,  (lt*mt*cos(a))/2,   (lth*cos(gs)*(3*msh + mth))/2,  -(lth*cos(gns)*(msh + mth))/2,          (3*lsh*msh*cos(bs))/4,          -(lsh*msh*cos(bns))/4
    0,            2*msh + mt + 2*mth, -(lt*mt*sin(a))/2,   (lth*sin(gs)*(3*msh + mth))/2,   (lth*sin(gns)*(msh + mth))/2,          (3*lsh*msh*sin(bs))/4,           (lsh*msh*sin(bns))/4
    (lt*mt*cos(a))/2,             -(lt*mt*sin(a))/2,  (mt*lt^2)/4 + It,                               0,                              0,                              0,                              0
    (lth*cos(gs)*(3*msh + mth))/2, (lth*sin(gs)*(3*msh + mth))/2,                 0, Ith + lth^2*msh + (lth^2*mth)/4,   -(lth^2*msh*cos(gns + gs))/2,   (lsh*lth*msh*cos(bs - gs))/2, -(lsh*lth*msh*cos(bns + gs))/4
    -(lth*cos(gns)*(msh + mth))/2,  (lth*sin(gns)*(msh + mth))/2,                 0,    -(lth^2*msh*cos(gns + gs))/2,            (mth*lth^2)/4 + Ith, -(lsh*lth*msh*cos(bs + gns))/4,                              0
    (3*lsh*msh*cos(bs))/4,         (3*lsh*msh*sin(bs))/4,                 0,    (lsh*lth*msh*cos(bs - gs))/2, -(lsh*lth*msh*cos(bs + gns))/4,            (msh*lsh^2)/4 + Ish,   -(lsh^2*msh*cos(bns + bs))/8
    -(lsh*msh*cos(bns))/4,          (lsh*msh*sin(bns))/4,                 0,  -(lsh*lth*msh*cos(bns + gs))/4,                              0,   -(lsh^2*msh*cos(bns + bs))/8,                            Ish];


%%% Impact desicion %%%
switch KW.iEvent
    case 1 % ground contact
        if strcmp(KW.Phase, 'KneesFree') % Grizzle model predicts this
            W = [ 1, 0, 0, 0, -lth*cos(gns), 0, -lsh*cos(bns)
                0, 1, 0, 0,  lth*sin(gns), 0,  lsh*sin(bns)];
            return;
        end
        if strcmp(KW.Phase, 'KneesLock') % Hsu's model predicts this      
            W = [ 1, 0, 0,  0, -lth*cos(gns), 0, -lsh*cos(bns)            
                0, 1, 0,  0,  lth*sin(gns), 0,  lsh*sin(bns)
                0, 0, 0, -1,             0, 1,             0                 
                0, 0, 0,  0,             1, 0,            -1];            
            return;       
        end
        if strcmp(KW.Phase, 'NSKneeFree')
            W = [ 1, 0, 0,  0, -lth*cos(gns), 0, -lsh*cos(bns)
                0, 1, 0,  0,  lth*sin(gns), 0,  lsh*sin(bns)
                0, 0, 0, -1,             0, 1,             0];
            warning('Non support knee was free before ground impact');
            return;
        end 
        if strcmp(KW.Phase, 'SKneeFree')
            W = [ 1, 0, 0, 0, -lth*cos(gns), 0, -lsh*cos(bns)
                0, 1, 0, 0,  lth*sin(gns), 0,  lsh*sin(bns)
                0, 0, 0, 0,             1, 0,            -1];
            warning('Support knee was free before ground impact');
            return;
        end
    case 2 % support knee locked
       if strcmp(KW.Phase, 'SKneeFree')
           W =  [ 1, 0, 0, lth*cos(gs), 0, lsh*cos(bs),  0
             0, 1, 0, lth*sin(gs), 0, lsh*sin(bs),  0
             0, 0, 0,          -1, 0,           1,  0 
             0, 0, 0,           0, 1,           0, -1];
         warning('Support knee got locked and non support knee is already locked');  
         return;
       elseif strcmp(KW.Phase, 'KneesFree')
           W =  [ 1, 0, 0, lth*cos(gs), 0, lsh*cos(bs),  0
               0, 1, 0, lth*sin(gs), 0, lsh*sin(bs),  0
               0, 0, 0,          -1, 0,           1,  0];
           warning('Support knee got locked and non support knee is not locked');  
       else error('Non support kneed got locked but it''s alredy locked');
       end
    case 3 % non support knee locked
        if strcmp(KW.Phase, 'NSKneeFree')
            W = [ 1, 0, 0, lth*cos(gs), 0, lsh*cos(bs),  0
              0, 1, 0, lth*sin(gs), 0, lsh*sin(bs),  0
              0, 0, 0,          -1, 0,           1,  0
              0, 0, 0,           0, 1,           0, -1];
            return;
        elseif strcmp(KW.Phase, 'KneesFree')
            W = [ 1, 0, 0, lth*cos(gs), 0, lsh*cos(bs),  0
                0, 1, 0, lth*sin(gs), 0, lsh*sin(bs),  0
                0, 0, 0,           0, 1,           0, -1];
            return;
        else error('Non support kneed got locked but it''s alredy locked');
        end

            


    
    
    
end


