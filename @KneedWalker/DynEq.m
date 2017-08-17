function [M, B, G, W, Wdot, Fq,C] = DynEq(KW, X)
% Decides what are the right dynamic equations and gives the corresponding 
% Matrices. For further information about the derivation of the matrices,
% look at DynamicEqsDerivation.m script


mt = KW.to(1); lt = KW.to(2); It = KW.to(3);
msh = KW.sh(1); lsh = KW.sh(2); Ish = KW.sh(3);
mth = KW.th(1); lth = KW.th(2); Ith = KW.th(3);
g = KW.g;
uShip = KW.Torques(1); uNShip = KW.Torques(2); uSknee = KW.Torques(3);...
        uNSknee = KW.Torques(4);


x = X(1); %#ok
dx = X(2);
y = X(3); %#ok
dy = X(4);
a = X(5);
da = X(6);
gs = X(7);
dgs = X(8);
gns = X(9);
dgns = X(10);
bs = X(11);
dbs = X(12);
bns = X(13);
dbns = X(14);


M = [            2*msh + mt + 2*mth,                             0,  (lt*mt*cos(a))/2,   (lth*cos(gs)*(3*msh + mth))/2,  -(lth*cos(gns)*(msh + mth))/2,          (3*lsh*msh*cos(bs))/4,          -(lsh*msh*cos(bns))/4
    0,            2*msh + mt + 2*mth, -(lt*mt*sin(a))/2,   (lth*sin(gs)*(3*msh + mth))/2,   (lth*sin(gns)*(msh + mth))/2,          (3*lsh*msh*sin(bs))/4,           (lsh*msh*sin(bns))/4
    (lt*mt*cos(a))/2,             -(lt*mt*sin(a))/2,  (mt*lt^2)/4 + It,                               0,                              0,                              0,                              0
    (lth*cos(gs)*(3*msh + mth))/2, (lth*sin(gs)*(3*msh + mth))/2,                 0, Ith + lth^2*msh + (lth^2*mth)/4,   -(lth^2*msh*cos(gns + gs))/2,   (lsh*lth*msh*cos(bs - gs))/2, -(lsh*lth*msh*cos(bns + gs))/4
    -(lth*cos(gns)*(msh + mth))/2,  (lth*sin(gns)*(msh + mth))/2,                 0,    -(lth^2*msh*cos(gns + gs))/2,            (mth*lth^2)/4 + Ith, -(lsh*lth*msh*cos(bs + gns))/4,                              0
    (3*lsh*msh*cos(bs))/4,         (3*lsh*msh*sin(bs))/4,                 0,    (lsh*lth*msh*cos(bs - gs))/2, -(lsh*lth*msh*cos(bs + gns))/4,            (msh*lsh^2)/4 + Ish,   -(lsh^2*msh*cos(bns + bs))/8
    -(lsh*msh*cos(bns))/4,          (lsh*msh*sin(bns))/4,                 0,  -(lsh*lth*msh*cos(bns + gs))/4,                              0,   -(lsh^2*msh*cos(bns + bs))/8,                            Ish];

B =    [(lsh*msh*sin(bns)*dbns^2)/4 + ((lth*msh*sin(gns))/2 + (lth*mth*sin(gns))/2)*dgns^2 + (- (3*lth*msh*sin(gs))/2 - (lth*mth*sin(gs))/2)*dgs^2 - (da^2*lt*mt*sin(a))/2 - (3*dbs^2*lsh*msh*sin(bs))/4
    (lsh*msh*cos(bns)*dbns^2)/4 + (3*lsh*msh*cos(bs)*dbs^2)/4 + ((lth*msh*cos(gns))/2 + (lth*mth*cos(gns))/2)*dgns^2 + ((3*lth*msh*cos(gs))/2 + (lth*mth*cos(gs))/2)*dgs^2 - (da^2*lt*mt*cos(a))/2
    0
    (lth*msh*(lsh*sin(bns + gs)*dbns^2 - 2*lsh*sin(bs - gs)*dbs^2 + 2*lth*sin(gns + gs)*dgns^2))/4
    (lsh*msh*sin(bs + gns)*dbs^2*lth)/4 + (msh*sin(gns + gs)*dgs^2*lth^2)/2
    (lsh*msh*(lsh*sin(bns + bs)*dbns^2 + 2*lth*sin(bs + gns)*dgns^2 + 4*lth*sin(bs - gs)*dgs^2))/8
    (msh*sin(bns + bs)*dbs^2*lsh^2)/8 + (lth*msh*sin(bns + gs)*dgs^2*lsh)/4];


% C =  [(lsh*msh*sin(bns)*dbns^2)/4 + dgns*((dgns*lth*msh*sin(gns))/2 + (dgns*lth*mth*sin(gns))/2) - dgs*((3*dgs*lth*msh*sin(gs))/2 + (dgs*lth*mth*sin(gs))/2) - (da^2*lt*mt*sin(a))/2 - (3*dbs^2*lsh*msh*sin(bs))/4
%  (lsh*msh*cos(bns)*dbns^2)/4 + (3*lsh*msh*cos(bs)*dbs^2)/4 + dgns*((dgns*lth*msh*cos(gns))/2 + (dgns*lth*mth*cos(gns))/2) + dgs*((3*dgs*lth*msh*cos(gs))/2 + (dgs*lth*mth*cos(gs))/2) - (da^2*lt*mt*cos(a))/2
%                                                                                                                                                                                                             0
%                                                                                                                (lth*msh*(lsh*sin(bns + gs)*dbns^2 - 2*lsh*sin(bs - gs)*dbs^2 + 2*lth*sin(gns + gs)*dgns^2))/4
%                                                                                                                                       (lsh*msh*sin(bs + gns)*dbs^2*lth)/4 + (msh*sin(gns + gs)*dgs^2*lth^2)/2
%                                                                                                                (lsh*msh*(lsh*sin(bns + bs)*dbns^2 + 2*lth*sin(bs + gns)*dgns^2 + 4*lth*sin(bs - gs)*dgs^2))/8
%                                                                                                                                       (msh*sin(bns + bs)*dbs^2*lsh^2)/8 + (lth*msh*sin(bns + gs)*dgs^2*lsh)/4];
% 
% 
G = [0
    g*(2*msh + mt + 2*mth)
    -(g*lt*mt*sin(a))/2
    (g*lth*sin(gs)*(2*msh + mth))/2
    (g*lth*sin(gns)*(2*msh + mth))/2
    (g*lsh*msh*sin(bs))/2
    (g*lsh*msh*sin(bns))/2];

%%% Dynamic eqs decision %%%

switch KW.Phase
    case 'KneesFree'
        W = [ 1, 0, 0, lth*cos(gs), 0, lsh*cos(bs), 0
            0, 1, 0, lth*sin(gs), 0, lsh*sin(bs), 0];
        
        Wdot = [ 0, 0, 0, -dgs*lth*sin(gs), 0, -dbs*lsh*sin(bs), 0
            0, 0, 0,  dgs*lth*cos(gs), 0,  dbs*lsh*cos(bs), 0];
        
        
    case 'SkneeFree'
        W = [ 1, 0, 0, lth*cos(gs), 0, lsh*cos(bs),  0
            0, 1, 0, lth*sin(gs), 0, lsh*sin(bs),  0
            0, 0, 0,           0, 1,           0, -1];
        Wdot = [ 0, 0, 0, -dgs*lth*sin(gs), 0, -dbs*lsh*sin(bs), 0
            0, 0, 0,  dgs*lth*cos(gs), 0,  dbs*lsh*cos(bs), 0
            0, 0, 0,                0, 0,                0, 0];
        
        
    case 'NSKneeFree'
        W = [ 1, 0, 0, lth*cos(gs), 0, lsh*cos(bs), 0
            0, 1, 0, lth*sin(gs), 0, lsh*sin(bs), 0
            0, 0, 0,          -1, 0,           1, 0];
        Wdot = [ 0, 0, 0, -dgs*lth*sin(gs), 0, -dbs*lsh*sin(bs), 0
            0, 0, 0,  dgs*lth*cos(gs), 0,  dbs*lsh*cos(bs), 0
            0, 0, 0,                0, 0,                0, 0];
        
        
    case 'KneesLock'
        W = [ 1, 0, 0, lth*cos(gs), 0, lsh*cos(bs),  0
            0, 1, 0, lth*sin(gs), 0, lsh*sin(bs),  0
            0, 0, 0,          -1, 0,           1,  0
            0, 0, 0,           0, 1,           0, -1];
        
        Wdot = [ 0, 0, 0, -dgs*lth*sin(gs), 0, -dbs*lsh*sin(bs), 0
               0, 0, 0,  dgs*lth*cos(gs), 0,  dbs*lsh*cos(bs), 0
            0, 0, 0,                0, 0,                0, 0
            0, 0, 0,                0, 0,                0, 0];
    otherwise
        error('No such phase');
end


Fq = [          0
    0
    uNShip + uShip
    uShip + uSknee
    uNSknee - uNShip
    -uSknee
    -uNSknee];

end

