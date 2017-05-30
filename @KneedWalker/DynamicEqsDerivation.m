% Torso parameters
syms mt lt It real
 
% Shin parameters
syms msh lsh Ish real
 
% Thigh parameters
syms mth lth Ith real
 
% Gravity
syms g
% Ith  = 1/12*mth*lth^2; It = 1/12*mt*lt^2; Ish = 1/12*msh*lsh^2;
 
% Generalized coordinates
syms x dx ddx y dy ddy a da dda gs dgs ddgs gns dgns ddgns bs dbs ddbs...
     bns dbns ddbns g real
 
q   = [x y a gs gns bs bns].';
dq  = [dx dy da dgs dgns dbs dbns].';
ddq = [ddx ddy dda ddgs ddgns ddbs ddbns].';

% Ith  = 1/12*mth*lth^2; It = 1/12*mt*lt^2; Ish = 1/12*msh*lsh^2;

% Center of massses positions
rt = [x + lt/2*sin(a), y + lt/2*cos(a)];
rs1 = [x + lth/2*sin(gs), y - lth/2*cos(gs)];
rns1 = [x - lth/2*sin(gns), y - lth/2*cos(gns)];
rs2 = rs1 + [lth/2*sin(gs) + lsh/2*sin(bs),-lth/2*cos(gs) - lsh/2*cos(bs)];
rns2 = rns1 + [-lth/2*sin(gns) - lsh/2*sin(bns),-lth/2*cos(gns) - lsh/2*cos(bns)];

% Center of masses velocities (using chain rule)
vt   = jacobian(rt,q)*dq;
vs1  = jacobian(rs1,q)*dq;
vns1 = jacobian(rns1,q)*dq;
vs2  = jacobian(rs2,q)*dq;
vns2 = jacobian(rns2,q)*dq;

%%% Kinetic energy %%%










