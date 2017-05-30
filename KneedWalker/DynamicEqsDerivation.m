% Torso parameters
syms mt lt It real
 
% Shin parameters
syms msh lsh Ish real
 
% Thigh parameters
syms mth lth Ith real
 
% Input moments
syms uShip uNShip uSknee uNSknee real
InMoments = [uShip, uNShip, uSknee, uNSknee].';

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
rs1 = [x + lth/2*sin(gs), y - lth/2*cos(gs)];    % support thigh
rns1 = [x - lth/2*sin(gns), y - lth/2*cos(gns)]; % non support thigh
rs2 = rs1 + [lth/2*sin(gs) + lsh/2*sin(bs),-lth/2*cos(gs) - lsh/2*cos(bs)];        % support shin
rns2 = rns1 + [-lth/2*sin(gns) - lsh/2*sin(bns),-lth/2*cos(gns) - lsh/2*cos(bns)]; % non support shin

% Center of masses velocities (using chain rule)
vt   = jacobian(rt,q)*dq;
vs1  = jacobian(rs1,q)*dq;
vns1 = jacobian(rns1,q)*dq;
vs2  = jacobian(rs2,q)*dq;
vns2 = jacobian(rns2,q)*dq;

%%% Kinetic energy %%%

% Torso
ket = 1/2*mt*(vt.'*vt) + 1/2*It*a^2;

% Support leg
ke_support_thigh = 1/2*mth*(vs1.'*vs1) + 1/2*Ith*gs^2;
ke_support_shin  = 1/2*msh*(vs2.'*vs2) + 1/2*Ish*bs^2;

% Non support leg
ke_nsupport_thigh = 1/2*mth*(vns1.'*vns1) + 1/2*Ith*gns^2;
ke_nsupport_shin  = 1/2*msh*(vns2.'*vs2) + 1/2*Ish*bns^2;

KE = simplify(ket + ke_support_thigh + ke_support_shin + ke_nsupport_thigh...
   + ke_nsupport_shin);

%%% Potential energy %%%

% Torso
pet = mt*g*rt(2);

% Support leg
pe_support_thigh = mth*g*rs1;
pe_support_shin = msh*g*rs2;

% Non support leg
pe_nsupport_thigh = mth*g*rns1;
pe_nsupport_shin = msh*g*rns2;

PE = simplify(pet + pe_support_thigh + pe_support_shin + pe_nsupport_thigh...
   + pe_nsupport_shin);

%%% Input forces power %%%

hipSAng   = da + dgs;
hipNSAng  = da - dgns;
kneeSAng  = dgs - dbs;
kneeNSAng = dgns - dbns;
relAngs = [hipSAng, hipNSAng, kneeSAang, kneeNSAng];
P = InMoments*relAngs;















