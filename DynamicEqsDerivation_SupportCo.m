% This script developes the dynamics equations of a kneed bipedal
% robot. The generalized coordinates are q = [x y a bs bns gs gns].' where
% x and y define the position of the stance leg, a bs bns gs gns are the
% angles of the support and non-support links respectively.

% Torso parameters
syms mt lt It real
 
% shank parameters
syms msh lsh Ish real
 
% Thigh parameters
syms mth lth Ith real
 
% Input moments
syms uSankle  uHip real
InMoments = [uHip, uSankle].';

% Gravity

syms g
% Ith  = 1/12*mth*lth^2; It = 1/12*mt*lt^2; Ish = 1/12*msh*lsh^2;
 
% Generalized coordinates
syms x dx ddx y dy ddy gs dgs ddgs gns dgns ddgns bs dbs ddbs...
     bns dbns ddbns g real
 
q   = [x y bs bns gs gns].';
dq  = [dx dy dbs dbns dgs dgns].';
ddq = [ddx ddy ddbs ddbns ddgs ddgns].';

% Ith  = 1/12*mth*lth^2; It = 1/12*mt*lt^2; Ish = 1/12*msh*lsh^2;

% Center of massses positions
rs2 = [x + lsh/2*sin(gs), y - lsh/2*cos(gs)];      % support shank
rsk = [x + lsh*sin(gs), y - lsh*cos(gs)];          % support knee
rs1 = rsk + [lth/2*sin(bs), -lth/2*sin(bs)];       % support thigh
rh  = rsk + [lth*sin(bs), -lth*cos(bs)];           % Hip
rt  = rh;                                          % Torso
rns1 = rh + [-lth/2*sin(bns), lth/2*cos(bns)];     % non-support thigh
rnsk = rh + [-lth*sin(bns), lth*cos(bns)];         % non-support knee
rns2 = rnsk + [-lsh/2*sin(gns), lsh/2*cos(gns)];   % non-support shank
rnsa = rnsk + [-lsh*sin(gns), lsh*cos(gns)];       % non-support shank

% Center of masses velocities (using chain rule)
vt   = jacobian(rt,q)*dq;
vs1  = jacobian(rs1,q)*dq;
vns1 = jacobian(rns1,q)*dq;
vs2  = jacobian(rs2,q)*dq;
vns2 = jacobian(rns2,q)*dq;
vnsa = jacobian(rnsa,q)*dq;


%%% Kinetic energy %%%

% Torso
ket = 1/2*mt*(vt.'*vt);

% Support leg
ke_support_thigh = 1/2*mth*(vs1.'*vs1) + 1/2*Ith*dbs^2;
ke_support_shank  = 1/2*msh*(vs2.'*vs2) + 1/2*Ish*dgs^2;

% Non support leg
ke_nsupport_thigh = 1/2*mth*(vns1.'*vns1) + 1/2*Ith*dbns^2;
ke_nsupport_shank  = 1/2*msh*(vns2.'*vns2) + 1/2*Ish*dgns^2;

KE = simplify(ket + ke_support_thigh + ke_support_shank + ke_nsupport_thigh...
   + ke_nsupport_shank);

%%% Potential energy %%%

% Torso
pet = mt*g*rt(2);

% Support leg
pe_support_thigh = mth*g*rs1(2);
pe_support_shank = msh*g*rs2(2);

% Non support leg
pe_nsupport_thigh = mth*g*rns1(2);
pe_nsupport_shank = msh*g*rns2(2);

PE = simplify(pet + pe_support_thigh + pe_support_shank + pe_nsupport_thigh...
   + pe_nsupport_shank);

%%% Input forces power %%%

hipSAng   = dbs;
hipNSAng  = dbns;
hip       = dbns - dbs;
kneeSAng  = dgs - dbs;
kneeNSAng = dgns - dbns;
relAngs = [hip, -dgs];
P = relAngs*InMoments;
Fq = jacobian(P,dq).';

%%% Ground Constraints %%%

W = jacobian([x,y],q);

%%% Knees Constraints %%%

SkneeLock  = bs - gs;
NSkneeLock = gns - bns;
W_k = jacobian([SkneeLock ,NSkneeLock],q);

%%% System's matrices %%%

G = jacobian(PE,q).'; % Potential energy vector
M = jacobian(KE,dq);
M = jacobian(M,dq); % Inertia matrix
W = simplify([W;W_k]);
GAMMA = 0;
for i = 1:6
    for j = 1:6
        for k = 1:6
            GAMMA = GAMMA + 1/2*(diff(M(i,j),q(k)) + diff(M(i,k),q(j))...
                    - diff(M(k,j),q(i)))*dq(j)*dq(k);
        end
    end
    B(i) = simplify(GAMMA); %#ok
GAMMA = 0;
end


Wdot = W;
n = size(W);
for i = 1:n(1)
Wdot(i,:) = (jacobian(W(i,:),q)*dq).';
end

Wdot = simplify(Wdot);
B = simplify(B.');
M = simplify(M);
G = simplify(G);



%%% Impact laws %%%%% 

%% ground contact

rnsa = rns1 + [-lth/2*sin(gns) - lsh*sin(bns),-lth/2*cos(gns) - lsh*cos(bns)];
W = jacobian(rnsa,q);
W = [W;W_k]; % depends on the robot's phase

%% knee lock
syms C
C = sym(zeros(7,1));
for i = 1:7
    sub(i) = diff(KE,dq(i)); %#ok
    for j = 1:7
        subb = diff(sub(i),q(j));
        C(i) = C(i) + subb*dq(j); 
    end
    C(i) = C(i) - diff(KE,q(i));
end
C = simplify(C);