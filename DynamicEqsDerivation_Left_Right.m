% Torso parameters
syms mt lt It real
 
% shank parameters
syms msh lsh Ish real
 
% Thigh parameters
syms mth lth Ith real
 
% Input moments
syms uLhip uRhip uLknee uRknee real
InMoments = [uLhip, uRhip, uLknee, uRknee].';

% Gravity

syms g
% Ith  = 1/12*mth*lth^2; It = 1/12*mt*lt^2; Ish = 1/12*msh*lsh^2;
 
% Generalized coordinates
syms x dx ddx y dy ddy a da dda g_l dg_l ddg_l g_r dg_r ddg_r b_l db_l ddb_l...
     b_r db_r ddb_r g real
 
q   = [x y a b_l b_r g_l g_r].';
dq  = [dx dy da db_l db_r dg_l dg_r].';
ddq = [ddx ddy dda ddb_l ddb_r ddg_l ddg_r].';

% Ith  = 1/12*mth*lth^2; It = 1/12*mt*lt^2; Ish = 1/12*msh*lsh^2;

% Center of massses positions
r_L_Ankle = [x, y];                                         % left ankle
r_L_Shank = [x + lsh/2*sin(g_l), y - lsh/2*cos(g_l)];       % left shank
r_L_Knee  = [x + lsh*sin(g_l), y - lsh*cos(g_l)];           % left knee
r_L_Thigh = r_L_Knee + [lth/2*sin(b_l), -lth/2*sin(b_l)];   % left thigh
r_Hip     = r_L_Knee + [lth*sin(b_l), -lth*cos(b_l)];       % Hip
r_Torso   = r_Hip + [-lt/2*sin(a), lt/2*cos(a)];            % Torso
r_R_Thigh = r_Hip + [-lth/2*sin(b_r), lth/2*cos(b_r)];      % right thigh
r_R_Knee  = r_Hip + [-lth*sin(b_r), lth*cos(b_r)];          % right knee
r_R_Shank = r_R_Knee + [-lsh/2*sin(g_r), lsh/2*cos(g_r)];   % right shank
r_R_Ankle = r_R_Knee + [-lsh*sin(g_r), lsh*cos(g_r)];       % right ankle

% Center of masses velocities (using chain rule)
v_Torso   = jacobian(r_Torso,q)*dq;
v_L_Thigh = jacobian(r_L_Thigh,q)*dq;
v_R_Thigh = jacobian(r_R_Thigh,q)*dq;
v_L_Shank = jacobian(r_L_Shank,q)*dq;
v_R_Shank = jacobian(r_R_Shank,q)*dq;
v_L_Ankle = jacobian(r_L_Shank,q)*dq;
v_R_Ankle = jacobian(r_R_Ankle,q)*dq;


%%% Kinetic energy %%%

% Torso
ke_Torso = 1/2*mt*(v_Torso.'*v_Torso) + 1/2*It*da^2;

% left leg
ke_L_Thigh = 1/2*mth*(v_L_Thigh.'*v_L_Thigh) + 1/2*Ith*db_l^2;
ke_L_Shank  = 1/2*msh*(v_L_Shank.'*v_L_Shank) + 1/2*Ish*dg_l^2;

% right leg
ke_R_Thigh = 1/2*mth*(v_R_Thigh.'*v_R_Thigh) + 1/2*Ith*db_r^2;
ke_R_Shank  = 1/2*msh*(v_R_Shank.'*v_R_Shank) + 1/2*Ish*dg_r^2;

KE = simplify(ke_Torso + ke_L_Thigh + ke_L_Shank + ke_R_Thigh...
   + ke_R_Shank);

%%% Potential energy %%%

% Torso
pe_Torso = mt*g*r_Torso(2);

% left leg
pe_L_Thigh = mth*g*r_L_Thigh(2);
pe_L_Shank = msh*g*r_L_Shank(2);

% right leg
pe_R_Thigh = mth*g*r_R_Thigh(2);
pe_R_Shank = msh*g*r_R_Shank(2);

PE = simplify(pe_Torso + pe_L_Thigh + pe_L_Shank + pe_R_Thigh...
   + pe_R_Shank);

%%% Input forces power %%%

hipLAng   = db_l - da;
hipRAng  = db_r - da;
kneeLAng  = dg_l - db_l;
kneeRAng = dg_r - db_r;
relAng_l = [hipLAng, hipRAng, kneeLAng, kneeRAng];
P = relAng_l*InMoments;
Fq = jacobian(P,dq).';

%%% Ground Constraints %%%

W_L = jacobian(r_L_Ankle,q);
W_R = jacobian(r_R_Ankle,q);

%%% System's matrices %%%

G = jacobian(PE,q).'; % Potential energy vector
M = jacobian(KE,dq);
M = jacobian(M,dq); % Inertia matrix
W_L = simplify(W_L);
W_R = simplify(W_R);
GAMMA = 0;
for i = 1:7
    for j = 1:7
        for k = 1:7
            GAMMA = GAMMA + 1/2*(diff(M(i,j),q(k)) + diff(M(i,k),q(j))...
                    - diff(M(k,j),q(i)))*dq(j)*dq(k);
        end
    end
    B(i) = simplify(GAMMA); %#ok
GAMMA = 0;
end

W_L_dot = W_L;
W_R_dot = W_R;
n = size(W_L);
for i = 1:n(1)
    W_L_dot(i,:) = (jacobian(W_L(i,:),q)*dq).';
    W_R_dot(i,:) = (jacobian(W_R(i,:),q)*dq).';
end

W_L_dot = simplify(W_L_dot);
W_R_dot = simplify(W_R_dot);
B = simplify(B.');
M = simplify(M);
G = simplify(G);
