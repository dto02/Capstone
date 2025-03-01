
%% JACOBIAN STARTS HERE
% Bar lengths in cm*
l1 = 4.45/2; % Half of distance between panto motors
l2 = 14.3; % Motor to elbow linkage length
l5 = 14.3;
l3 = 19.3; % Elbow to tip linkage length
l4 = 19.3;
l6 = 17.7/2; % Half of distance between shoulder motors
l8 = 17.7/2; %16.1

%% ORIGINS AND K for 0 to 6 Coordinates
o3 = Tb_3(1:3,4);
o2 = Tb_2(1:3,4);
o1 = Tb_1(1:3,4);
o0 = Tb_0(1:3,4);

i6 = Tb_6(1:3,1);
k6 = Tb_6(1:3,3);

k5 = Tb_5(1:3,3);
k4 = Tb_4(1:3,3);
k3 = Tb_3(1:3,3);
k2 = Tb_2(1:3,3);
k1 = Tb_1(1:3,3);
k0 = Tb_0(1:3,3);

%% ORIGINS AND K FOR prime coordinates
o3p = Tb_3p(1:3,4);
o2p = Tb_2p(1:3,4);
o1p = Tb_1p(1:3,4);
o0p = Tb_0p(1:3,4);

k3p = Tb_3p(1:3,3);
k2p = Tb_2p(1:3,3);
k1p = Tb_1p(1:3,3);
k0p = Tb_0p(1:3,3);

%% Jacobian Parameters
T8 = inv([k3 k4 k5]);
T5 = [k0 k3 k3];
T3 = inv([cross(k0,o3-o0) cross(k3,o3-o0) cross(k3,o3-o2)]);
T3p = inv([cross(k0p,o3p-o0p) cross(k3p,o3p-o0p) cross(k3p,o3p-o2p)]);

%% T7 calculation
% Define the vectors as symbolic column matrices
o3 = o3 + [0; l6; 0];
i = [1;0;0]; j = [0;1;0]; k = [0;0;1]; ob = 0; q3 = pantoAngleR_A; q2 = pantoAngleL_A;
o1_prime = ob + [l1; 0; 0];
o2_prime = o1_prime + [l2*cos(q3); l2*sin(q3); 0];
o3_prime = [i.' * o3; norm((j*j.' + k*k.')*o3); 0];
o5_prime = ob - [l1; 0; 0];
o4_prime = o5_prime + [l5*cos(q2); l5*sin(q2); 0];

% Compute V(q2, q3)
numerator_V = k.' * cross((o4_prime - o3_prime), (o5_prime - o4_prime));
denominator_V = k.' * cross((o4_prime - o2_prime), (o4_prime - o3_prime));
V_q2_q3 = numerator_V / denominator_V;

% Compute V'(q2, q3)
numerator_Vp = 2*l1 * j.' * (o4_prime - o3_prime);
Vp_q2_q3 = numerator_Vp / denominator_V - V_q2_q3;

T7 = [1 0 0; 0 -Vp_q2_q3/V_q2_q3 1/V_q2_q3; 0 1 0];

%% T7p calculation
% Define the vectors as symbolic column matrices
i = [1;0;0]; j = [0;1;0]; k = [0;0;1]; ob = 0; q3p = pantoAngleR_Ap; q2p = pantoAngleL_Ap;
o3p = o3p + [0; -l6; 0];
o1p_prime = ob + [l1; 0; 0];
o2p_prime = o1p_prime + [l2*cos(q3p); l2*sin(q3p); 0];
o3p_prime = [i.' * o3p; norm((j*j.' + k*k.')*o3p); 0];
o5p_prime = ob - [l1; 0; 0];
o4p_prime = o5p_prime + [l5*cos(q2p); l5*sin(q2p); 0];

% Compute V(q2, q3)
numerator_pV = k.' * cross((o4p_prime - o3p_prime), (o5p_prime - o4p_prime));
denominator_pV = k.' * cross((o4p_prime - o2p_prime), (o4p_prime - o3p_prime));
pV_q2_q3 = numerator_pV / denominator_pV;

% Compute V'(q2, q3)
numerator_pVp = 2*l1 * j.' * (o4p_prime - o3p_prime);
pVp_q2_q3 = numerator_pVp / denominator_pV - V_q2_q3;

T7p = [1 0 0; 0 -pVp_q2_q3/pV_q2_q3 1/pV_q2_q3; 0 1 0];
%% Jacobian Matrix Computation
skewj6 = [   0    -j6(3)   j6(2);
           j6(3)    0    -j6(1);
          -j6(2)   j6(1)    0 ];

Jacobian = zeros(7,6);
Jacobian(1:3,1:3) = T7*T3;
Jacobian(4:6,1:3) = T7p*T3p;
Jacobian(7,1:3) = -1*i.'*T8*T5*T3;
Jacobian(1:3,4:6) = T7*T3*l8*skewj6;
Jacobian(4:6,4:6) = -T7p*T3p*l8*skewj6;
Jacobian(7,4:6) = i.'*T8*(eye(3) - T5*T3*l8*skewj6);

%% Joint Torque Testing
wrench = [0;0;10;0;0;0];
torque = Jacobian*wrench
