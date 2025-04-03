syms q_1 q_2 q_3 q_4 q_5 real
syms q_1_dot q_2_dot q_3_dot q_4_dot q_5_dot real
syms l_1 l_2 l_3 l_4 l_5 real

DH_table = [0 0 0 q_1; ...
            l_1 pi/2 0 q_2; ...
            l_2 0 0 q_3; ...
            l_3 0 0 q_4+pi/2; ...
            0 pi/2 l_4 q_5];

[~, T_cell] = compute_forward_kinematics(DH_table);
T_01 = T_cell{1};
T_12 = T_cell{2};
T_23 = T_cell{3};
T_34 = T_cell{4};
T_45 = T_cell{5};

T_02 = T_01 * T_12;
T_03 = T_02 * T_23;
T_04 = T_03 * T_34;
T_05 = T_04 * T_45;

p5E = T_05(1:3,1:3) * [0;0;l_5]; % position vector from q5 joint origin to the end effector, expressed in frame 0 coordinates
p4E = T_04(1:3,1:3) * [0;-l_4;0] + p5E(1:3);
p3E = T_03(1:3,1:3) * [l_3;0;0]  + p4E(1:3);
p2E = T_02(1:3,1:3) * [l_2;0;0]  + p3E(1:3);
p1E = T_01(1:3,1:3) * [l_1;0;0]  + p2E(1:3);
p0E =                 [0;0;0]    + p1E(1:3);

z1_1 = [0;0;1;1]; % z axis of frame 1 in frame 1 coordinates
z2_2 = [0;0;1;1];
z3_3 = [0;0;1;1];
z4_4 = [0;0;1;1];
z5_5 = [0;0;1;1];

z1_0 = T_01(1:3,1:3) * z1_1(1:3); % z axis of frame 1 in frame 0 coordinates
z2_0 = T_02(1:3,1:3) * z2_2(1:3);
z3_0 = T_03(1:3,1:3) * z3_3(1:3);
z4_0 = T_04(1:3,1:3) * z4_4(1:3);
z5_0 = T_05(1:3,1:3) * z5_5(1:3);

term1 = cross(z1_0(1:3), p1E(1:3));
term2 = cross(z2_0(1:3), p2E(1:3));
term3 = cross(z3_0(1:3), p3E(1:3));
term4 = cross(z4_0(1:3), p4E(1:3));
term5 = cross(z5_0(1:3), p5E(1:3));

J1 = [term1; z1_0(1:3)];
J2 = [term2; z2_0(1:3)];
J3 = [term3; z3_0(1:3)];
J4 = [term4; z4_0(1:3)];
J5 = [term5; z5_0(1:3)];

jacobiann = [J1 J2 J3 J4 J5];
jacobiann = simplify(jacobiann)


% subs(jacobiann * [1;0;0;0;0], [q_1 q_2 q_3 q_4 q_5], [0 0 0 0 0])



% jacobian(p0E,[q_1 q_2 q_3 q_4 q_5])


