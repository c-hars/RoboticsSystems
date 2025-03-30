clear




%% our robot forward kinematics

syms q_1 q_2 q_3 q_4 q_5 real
% syms l_1 l_2 l_3 l_4 l_5 real
l_1 = 1;
l_2 = 1;
l_3 = 1;
l_4 = 1;
l_5 = 1;

DH_table = [0 0 0 q_1; ...
            l_1 pi/2 0 q_2; ...
            l_2 0 0 q_3; ...
            l_3 0 0 q_4+pi/2; ...
            0 pi/2 l_4 q_5];

[T_05, T_cell] = compute_forward_kinematics(DH_table);
T_01 = T_cell{1};
T_12 = T_cell{2};
T_23 = T_cell{3};
T_34 = T_cell{4};
T_45 = T_cell{5};

E_5 = [0;0;l_5;1];
E_0 = simplify(T_05 * E_5) % end effector position in frame 0 coordinates

% expr = T_01*T_12*T_23*T_34*[0;-l_4;0;1];
% expr = subs(expr, q_1, 0);
% simplify(expr)
% return

%% draw_robot()

% q_coords = [0 0 0 0 0];
q_coords = [0 deg2rad(5) -deg2rad(5) 0 0];
q_coords(4) = -pi/2-q_coords(2) - q_coords(3);
% q_coords = [0 1 -1 1 -1]*pi/6;

j2_1 = [l_1; 0; 0; 1];
j3_2 = [l_2; 0; 0; 1];
j4_3 = [l_3; 0; 0; 1];
j5_4 = [0; -l_4; 0; 1];
jE_5 = [0; 0; l_5; 1];

j2_0 = T_01*j2_1;
j2_0 = subs(j2_0, [q_1 q_2 q_3 q_4 q_5], q_coords);

j3_0 = T_01*T_12*j3_2;
j3_0 = subs(j3_0, [q_1 q_2 q_3 q_4 q_5], q_coords);

j4_0 = T_01*T_12*T_23*j4_3;
j4_0 = subs(j4_0, [q_1 q_2 q_3 q_4 q_5], q_coords);

j5_0 = T_01*T_12*T_23*T_34*j5_4;
j5_0 = subs(j5_0, [q_1 q_2 q_3 q_4 q_5], q_coords);

jE_0 = T_01*T_12*T_23*T_34*T_45*jE_5;
jE_0 = subs(jE_0, [q_1 q_2 q_3 q_4 q_5], q_coords);


figure(1); clf

subplot(1,3,1); hold on

plot_chessboard([2.5 0 0], 1, 1, 0.01)
line([0, j2_0(1)], [0, j2_0(2)], [0, j2_0(3)], 'LineWidth', 5)
line([j2_0(1), j3_0(1)], [j2_0(2), j3_0(2)], [j2_0(3), j3_0(3)], 'LineWidth', 5)
line([j3_0(1), j4_0(1)], [j3_0(2), j4_0(2)], [j3_0(3), j4_0(3)], 'LineWidth', 5)
line([j4_0(1), j5_0(1)], [j4_0(2), j5_0(2)], [j4_0(3), j5_0(3)], 'LineWidth', 5)
line([j5_0(1), jE_0(1)], [j5_0(2), jE_0(2)], [j5_0(3), jE_0(3)], 'LineWidth', 5)

scatter3(0,0,0,1000,'k.')
xlabel('x'); ylabel('y'); zlabel('z')
grid on
view(45,50)
title("Oriented View")
axis square
% zlim([-0.1 0.1])
ylim([-2.5 2.5])
xlim([0 5])


subplot(1,3,2); hold on

plot_chessboard([2.5 0 0], 1, 1, 0.01)
line([0, j2_0(1)], [0, j2_0(2)], [0, j2_0(3)], 'LineWidth', 5)
line([j2_0(1), j3_0(1)], [j2_0(2), j3_0(2)], [j2_0(3), j3_0(3)], 'LineWidth', 5)
line([j3_0(1), j4_0(1)], [j3_0(2), j4_0(2)], [j3_0(3), j4_0(3)], 'LineWidth', 5)
line([j4_0(1), j5_0(1)], [j4_0(2), j5_0(2)], [j4_0(3), j5_0(3)], 'LineWidth', 5)
line([j5_0(1), jE_0(1)], [j5_0(2), jE_0(2)], [j5_0(3), jE_0(3)], 'LineWidth', 5)

scatter3(0,0,0,1000,'k.')
xlabel('x'); ylabel('y'); zlabel('z')
grid on
view(0,0)
title("Side View")
axis square
% zlim([-0.1 0.1])
ylim([-2.5 2.5])
xlim([0 5])

subplot(1,3,3); hold on

plot_chessboard([2.5 0 0], 1, 1, 0.01)
line([0, j2_0(1)], [0, j2_0(2)], [0, j2_0(3)], 'LineWidth', 5)
line([j2_0(1), j3_0(1)], [j2_0(2), j3_0(2)], [j2_0(3), j3_0(3)], 'LineWidth', 5)
line([j3_0(1), j4_0(1)], [j3_0(2), j4_0(2)], [j3_0(3), j4_0(3)], 'LineWidth', 5)
line([j4_0(1), j5_0(1)], [j4_0(2), j5_0(2)], [j4_0(3), j5_0(3)], 'LineWidth', 5)
line([j5_0(1), jE_0(1)], [j5_0(2), jE_0(2)], [j5_0(3), jE_0(3)], 'LineWidth', 5)

scatter3(0,0,0,1000,'k.')
xlabel('x'); ylabel('y'); zlabel('z')
grid on
view(0,90)
title("Bird's Eye View")
axis square
% zlim([-0.1 0.1])
ylim([-2.5 2.5])
xlim([0 5])


