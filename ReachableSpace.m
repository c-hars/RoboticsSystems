clear




%% 

q1_min = -deg2rad(60);
q1_max = deg2rad(60);

q2_min = deg2rad(5);
q2_max = deg2rad(90);

q3_min = deg2rad(-170);
q3_max = deg2rad(-10);

q4_min = -pi/2;
q4_max = 0;

q5_min = -pi/2;
q5_max = pi/2;

n_divs = 16;

pos_array = zeros(n_divs^5,3);
i = 1;
for q1=linspace(q1_min,q1_max,n_divs)
    for q2=linspace(q2_min,q2_max,n_divs)
        for q3=linspace(q3_min,q3_max,n_divs)
            for q4=linspace(q4_min,q4_max,n_divs)
                for q5=linspace(q5_min,q5_max,n_divs)
                    q4_wristdown = (-pi/2 - q2 - q3);
                    pos = compute_end_effector_pos_fast([q1 q2 q3 q4_wristdown q5]);
                    % pos = compute_end_effector_pos_fast([q1 q2 q3 q4 q5]);
                    pos_array(i,:) = pos';
                    i = i + 1;
                end
            end
        end
    end
end

%%

% figure(1); clf; hold on
% plot_chessboard([0.4 0 -0.2], 0.4, 0.4, 0.05)
% mask = pos_array(:,3) >= -0.2 & pos_array(:,3) <= 0.0;
% pos_array = pos_array(mask,:);
% scatter3(pos_array(:,1), pos_array(:,2), pos_array(:,3), 20, 'k.', 'MarkerEdgeAlpha', 0.5)
% xlabel('x'); ylabel('y'); zlabel('z')
% grid on
% view(55,20)
% title("Oriented View")
% axis equal

% return

%%

figure(1); clf; hold on

subplot(1,3,1); hold on
mask = abs(pos_array(:,3) - -0.075) <= 0.075;
pos_array12 = pos_array(mask,:);
mask = abs(pos_array12(:,1) - 0.4) <= 0.3;
pos_array12 = pos_array12(mask,:);
mask = abs(pos_array12(:,2) - 0.0) <= 0.3;
pos_array12 = pos_array12(mask,:);
plot_chessboard([0.4 0 -0.2], 0.4, 0.4, 0.05)
scatter3(pos_array12(:,1), pos_array12(:,2), pos_array12(:,3), 36, 'k.', 'MarkerEdgeAlpha', 0.5)
xlabel('x'); ylabel('y'); zlabel('z')
grid on
view(90,0)
title("Front View")
axis equal
zlim([-0.2 0])

subplot(1,3,2); hold on
plot_chessboard([0.4 0 -0.2], 0.4, 0.4, 0.05)
scatter3(pos_array12(:,1), pos_array12(:,2), pos_array12(:,3), 36, 'k.', 'MarkerEdgeAlpha', 0.5)
xlabel('x'); ylabel('y'); zlabel('z')
grid on
grid on
view(0,0)
title("Side View")
axis equal
zlim([-0.2 0])

subplot(1,3,3); hold on
plot_chessboard([0.4 0 -0.2], 0.4, 0.4, 0.05)
scatter3(pos_array12(:,1), pos_array12(:,2), pos_array12(:,3), 36, 'k.', 'MarkerEdgeAlpha', 0.5)
xlabel('x'); ylabel('y'); zlabel('z')
grid on
view(0,90)
title("Bird's Eye View")
axis equal

%%

figure(2); clf; hold on

subplot(1,2,1); hold on
mask = abs(pos_array(:,2)-0) < 0.05;
pos_array2 = pos_array(mask,:);
plot_chessboard([0.4 0 -0.2], 0.4, 0.4, 0.05)
scatter3(pos_array2(:,1), pos_array2(:,2), pos_array2(:,3), 'k.', 'MarkerEdgeAlpha', 0.9)
xlabel('x'); ylabel('y'); zlabel('z')
grid on
view(0,0)
title("Side View")
axis equal
% zlim([-0.1 0.1])
% ylim([-2.5 2.5])
% xlim([0 5])

subplot(1,2,2); hold on
mask = abs(pos_array(:,3) - (-0.2+0.05)) < 0.05/2;
pos_array3 = pos_array(mask,:);
plot_chessboard([0.4 0 -0.2], 0.4, 0.4, 0.05)
scatter3(pos_array3(:,1), pos_array3(:,2), pos_array3(:,3), 'k.', 'MarkerEdgeAlpha', 0.9)
xlabel('x'); ylabel('y'); zlabel('z')
grid on
view(0,90)
title("Bird's Eye View")
axis equal
% zlim([-0.1 0.1])
% ylim([-2.5 2.5])
% xlim([0 5])


%%

function pos = compute_end_effector_pos_fast(q)

q_1 = q(1);
q_2 = q(2);
q_3 = q(3);
q_4 = q(4);
q_5 = q(5);

l_1 = 0;
l_2 = 0.4;
% l_3 = 0.2;
% l_4 = 0.15;
% l_5 = 0.05;
l_3 = 0.4;
l_4 = 0.05;
l_5 = 0.05;

% our robot forward kinematics
x = l_5*(cos(q_1 + q_2 + q_3 + q_4)/2 + cos(q_2 - q_1 + q_3 + q_4)/2) + cos(q_1)*(l_1 + l_3*cos(q_2 + q_3) + l_2*cos(q_2) + l_4*cos(q_2 + q_3 + q_4));
y = sin(q_1)*(l_1 + l_3*cos(q_2 + q_3) + l_2*cos(q_2) + l_4*cos(q_2 + q_3 + q_4)) + l_5*(sin(q_1 + q_2 + q_3 + q_4)/2 - sin(q_2 - q_1 + q_3 + q_4)/2);
z = l_3*sin(q_2 + q_3) + l_2*sin(q_2) + l_4*sin(q_2 + q_3 + q_4) + l_5*sin(q_2 + q_3 + q_4);
pos = [x;y;z];

end



