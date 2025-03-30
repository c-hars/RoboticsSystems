clear all
close all

% Define link lengths and position
l1 = 0;
l2 = 0.4;
l3 = 0.4;
l4 = 0.05;
l5 = 0.05;

% Define position and dimensions of chessboard
chessmidpoint = [0.4; 0; 0];
chessW = 0.3;
chessH = 0.02;

% Test positions for validating inverse kinematics
upperleft_0 = chessmidpoint + [(chessW*3/8); (chessW*3/8); 0.02]
lowerright_0 = chessmidpoint + [(-chessW*3/8); (-chessW*5/8); 0]

% Position of end effector in zero frame
pepos_0 = lowerright_0; 

q1 = -atan2(pepos_0(2),pepos_0(1))

R10 = R_z(q1);

p3pos_0 = pepos_0 + [0;0;(l5)];
p2pos_0 = p3pos_0 + [0;0;(l4)];

p2pos_1 = R10*p2pos_0 - [l1;0;0];

cos_q3 = (p2pos_1(1)^2 + p2pos_1(3)^2 - (l2^2 + l3^2)) / (2*l2*l3);
q3 = -acos(max(min(cos_q3, 1), -1))

q2 = (atan2(p2pos_1(3),p2pos_1(1)) - asin(l3*sin(pi-q3)/(sqrt(p2pos_1(1)^2 + p2pos_1(3)^2))))

q5 = -q1

q4 = -q2 - q3 - pi/2

% Transformation matrices of arm
T10 = transformation_matrix(0,      0,    0,        q1);
T21 = transformation_matrix(pi/2,  l1,          0,        q2);
T32 = transformation_matrix(0,     l2,          0,        q3);
T43 = transformation_matrix(0,     l3,          0, q4 + pi/2);
T54 = transformation_matrix(pi/2,   0,         l4,        q5);
T65 = transformation_matrix(0,      0,         l5,         0);

T20 = T21*T10;
T30 = T32*T20;
T40 = T43*T30;
T50 = T54*T40;
T60 = T65*T50;

points(:,2) = double(transpose(T10(1:3,1:3))*T10(1:3,4));
points(:,3) = double(transpose(T20(1:3,1:3))*T20(1:3,4));
points(:,4) = double(transpose(T30(1:3,1:3))*T30(1:3,4));
points(:,5) = double(transpose(T40(1:3,1:3))*T40(1:3,4));
points(:,6) = double(transpose(T50(1:3,1:3))*T50(1:3,4));
points(:,7) = double(transpose(T60(1:3,1:3))*T60(1:3,4));

disp(points)

figure
scatter3(points(1,:),points(2,:),points(3,:), 60, 'filled', 'k')
hold on

for i = 1:6
   plot3([points(1,i), points(1,i+1)], [points(2,i), points(2,i+1)], [points(3,i), points(3,i+1)], 'k-', 'LineWidth', 2);
end
plot_chessboard(chessmidpoint, chessW, chessW, chessH)
axis equal
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');

%% Functions

function tm = invert_mat(T)
    R = T(1:3,1:3);
    Rt = transpose(R);
    d = T(1:3,4);
    tm = [Rt, Rt*-d; 0 0 0 1];
end

function tm = transformation_matrix(alpha, a, d, theta)
    R1 = R_x(alpha);
    R2 = R_z(theta);
    tm1 = [R1, [a; 0; 0]; 0 0 0 1];
    tm2 = [R2, [0; 0; d]; 0 0 0 1];
    tm = tm2*tm1;
end

function R = R_z(theta)
    R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
end

function R = R_x(theta)
    R = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
end

