clear all
close all

syms theta real
syms q1 q2 q3 q4 q5 real

l1 = 0.05;
l2 = 0.2;
l3 = 0.2;
l4 = 0.05;
l5 = 0.1;

chessboard = 0.3;
offset = 0.4;

T10 = transformation_matrix(0,     0, 0.2, q1);
T21 = transformation_matrix(pi/2,  l1,  0, q2);
T32 = transformation_matrix(0,    l2,  0, q3);
T43 = transformation_matrix(0,    l3,  0, q4 + pi/2);
T54 = transformation_matrix(pi/2, 0, l4, q5);
T65 = transformation_matrix(0,     0, l5,  0);

T20 = T21*T10;
T30 = T32*T20;
T40 = T43*T30;
T50 = T54*T40;
T60 = T65*T50;

points = zeros(3,6);
T1plot = subs(T10, [q1, q2, q3, q4, q5], [0,0,0,0,0]);
T2plot = subs(T20, [q1, q2, q3, q4, q5], [0,0,0,0,0]);
T3plot = subs(T30, [q1, q2, q3, q4, q5], [0,0,0,0,0]);
T4plot = subs(T40, [q1, q2, q3, q4, q5], [0,0,0,0,0]);
T5plot = subs(T50, [q1, q2, q3, q4, q5], [0,0,0,0,0]);
T6plot = subs(T60, [q1, q2, q3, q4, q5], [0,0,0,0,0]);

points(:,2) = double(transpose(T1plot(1:3,1:3))*T1plot(1:3,4));
points(:,3) = double(transpose(T2plot(1:3,1:3))*T2plot(1:3,4));
points(:,4) = double(transpose(T3plot(1:3,1:3))*T3plot(1:3,4));
points(:,5) = double(transpose(T4plot(1:3,1:3))*T4plot(1:3,4));
points(:,6) = double(transpose(T5plot(1:3,1:3))*T5plot(1:3,4));
points(:,7) = double(transpose(T6plot(1:3,1:3))*T6plot(1:3,4));



%% ZX plane
q_ranges = {[0, 2*pi], [-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2]};
n_divs = 10;

pts_list = zeros(3, 5^5);
i = 1;
for i2 = linspace(q_ranges{2}(1), q_ranges{2}(2), n_divs)
    for i3 = linspace(q_ranges{3}(1), q_ranges{3}(2), n_divs)
        for i4 = linspace(q_ranges{4}(1), q_ranges{4}(2), n_divs)
            for i5 = linspace(q_ranges{5}(1), q_ranges{5}(2), n_divs)
                T_current = subs(T60, [q1, q2, q3, q4, q5], [0, i2, i3, i4, i5]);
                T_current = double(T_current);
                %T_current = T_sub(0,i2,i3,i4,i5);
                xyz = transpose(T_current(1:3,1:3))*T_current(1:3,4);
                xyz = double(xyz);
                pts_list(:,i) = xyz;
                i = i+1;
            end
        end
    end
end

%% XY Plane



%% Plotting 

figure; clf;
view(3)
scatter3(pts_list(1,:), pts_list(2,:), pts_list(3,:), 36, 'k');
hold on
plot3([offset, chessboard+offset], [0,0], [0,0],'r-', 'LineWidth', 2);
axis equal
xlim([-0.1 1]);
zlim([-0.1 1]);
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Demonstration of link lengths reach over chessboard')

view(0, 0);



%% Functions

function T = T_fast(q1, q2, q3, q4, q5)
T = [sin(q1)*sin(q5) - cos(q5)*(cos(q4 + pi/2)*(sin(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064) - cos(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064)) + sin(q4 + pi/2)*(sin(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064) + cos(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064))) - (4967757600021511*sin(q5)*(cos(q4 + pi/2)*(sin(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064) + cos(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064)) - sin(q4 + pi/2)*(sin(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064) - cos(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064))))/81129638414606681695789005144064,   cos(q1)*sin(q5) - cos(q5)*(cos(q4 + pi/2)*(sin(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) + cos(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1))) - sin(q4 + pi/2)*(sin(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)) - cos(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)))) + (4967757600021511*sin(q5)*(cos(q4 + pi/2)*(sin(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)) - cos(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2))) + sin(q4 + pi/2)*(sin(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) + cos(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)))))/81129638414606681695789005144064, (4967757600021511*sin(q5))/81129638414606681695789005144064 + cos(q5)*(cos(q4 + pi/2)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4 + pi/2)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) + (4967757600021511*sin(q5)*(cos(q4 + pi/2)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4 + pi/2)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))))/81129638414606681695789005144064, cos(q5)*(cos(q4 + pi/2) + cos(q4 + pi/2)*(cos(q3) + cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4 + pi/2)*(sin(q3) + cos(q2)*sin(q3) + cos(q3)*sin(q2))) - (4967757600021511*sin(q5)*(sin(q4 + pi/2) + cos(q4 + pi/2)*(sin(q3) + cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4 + pi/2)*(cos(q3) + cos(q2)*cos(q3) - sin(q2)*sin(q3))))/81129638414606681695789005144064;
(4967757600021511*cos(q5)*(cos(q4 + pi/2)*(sin(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064) + cos(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064)) - sin(q4 + pi/2)*(sin(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064) - cos(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064))))/81129638414606681695789005144064 - cos(q5)*sin(q1) - sin(q5)*(cos(q4 + pi/2)*(sin(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064) - cos(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064)) + sin(q4 + pi/2)*(sin(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064) + cos(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064))), - cos(q1)*cos(q5) - (4967757600021511*cos(q5)*(cos(q4 + pi/2)*(sin(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)) - cos(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2))) + sin(q4 + pi/2)*(sin(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) + cos(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)))))/81129638414606681695789005144064 - sin(q5)*(cos(q4 + pi/2)*(sin(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) + cos(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1))) - sin(q4 + pi/2)*(sin(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)) - cos(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)))), sin(q5)*(cos(q4 + pi/2)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4 + pi/2)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))) - (4967757600021511*cos(q5)*(cos(q4 + pi/2)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4 + pi/2)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))))/81129638414606681695789005144064 - (4967757600021511*cos(q5))/81129638414606681695789005144064, (4967757600021511*cos(q5)*(sin(q4 + pi/2) + cos(q4 + pi/2)*(sin(q3) + cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4 + pi/2)*(cos(q3) + cos(q2)*cos(q3) - sin(q2)*sin(q3))))/81129638414606681695789005144064 + sin(q5)*(cos(q4 + pi/2) + cos(q4 + pi/2)*(cos(q3) + cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4 + pi/2)*(sin(q3) + cos(q2)*sin(q3) + cos(q3)*sin(q2)));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                      (4967757600021511*sin(q1))/81129638414606681695789005144064 + cos(q4 + pi/2)*(sin(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064) + cos(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064)) - sin(q4 + pi/2)*(sin(q3)*(cos(q1)*sin(q2) + (4967757600021511*cos(q2)*sin(q1))/81129638414606681695789005144064) - cos(q3)*(cos(q1)*cos(q2) - (4967757600021511*sin(q1)*sin(q2))/81129638414606681695789005144064)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                         (4967757600021511*cos(q1))/81129638414606681695789005144064 - cos(q4 + pi/2)*(sin(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1)) - cos(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2))) - sin(q4 + pi/2)*(sin(q3)*((4967757600021511*cos(q1)*cos(q2))/81129638414606681695789005144064 - sin(q1)*sin(q2)) + cos(q3)*((4967757600021511*cos(q1)*sin(q2))/81129638414606681695789005144064 + cos(q2)*sin(q1))),                                                                                                                                             sin(q4 + pi/2)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - cos(q4 + pi/2)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) + 24678615572571482867467662723121/6582018229284824168619876730229402019930943462534319453394436096,                                                                                                                                                                                                                    sin(q4 + pi/2) + cos(q4 + pi/2)*(sin(q3) + cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4 + pi/2)*(cos(q3) + cos(q2)*cos(q3) - sin(q2)*sin(q3)) + 2;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                                                                                                                                   1];
end


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


