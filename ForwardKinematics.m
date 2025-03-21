syms theta real



l1 = 1;
l2 = 0.2;
l3 = 0.2;
l4 = 0.2;
l5 = 0.1;
syms q1 q2 q3 q4 q5 real



T1 = transformation_matrix(0,     0, l1, q1);
T2 = transformation_matrix(pi/2,  0,  0, q2 + pi/2);
T3 = transformation_matrix(0,    l2,  0, q3);
T4 = transformation_matrix(0,    l3,  0, q4 - pi/2);
T5 = transformation_matrix(-pi/2, 0, l4, q5);
T6 = transformation_matrix(0,     0, l5,  0);

T = T6*T5*T4*T3*T2*T1;

% T = invert_mat(T);


T_current = subs(T, [q1, q2, q3, q4, q5], [0,0,0,0,0]);
T_current = double(T_current)

% return

q_ranges = {[0, 2*pi], [-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2], [-pi/2, pi/2]};
n_divs = 10;

figure(1); clf; hold on
xlabel('x'); ylabel('y'); zlabel('z')
pts_list = zeros(3, 5^5);
i = 1;
for i1 = linspace(q_ranges{1}(1), q_ranges{1}(2), n_divs)
    for i2 = linspace(q_ranges{2}(1), q_ranges{2}(2), n_divs)
        for i3 = linspace(q_ranges{3}(1), q_ranges{3}(2), n_divs)
            for i4 = linspace(q_ranges{4}(1), q_ranges{4}(2), n_divs)
                for i5 = linspace(q_ranges{5}(1), q_ranges{5}(2), n_divs)
                    % T_current = subs(T, [q1, q2, q3, q4, q5], [i1, i2, i3, i4, i5]);
                    % T_current = double(T_current);
                    T_current = T_fast(i1,i2,i3,i4,i5);
                    xyz = T_current(1:3,4);
                    xyz = double(xyz);
                    pts_list(:,i) = xyz;
                    i = i+1;
                end
            end
        end
    end
end

scatter3(pts_list(1,:), pts_list(2,:), pts_list(3,:), 36, 'k');

function T = T_fast(q1, q2, q3, q4, q5)
T = [cos(q1)*(cos(q2 + pi/2)*(cos(q3)*(cos(q5)*cos(q4 - pi/2) - (4967757600021511*sin(q5)*sin(q4 - pi/2))/81129638414606681695789005144064) - sin(q3)*(cos(q5)*sin(q4 - pi/2) + (4967757600021511*cos(q4 - pi/2)*sin(q5))/81129638414606681695789005144064)) - sin(q2 + pi/2)*(cos(q3)*(cos(q5)*sin(q4 - pi/2) + (4967757600021511*cos(q4 - pi/2)*sin(q5))/81129638414606681695789005144064) + sin(q3)*(cos(q5)*cos(q4 - pi/2) - (4967757600021511*sin(q5)*sin(q4 - pi/2))/81129638414606681695789005144064))) - sin(q1)*(sin(q5) + (4967757600021511*cos(q2 + pi/2)*(cos(q3)*(cos(q5)*sin(q4 - pi/2) + (4967757600021511*cos(q4 - pi/2)*sin(q5))/81129638414606681695789005144064) + sin(q3)*(cos(q5)*cos(q4 - pi/2) - (4967757600021511*sin(q5)*sin(q4 - pi/2))/81129638414606681695789005144064)))/81129638414606681695789005144064 + (4967757600021511*sin(q2 + pi/2)*(cos(q3)*(cos(q5)*cos(q4 - pi/2) - (4967757600021511*sin(q5)*sin(q4 - pi/2))/81129638414606681695789005144064) - sin(q3)*(cos(q5)*sin(q4 - pi/2) + (4967757600021511*cos(q4 - pi/2)*sin(q5))/81129638414606681695789005144064)))/81129638414606681695789005144064), - sin(q1)*(cos(q2 + pi/2)*(cos(q3)*(cos(q5)*cos(q4 - pi/2) - (4967757600021511*sin(q5)*sin(q4 - pi/2))/81129638414606681695789005144064) - sin(q3)*(cos(q5)*sin(q4 - pi/2) + (4967757600021511*cos(q4 - pi/2)*sin(q5))/81129638414606681695789005144064)) - sin(q2 + pi/2)*(cos(q3)*(cos(q5)*sin(q4 - pi/2) + (4967757600021511*cos(q4 - pi/2)*sin(q5))/81129638414606681695789005144064) + sin(q3)*(cos(q5)*cos(q4 - pi/2) - (4967757600021511*sin(q5)*sin(q4 - pi/2))/81129638414606681695789005144064))) - cos(q1)*(sin(q5) + (4967757600021511*cos(q2 + pi/2)*(cos(q3)*(cos(q5)*sin(q4 - pi/2) + (4967757600021511*cos(q4 - pi/2)*sin(q5))/81129638414606681695789005144064) + sin(q3)*(cos(q5)*cos(q4 - pi/2) - (4967757600021511*sin(q5)*sin(q4 - pi/2))/81129638414606681695789005144064)))/81129638414606681695789005144064 + (4967757600021511*sin(q2 + pi/2)*(cos(q3)*(cos(q5)*cos(q4 - pi/2) - (4967757600021511*sin(q5)*sin(q4 - pi/2))/81129638414606681695789005144064) - sin(q3)*(cos(q5)*sin(q4 - pi/2) + (4967757600021511*cos(q4 - pi/2)*sin(q5))/81129638414606681695789005144064)))/81129638414606681695789005144064), cos(q2 + pi/2)*(cos(q3)*(cos(q5)*sin(q4 - pi/2) + (4967757600021511*cos(q4 - pi/2)*sin(q5))/81129638414606681695789005144064) + sin(q3)*(cos(q5)*cos(q4 - pi/2) - (4967757600021511*sin(q5)*sin(q4 - pi/2))/81129638414606681695789005144064)) - (4967757600021511*sin(q5))/81129638414606681695789005144064 + sin(q2 + pi/2)*(cos(q3)*(cos(q5)*cos(q4 - pi/2) - (4967757600021511*sin(q5)*sin(q4 - pi/2))/81129638414606681695789005144064) - sin(q3)*(cos(q5)*sin(q4 - pi/2) + (4967757600021511*cos(q4 - pi/2)*sin(q5))/81129638414606681695789005144064)), (cos(q3)*(cos(q5)*cos(q4 - pi/2) - (4967757600021511*sin(q5)*sin(q4 - pi/2))/81129638414606681695789005144064))/5 - (4967757600021511*sin(q5))/81129638414606681695789005144064 + (cos(q5)*cos(q4 - pi/2))/5 - (sin(q3)*(cos(q5)*sin(q4 - pi/2) + (4967757600021511*cos(q4 - pi/2)*sin(q5))/81129638414606681695789005144064))/5 - (4967757600021511*sin(q5)*sin(q4 - pi/2))/405648192073033408478945025720320 + cos(q2 + pi/2)*(cos(q3)*(cos(q5)*sin(q4 - pi/2) + (4967757600021511*cos(q4 - pi/2)*sin(q5))/81129638414606681695789005144064) + sin(q3)*(cos(q5)*cos(q4 - pi/2) - (4967757600021511*sin(q5)*sin(q4 - pi/2))/81129638414606681695789005144064)) + sin(q2 + pi/2)*(cos(q3)*(cos(q5)*cos(q4 - pi/2) - (4967757600021511*sin(q5)*sin(q4 - pi/2))/81129638414606681695789005144064) - sin(q3)*(cos(q5)*sin(q4 - pi/2) + (4967757600021511*cos(q4 - pi/2)*sin(q5))/81129638414606681695789005144064));
cos(q1)*(cos(q2 + pi/2)*(cos(q3)*((4967757600021511*cos(q5)*sin(q4 - pi/2))/81129638414606681695789005144064 + cos(q4 - pi/2)*sin(q5)) + sin(q3)*((4967757600021511*cos(q5)*cos(q4 - pi/2))/81129638414606681695789005144064 - sin(q5)*sin(q4 - pi/2))) + sin(q2 + pi/2)*(cos(q3)*((4967757600021511*cos(q5)*cos(q4 - pi/2))/81129638414606681695789005144064 - sin(q5)*sin(q4 - pi/2)) - sin(q3)*((4967757600021511*cos(q5)*sin(q4 - pi/2))/81129638414606681695789005144064 + cos(q4 - pi/2)*sin(q5)))) + sin(q1)*(cos(q5) + (4967757600021511*cos(q2 + pi/2)*(cos(q3)*((4967757600021511*cos(q5)*cos(q4 - pi/2))/81129638414606681695789005144064 - sin(q5)*sin(q4 - pi/2)) - sin(q3)*((4967757600021511*cos(q5)*sin(q4 - pi/2))/81129638414606681695789005144064 + cos(q4 - pi/2)*sin(q5))))/81129638414606681695789005144064 - (4967757600021511*sin(q2 + pi/2)*(cos(q3)*((4967757600021511*cos(q5)*sin(q4 - pi/2))/81129638414606681695789005144064 + cos(q4 - pi/2)*sin(q5)) + sin(q3)*((4967757600021511*cos(q5)*cos(q4 - pi/2))/81129638414606681695789005144064 - sin(q5)*sin(q4 - pi/2))))/81129638414606681695789005144064),   cos(q1)*(cos(q5) + (4967757600021511*cos(q2 + pi/2)*(cos(q3)*((4967757600021511*cos(q5)*cos(q4 - pi/2))/81129638414606681695789005144064 - sin(q5)*sin(q4 - pi/2)) - sin(q3)*((4967757600021511*cos(q5)*sin(q4 - pi/2))/81129638414606681695789005144064 + cos(q4 - pi/2)*sin(q5))))/81129638414606681695789005144064 - (4967757600021511*sin(q2 + pi/2)*(cos(q3)*((4967757600021511*cos(q5)*sin(q4 - pi/2))/81129638414606681695789005144064 + cos(q4 - pi/2)*sin(q5)) + sin(q3)*((4967757600021511*cos(q5)*cos(q4 - pi/2))/81129638414606681695789005144064 - sin(q5)*sin(q4 - pi/2))))/81129638414606681695789005144064) - sin(q1)*(cos(q2 + pi/2)*(cos(q3)*((4967757600021511*cos(q5)*sin(q4 - pi/2))/81129638414606681695789005144064 + cos(q4 - pi/2)*sin(q5)) + sin(q3)*((4967757600021511*cos(q5)*cos(q4 - pi/2))/81129638414606681695789005144064 - sin(q5)*sin(q4 - pi/2))) + sin(q2 + pi/2)*(cos(q3)*((4967757600021511*cos(q5)*cos(q4 - pi/2))/81129638414606681695789005144064 - sin(q5)*sin(q4 - pi/2)) - sin(q3)*((4967757600021511*cos(q5)*sin(q4 - pi/2))/81129638414606681695789005144064 + cos(q4 - pi/2)*sin(q5)))), (4967757600021511*cos(q5))/81129638414606681695789005144064 - cos(q2 + pi/2)*(cos(q3)*((4967757600021511*cos(q5)*cos(q4 - pi/2))/81129638414606681695789005144064 - sin(q5)*sin(q4 - pi/2)) - sin(q3)*((4967757600021511*cos(q5)*sin(q4 - pi/2))/81129638414606681695789005144064 + cos(q4 - pi/2)*sin(q5))) + sin(q2 + pi/2)*(cos(q3)*((4967757600021511*cos(q5)*sin(q4 - pi/2))/81129638414606681695789005144064 + cos(q4 - pi/2)*sin(q5)) + sin(q3)*((4967757600021511*cos(q5)*cos(q4 - pi/2))/81129638414606681695789005144064 - sin(q5)*sin(q4 - pi/2))), (4967757600021511*cos(q5))/81129638414606681695789005144064 + (cos(q3)*((4967757600021511*cos(q5)*sin(q4 - pi/2))/81129638414606681695789005144064 + cos(q4 - pi/2)*sin(q5)))/5 + (sin(q3)*((4967757600021511*cos(q5)*cos(q4 - pi/2))/81129638414606681695789005144064 - sin(q5)*sin(q4 - pi/2)))/5 + (4967757600021511*cos(q5)*sin(q4 - pi/2))/405648192073033408478945025720320 + (cos(q4 - pi/2)*sin(q5))/5 - cos(q2 + pi/2)*(cos(q3)*((4967757600021511*cos(q5)*cos(q4 - pi/2))/81129638414606681695789005144064 - sin(q5)*sin(q4 - pi/2)) - sin(q3)*((4967757600021511*cos(q5)*sin(q4 - pi/2))/81129638414606681695789005144064 + cos(q4 - pi/2)*sin(q5))) + sin(q2 + pi/2)*(cos(q3)*((4967757600021511*cos(q5)*sin(q4 - pi/2))/81129638414606681695789005144064 + cos(q4 - pi/2)*sin(q5)) + sin(q3)*((4967757600021511*cos(q5)*cos(q4 - pi/2))/81129638414606681695789005144064 - sin(q5)*sin(q4 - pi/2)));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              sin(q1)*((4967757600021511*sin(q2 + pi/2)*(cos(q3)*sin(q4 - pi/2) + cos(q4 - pi/2)*sin(q3)))/81129638414606681695789005144064 - (4967757600021511*cos(q2 + pi/2)*(cos(q3)*cos(q4 - pi/2) - sin(q3)*sin(q4 - pi/2)))/81129638414606681695789005144064 + 4967757600021511/81129638414606681695789005144064) - cos(q1)*(sin(q2 + pi/2)*(cos(q3)*cos(q4 - pi/2) - sin(q3)*sin(q4 - pi/2)) + cos(q2 + pi/2)*(cos(q3)*sin(q4 - pi/2) + cos(q4 - pi/2)*sin(q3))),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 cos(q1)*((4967757600021511*sin(q2 + pi/2)*(cos(q3)*sin(q4 - pi/2) + cos(q4 - pi/2)*sin(q3)))/81129638414606681695789005144064 - (4967757600021511*cos(q2 + pi/2)*(cos(q3)*cos(q4 - pi/2) - sin(q3)*sin(q4 - pi/2)))/81129638414606681695789005144064 + 4967757600021511/81129638414606681695789005144064) + sin(q1)*(sin(q2 + pi/2)*(cos(q3)*cos(q4 - pi/2) - sin(q3)*sin(q4 - pi/2)) + cos(q2 + pi/2)*(cos(q3)*sin(q4 - pi/2) + cos(q4 - pi/2)*sin(q3))),                                                                                                                                                                                                                                                                                                                       cos(q2 + pi/2)*(cos(q3)*cos(q4 - pi/2) - sin(q3)*sin(q4 - pi/2)) - sin(q2 + pi/2)*(cos(q3)*sin(q4 - pi/2) + cos(q4 - pi/2)*sin(q3)) + 24678615572571482867467662723121/6582018229284824168619876730229402019930943462534319453394436096,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            cos(q2 + pi/2)*(cos(q3)*cos(q4 - pi/2) - sin(q3)*sin(q4 - pi/2)) - sin(q2 + pi/2)*(cos(q3)*sin(q4 - pi/2) + cos(q4 - pi/2)*sin(q3)) - (cos(q3)*sin(q4 - pi/2))/5 - (cos(q4 - pi/2)*sin(q3))/5 - sin(q4 - pi/2)/5 + 9873027343927236252929815095344226422974278051215816518405269749/32910091146424120843099383651147010099654717312671597266972180480;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                1];
 
end


function tm = invert_mat(T)
    R = T(1:3,1:3);
    d = T(1:3,4);
    tm = [R', R'*-d; 0 0 0 1];
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


