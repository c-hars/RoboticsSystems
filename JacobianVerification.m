







% clear

%%

% qspan = 0:0.05:pi;
% Q = repmat(qspan, 5,1)';


%%

% q_vals = 0:0.1:1;
% q_f = q_vals(end);
% n_qi = length(q_vals);
% 
% Q = [ [zeros(n_qi*1,0); q_vals'; zeros(n_qi*4,1)] ...
%       [zeros(n_qi*1,1); q_vals'; zeros(n_qi*3,1)] ...
%       [zeros(n_qi*2,1); q_vals'; zeros(n_qi*2,1)] ...
%       [zeros(n_qi*3,1); q_vals'; zeros(n_qi*1,1)] ...
%       [zeros(n_qi*4,1); q_vals'; zeros(n_qi*0,1)] ...
%       ];
% 
% Q(1*n_qi+1:end,1) = q_f;
% Q(2*n_qi+1:end,2) = q_f;
% Q(3*n_qi+1:end,3) = q_f;
% Q(4*n_qi+1:end,4) = q_f;


%% computation

Ts = 0.01;
tspan = 0:Ts:1;

Q = zeros(length(tspan), 5);
for i = 1:5
    Q(:,i) = sin(2*2*pi*rand*tspan + rand);
end
n = length(Q);

vel_numeric = zeros(n,3);
vel_analytic = zeros(n,3);
angvel_numeric = zeros(n,3);
angvel_analytic = zeros(n,3);


L = [0 0.4 0.4 0.05 0.05]; % lengths l_i

for i=2:n

    % compute the linear velocity using the analytic formula (Jacobian)
    Q_dot = (Q(i,:) - Q(i-1,:)) / Ts;
    vels = subs(jacobiann * Q_dot', [q_1 q_2 q_3 q_4 q_5], Q(i,:));
    vels = subs(vels, [l_1 l_2 l_3 l_4 l_5], L);
    vel_analytic(i,:) = double(vels(1:3));
    angvel_analytic(i,:) = double(vels(4:6));
    
    % numerically compute the linear velocity
    vel = compute_end_effector_pos_fast(Q(i,:),L) - compute_end_effector_pos_fast(Q(i-1,:),L);
    vel = vel / Ts;
    vel_numeric(i,:) = vel;
    
    % numerically compute the angular velocity
    %       C_05_dot = -S(w_05) * C_05
    %       S(w_05) = -C_05_dot * C_05'
    %       then extract the skew-symmetric entries of S(w_05) to get w_05, the ang vel of frame 5 relative to frame 0
    dRdt = compute_end_effector_orientation_fast(Q(i,:)) - compute_end_effector_orientation_fast(Q(i-1,:));
    dRdt = dRdt / Ts;
    Omega =  dRdt * compute_end_effector_orientation_fast(Q(i,:))';
    angvel_numeric(i,1) = Omega(3,2);
    angvel_numeric(i,2) = Omega(1,3);
    angvel_numeric(i,3) = Omega(2,1);

end



%% linear velocity plot

figure(1); clf

subplot(3,1,1); hold on; title("Linear velocity of end effector position")
tspan = (0:n-1) * Ts; 
plot(tspan,vel_numeric(:,1))
plot(tspan,vel_analytic(:,1))
ylabel('v\_x [m/s]')

subplot(3,1,2); hold on
tspan = (0:n-1) * Ts; 
plot(tspan,vel_numeric(:,2))
plot(tspan,vel_analytic(:,2))
ylabel('v\_y [m/s]')

subplot(3,1,3); hold on
tspan = (0:n-1) * Ts; 
plot(tspan,vel_numeric(:,3))
plot(tspan,vel_analytic(:,3))
xlabel('Time [s]')
ylabel('v\_z [m/s]')

legend('via FK', 'via Jacobian')


%% angular velocity plot

figure(2); clf

subplot(3,1,1); hold on; title("Angular velocity of end effector position")
tspan = (0:n-1) * Ts; 
plot(tspan,angvel_numeric(:,1))
plot(tspan,angvel_analytic(:,1))
ylabel('w\_x [m/s]')

subplot(3,1,2); hold on
tspan = (0:n-1) * Ts; 
plot(tspan,angvel_numeric(:,2))
plot(tspan,angvel_analytic(:,2))
ylabel('w\_y [m/s]')

subplot(3,1,3); hold on
tspan = (0:n-1) * Ts; 
plot(tspan,angvel_numeric(:,3))
plot(tspan,angvel_analytic(:,3))
xlabel('Time [s]')
ylabel('w\_z [m/s]')

legend('via FK', 'via Jacobian')


%% functions

function pos = compute_end_effector_pos_fast(q,L)

q_1 = q(1);
q_2 = q(2);
q_3 = q(3);
q_4 = q(4);
q_5 = q(5);

l_1 = L(1);
l_2 = L(2);
l_3 = L(3);
l_4 = L(4);
l_5 = L(5);

% our robot forward kinematics
x = l_5*(cos(q_1 + q_2 + q_3 + q_4)/2 + cos(q_2 - q_1 + q_3 + q_4)/2) + cos(q_1)*(l_1 + l_3*cos(q_2 + q_3) + l_2*cos(q_2) + l_4*cos(q_2 + q_3 + q_4));
y = sin(q_1)*(l_1 + l_3*cos(q_2 + q_3) + l_2*cos(q_2) + l_4*cos(q_2 + q_3 + q_4)) + l_5*(sin(q_1 + q_2 + q_3 + q_4)/2 - sin(q_2 - q_1 + q_3 + q_4)/2);
z = l_3*sin(q_2 + q_3) + l_2*sin(q_2) + l_4*sin(q_2 + q_3 + q_4) + l_5*sin(q_2 + q_3 + q_4);
pos = [x;y;z];

end

function R = compute_end_effector_orientation_fast(q)

q_1 = q(1);
q_2 = q(2);
q_3 = q(3);
q_4 = q(4);
q_5 = q(5);

% our robot forward kinematics
R_11 = -cos(q_5)*sin(q_2+q_3+q_4)*cos(q_1)+sin(q_1)*sin(q_5);
R_12 =  sin(q_5)*sin(q_2+q_3+q_4)*cos(q_1)+sin(q_1)*cos(q_5);
R_13 =           cos(q_2+q_3+q_4)*cos(q_1);

R_21 = -cos(q_5)*sin(q_2+q_3+q_4)*sin(q_1) - cos(q_1)*sin(q_5);
R_22 =  sin(q_5)*sin(q_2+q_3+q_4)*sin(q_1) - cos(q_1)*cos(q_5);
R_23 =           cos(q_2+q_3+q_4)*sin(q_1);

R_31 =  cos(q_5)*cos(q_2+q_3+q_4);
R_32 = -sin(q_5)*cos(q_2+q_3+q_4);
R_33 =           sin(q_2+q_3+q_4);

R = [R_11 R_12 R_13; ...
     R_21 R_22 R_23; ...
     R_31 R_32 R_33];


end