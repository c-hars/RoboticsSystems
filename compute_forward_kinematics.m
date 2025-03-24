function [T_0E, T_cell] = compute_forward_kinematics(DH_table)
    % INPUTS
    % DH_table: Denavit-Hartenburg table
    %   Column 1: a_{i-1}
    %   Column 2: alpha_{i-1}
    %   Column 3: d_{i}
    %   Column 4: theta_{i}
    % 
    % OUTPUTS
    % T_0E: transformation matrix from frame E to frame 0
    % T_cell: cell array containing the individual transformation matrices
    %   T_cell{i} contains T_{i-1}{i}, the transformation matrix from frame i to i-1
    
    n_frames = size(DH_table,1);
    T_cell = cell(n_frames,1);
    for i=1:n_frames
        % extract this row's parameters from the DH table
        a = DH_table(i,1);
        alpha = DH_table(i,2);
        d = DH_table(i,3);
        theta = DH_table(i,4);

        % calculate this row's transformation matrix
        T_cell{i} = D_x(a) * R_x(alpha) * D_z(d) * R_z(theta);
    end
    
    % compute T_0E, the transformation matrix from frame E to frame 0
    T_0E = [eye(3) [0;0;0]; [0 0 0 1]];
    for i=1:n_frames
        T_0E = T_0E * T_cell{i};
    end
    T_0E = simplify(T_0E);
    
end

function tm = R_x(theta)
    R = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
    tm = [R [0;0;0]; 0 0 0 1];
end

function tm = D_x(d)
    tm = [eye(3) [d;0;0]; 0 0 0 1];
end

function tm = R_z(theta)
    R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    tm = [R [0;0;0]; 0 0 0 1];
end

function tm = D_z(d)
    tm = [eye(3) [0;0;d]; 0 0 0 1];
end