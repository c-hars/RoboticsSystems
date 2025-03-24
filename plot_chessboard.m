function plot_chessboard(midpoint, L, W, H)
% plot_chessboard([1 1 0.2], 0.3, 0.3, 0.02) will plot a chessboard as follows
%   - centre of the chessboard at (x,y) = (1,1)
%   - bottom face of the chessboard sits on the z=0.2 plane
%   - side dimensions: 0.3, 0.3
%   - height: 0.2

    dx = L / 2;
    dy = W / 2;
    dz = H;

    % Define the 8 vertices of the rectangular prism    
    vertices = [midpoint(1) - dx, midpoint(2) - dy, midpoint(3);
                midpoint(1) + dx, midpoint(2) - dy, midpoint(3);
                midpoint(1) + dx, midpoint(2) + dy, midpoint(3);
                midpoint(1) - dx, midpoint(2) + dy, midpoint(3);
                midpoint(1) - dx, midpoint(2) - dy, midpoint(3) + dz;
                midpoint(1) + dx, midpoint(2) - dy, midpoint(3) + dz;
                midpoint(1) + dx, midpoint(2) + dy, midpoint(3) + dz;
                midpoint(1) - dx, midpoint(2) + dy, midpoint(3) + dz];

    % Define the faces using the vertices
    faces = [1 2 3 4; % Bottom face
             5 6 7 8; % Top face
             1 2 6 5; % Side face 1
             2 3 7 6; % Side face 2
             3 4 8 7; % Side face 3
             4 1 5 8]; % Side face 4

    % Plot the rectangular prism
    patch('Vertices', vertices, 'Faces', faces, ...
          'FaceColor', 'b', 'EdgeAlpha', 0.4, 'FaceAlpha', 0.1);

end