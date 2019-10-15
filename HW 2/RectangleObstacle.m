%% RectangleObstacle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Mert Karakas (UVA)
% AMR 2019 
% Date: 10/11/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [robot] = RectangleObstacle(x,y,theta)

center = [x y];

% Robot rectangle shape

a = [-10 -10];
b = [10 -10];
c = [10 10];
d = [-10 10];

% Rotation Matrix

rotmat = [cos(theta) -sin(theta); sin(theta) cos(theta)];

rota = (rotmat * (a'));
rotb = (rotmat * (b'));
rotc = (rotmat * (c'));
rotd = (rotmat * (d'));

% Final Robot Configuration after transformation

robot1 = [rota(1) + center(1), rota(2) + center(2)];
robot2 = [rotb(1) + center(1), rotb(2) + center(2)];
robot3 = [rotc(1) + center(1), rotc(2) + center(2)];
robot4 = [rotd(1) + center(1), rotd(2) + center(2)];

robot = [robot1;robot2;robot3;robot4;robot1];
 
end


