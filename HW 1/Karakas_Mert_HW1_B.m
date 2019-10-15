%% AMR HW 1: Homogeneous Transformation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Mert Karakas (UVA)
% AMR 2019 
% Date: 09/15/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
close all
clc

%% Parameters

% Workspace Size
xlim([0 200])
ylim([0 200]) 

%Initialize a vector of positions for the robot
x=[]; 
y=[];
vel=[];
t=[];

%% Robot Initial Pose

M = 13; % M
K = 11; % K
Group8 = pi/(5 + 8); % Group 8

x(1) = M; % Given in HW Doc
y(1) = K;

% Initial Orientation 
theta(1) = Group8; % random theta in radians

% Build Robot Model
robot = RectangularRobot(x,y,theta(1));

plot(robot(:,1),robot(:,2),'-');
xlim([0 200])
ylim([0 200])
    
%% Move Robot

Kv = 0.3;

% Goal
xg = 200*rand(1,1);
yg = 200*rand(1,1);

%number of steps of the simualtion
nstep = 1000;

%time step
dt = 0.1;
i = 1;
 
while (euclidean_distance(x(i), y(i), xg, yg) >= 0.1)
    t(i) = i*dt;
    vel(i) = Kv * sqrt((xg - x(i))^2 + (yg - y(i))^2);
    %robot non-holonomic dynamics (as seen in class)
    
    x(i+1) = x(i) + vel(i) * cos(theta(i)) * dt;
    y(i+1) = y(i) + vel(i) * sin(theta(i)) * dt;
    theta(i+1) = atan2((yg - y(i)), (xg - x(i)));
    
    robot = RectangularRobot(x(i),y(i),theta(i));
    plot(robot(:,1),robot(:,2),'-',x,y,'-',xg, yg, 'r*');
    xlim([0 200])
    ylim([0 200])
    pause(0.01)
    i = i + 1;
end

figure(2);
plot(t,vel);


