%% Move Robot Sample Code

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Nicola Bezzo (UVA)
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

%Velocity (constant for this demo example) 
vel = 5;

%Steering angle
steering = pi/4; 

%Initialize a vector of positions for the robot
x=[]; 
y=[];

%% Robot Initial Pose

x(1) = 100; % Given in HW Doc
y(1) = 100;

% Initial Orientation 
theta(1) = 2*pi*rand; % random theta in radians

% Build Robot Model
robot = RectangularRobot(x,y,theta(1));

plot(robot(:,1),robot(:,2),'-');
xlim([0 200])
ylim([0 200])
    
%% Move Robot

%number of steps of the simualtion
nstep = 500;

%time step
dt = 0.1;

 
for i = 1:nstep
    
    %robot non-holonomic dynamics (as seen in class)
    x(i+1) = x(i) + vel * cos(theta(i)) * dt;
    y(i+1) = y(i) + vel * sin(theta(i)) * dt;
    theta(i+1) = theta(i) + steering * dt;
    
    robot = RectangularRobot(x(i),y(i),theta(i));
    plot(robot(:,1),robot(:,2),'-',x,y,'-');
    xlim([0 200])
    ylim([0 200])
    pause(0.01)
    
end
