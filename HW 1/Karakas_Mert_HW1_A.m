%% AMR HW 1: Homogeneous Transformation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Code by: Mert Karakas (UVA)
% AMR 2019 
% Date: 09/15/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
close all
clc
hold on
grid on
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

Bx = 13; % M
By = 11; % K
theta8 = pi/(5 + 8); % Group 8

robot = HomogeneousTransformation(x(1) + Bx, y(1) + By, theta(1) + theta8);
plot(robot(:,1),robot(:,2),'-');

