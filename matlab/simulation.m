%% 3D Vision Group 1: Using Lines for Pose Estimation
clear all, close all, clc

%% Load Parameters
parameterFile = 'parameters.mat';
load(parameterFile);

%% generate 3D points

% Initialize random number generator
rng(0,'twister')

% Generate 3D Pointcloud
pointcloud3D = Pointcloud3D(numberOfPoints,shape,scale,...
    anisotropicGaussianMean,anisotropicGaussianVariance);

% Plot the point cloud of the true points
figure(1)
pointcloud3D.plotTruePointcloud()
axis vis3d
hold on

% initialize camera
Cam = Camera(cameraRadius, polarAngleMax);
[P, ~] = Cam.getPose();

% plot camera position
Cam.visualizeCamera(1)

% Initialize camera
T_WC = [[1 0 0; 0 sqrt(3)/2 1/2; 0 -1/2 sqrt(3)/2] [0,1,1]'; zeros(1,3) 1];

% add noise to 3d points
% pointcloud3D.addNoiseToAllPoints(T_WC)
% 
% hold on
% 
% pointcloud3D.plotNoisyPointcloud()

% project 3d to 2d points

% add pixel noise to 2d projection

% estimate camera pose with PnP algorithm