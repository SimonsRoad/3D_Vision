%% 3D Vision Group 1: Using Lines for Pose Estimation
clear all, close all, clc

%% Load Parameters
parameterFile = 'parameters.mat';
load(parameterFile);

%% generate 3D points

% Initialize random number generator
% rng(0,'twister')

% Generate 3D Pointcloud
pointcloud3D = Pointcloud3D(numberOfPoints,shape,scale,...
    anisotropicGaussianMean,anisotropicGaussianVariance);

% Initialize camera with random azimutal and polar angle
azimutalAngle = 2*pi*rand();
polarAngle = polarAngleMax*rand();
camera = Camera(cameraRadius, azimutalAngle, polarAngle, focalLength, px, py, mx, my, skew);

% Copy pointcloud3D to pointCloud3D of camera
camera.pointCloud3D = pointcloud3D;

% Plot the point cloud of the true points
figure(1)
camera.pointCloud3D.plotTruePointcloud()
axis vis3d
hold on

% Add noise to 3d points
[trueP, ~] = camera.getPose();
camera.pointCloud3D.addNoiseToAllPoints(trueP)

% Plot the point cloud of the noisy points
camera.pointCloud3D.plotNoisyPointcloud('true')

% Project 3d to 2d points
camera.projectFrom3DTo2D();

% Plot 2D points
camera.plot2DPoints(2);

% add pixel noise to 2d projection

% estimate camera pose with PnP algorithm
camera.setPnPAlgorithm(pnpAlgorithm);
camera.estimatePose();

% Plot camera true pose and estimated pose of the camera
camera.visualizeCamera(1)
