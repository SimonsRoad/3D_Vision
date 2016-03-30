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
Cam = Camera(cameraRadius, azimutalAngle, polarAngle, focalLength, px, py, mx, my, skew, xResolution, yResolution);

% Copy pointcloud3D to pointCloud3D of Cam
Cam.pointCloud3D = pointcloud3D;

% Plot the point cloud of the true points
figure(1)
Cam.pointCloud3D.plotTruePointcloud()
axis vis3d
hold on

% Plot camera true pose and estimated pose of the camera
Cam.visualizeCamera(1)

% Add noise to 3d points
[trueP, ~] = Cam.getPose();
Cam.pointCloud3D.addNoiseToAllPoints(trueP)

% Plot the point cloud of the noisy points
Cam.pointCloud3D.plotNoisyPointcloud('false')

% Project 3d to 2d points
Cam.projectFrom3DTo2D();

% Plot 2D points
Cam.plot2DPoints(2);

% add pixel noise to 2d projection

% estimate camera pose with PnP algorithm