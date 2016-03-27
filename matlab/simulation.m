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

% Initialize camera with random azimutal and polar angle
azimutalAngle = 2*pi*rand();
polarAngle = polarAngleMax*rand();
Cam = Camera(cameraRadius, azimutalAngle, polarAngle, focalLength, px, py, mx, my, skew);

% Plot camera true pose and estimated pose of the camera
Cam.visualizeCamera(1)

% Add noise to 3d points
[trueP, ~] = Cam.getPose();
pointcloud3D.addNoiseToAllPoints(trueP)

% Plot the point cloud of the noisy points
pointcloud3D.plotNoisyPointcloud('false')

% project 3d to 2d points

% add pixel noise to 2d projection

% estimate camera pose with PnP algorithm