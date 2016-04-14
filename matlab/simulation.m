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

% Initialize camera with random azimutal and polar angle
azimutalAngle = 2*pi*rand();
polarAngle = polarAngleMax*rand();

%% Initialize a Camera object
% Construct Camera object
camera = Camera(cameraRadius, azimutalAngle, polarAngle, focalLength, x0, y0, kx, ky, skew, xResolution, yResolution);
% Copy pointcloud3D to pointCloud3D of camera
camera.pointCloud3D = pointcloud3D;

% Add noise to 3d points
[trueP, ~] = camera.getPose();
camera.pointCloud3D.addNoiseToAllPoints(trueP);

%% Project true 3D points to 2D points
% Project 3d to 2d points
camera.projectFrom3DTo2D();

% Add distortion to 2d projection
camera.addDistortion(kappa, p);

% calculate homoegeneous distorted pixels
camera.calculateHomoegenousDistortedPixelPoints();

% calculate euclidean pixels
camera.setDistortedPixelCoordinatesFromHomogeneousCoordinates();

% add pixel noise to 2d projection
camera.addPixelNoise('gaussian', pixelNoiseMean, pixelNoiseVariance);

% back transformation from pixel to image coordinates
camera.transformFromPixelToImage();

%% Pose Estimation
% estimate camera pose with PnP algorithm
camera.setPnPAlgorithm(pnpAlgorithm);
camera.estimatePose();

% compute error
[errorInTranslation, errorInOrientation] = camera.computePoseError();
disp(['Translation Error: ' num2str(errorInTranslation)])
disp(['Orientation Error: ' num2str(errorInOrientation)])

%% Plots
% Plot the point cloud of the true points
fig1 = figure(1)
camera.pointCloud3D.plotTruePointcloud();
axis vis3d
hold on

% Plot the point cloud of the noisy points
camera.pointCloud3D.plotNoisyPointcloud('false');

% Plot camera true pose and estimated pose of the camera
camera.visualizeCamera(1)
hold off

% Plot image plane
fig2 = figure(2)
camera.pointCloud2D.plotImagePoints();
title('Image Plane')
xlabel('')
ylabel('')
axis equal
hold on
legend('true 3D projection')
pause
camera.pointCloud2D.plotDistortedImagePoints();
legend('true 3D projection','true 3D distorted projection')
pause
hold off

% Plot pixel plane
fig3 = figure(3)
camera.pointCloud2D.plotDistortedPixelPoints();
title('Pixel space')
xlabel('')
ylabel('')
axis equal
hold on
legend('distorted image points to pixel')
pause
camera.pointCloud2D.plotNoisyPixelPoints();
legend('distorted image points to pixel','added pixel noise')
pause
hold off

% Plot pixel back projection to image plane
figure(fig2)
camera.pointCloud2D.plotImagePoints();
title('Image Plane')
xlabel('')
ylabel('')
axis equal
hold on
camera.pointCloud2D.plotDistortedImagePoints();
camera.pointCloud2D.plotBackProjectedImagePoints();
legend('true 3D projection','true 3D distorted projection','back projection from pixel')
hold off