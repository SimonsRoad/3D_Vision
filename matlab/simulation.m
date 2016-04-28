%% 3D Vision Group 1: Using Lines for Pose Estimation
clear all, close all, clc

%% Load Parameters
parameterFile = 'parameters.mat';
load(parameterFile);

%% generate 3D points
% Initialize random number generator
rng('shuffle','twister')

% Generate 3D Pointcloud
pointcloud3D = Pointcloud3D(numberOfPoints,shape,scale,...
    anisotropicGaussianMean,anisotropicGaussianVariance);

%% Generate 3D lines
% Generate 3D Linecloud
linecloud3D = Linecloud3D(numberOfLines, shape, scale, linecloudMean, linecloudVariance);

%% Initialize a Camera object
% Initialize camera with random azimutal and polar angle
azimutalAngle = 2*pi*rand();
polarAngle = polarAngleMax*rand();

% Construct Camera object
camera = Camera(cameraRadius, azimutalAngle, polarAngle, focalLength, x0, y0, kx, ky, skew, xResolution, yResolution);

% Copy pointcloud3D to pointCloud3D of camera
camera.pointCloud3D = pointcloud3D;

% Copy linecloud3D to lineCloud3D of camera
camera.lineCloud3D = linecloud3D;

% Add noise to 3d points
[trueP, ~] = camera.getPose();
camera.pointCloud3D.addNoiseToAllPoints(trueP);

camera.pointCloud3D.computeCameraFrameCoordinates(trueP);

%% Project true 3D lines to 2D lines
% Project 3D to 2D pixel lines
camera.projectLinesFrom3DToPixel();

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
disp(['Translation Error: ' num2str(errorInTranslation) ' [%]'])
disp(['Orientation Error: ' num2str(errorInOrientation) '   [degrees]'])

%% Plots
% Plot the point cloud of the true points
fig1 = figure(1);
camera.pointCloud3D.plotTruePointcloud();

% Plot camera true pose with true point cloud
fig2 = figure(2);
camera.pointCloud3D.plotTruePointcloud();
axis equal
hold on
camera.visualizeTrueCamera(2)
hold off

% Plot camera true pose, true point cloud and point cloud of the noisy points
fig3 = figure(3);
camera.pointCloud3D.plotTruePointcloud();
axis equal
hold on
camera.visualizeTrueCamera(3)
camera.pointCloud3D.plotNoisyPointcloud('false');
camera.plotConfidenceIntervals();
hold off

% Plot true 3D points projected to image plane
fig4 = figure(4);
camera.pointCloud2D.plotImagePoints();
title('Image Plane')
xlabel('')
ylabel('')
axis([-.5*xResolution/kx .5*xResolution/kx -.5*yResolution/ky .5*yResolution/ky])
legend('true 3D projection')

% Plot projected points and distorted points on image plane
fig5 = figure(5);
camera.pointCloud2D.plotImagePoints();
hold on
camera.pointCloud2D.plotDistortedImagePoints();
title('Image Plane')
xlabel('')
ylabel('')
axis([-.5*xResolution/kx .5*xResolution/kx -.5*yResolution/ky .5*yResolution/ky])
legend('true 3D projection','true 3D distorted projection')
hold off

% Plot projected, distorted and backprojected points on image plane
fig6 = figure(6);
camera.pointCloud2D.plotImagePoints();
hold on
camera.pointCloud2D.plotDistortedImagePoints();
camera.pointCloud2D.plotBackProjectedImagePoints();
title('Image Plane')
xlabel('')
ylabel('')
axis([-.5*xResolution/kx .5*xResolution/kx -.5*yResolution/ky .5*yResolution/ky])
legend('true 3D projection','true 3D distorted projection','back projection from pixel')

% Plot pixel representation of distorted image plane points
fig7 = figure(7);
camera.pointCloud2D.plotDistortedPixelPoints();
title('Pixel Space')
xlabel('')
ylabel('')
axis([0 xResolution 0 yResolution])
legend('distorted image points to pixel')

% Plot pixel and noisy pixel points
fig8 = figure(8);
camera.pointCloud2D.plotDistortedPixelPoints();
hold on
camera.pointCloud2D.plotNoisyPixelPoints();
title('Pixel Space')
xlabel('')
ylabel('')
axis([0 xResolution 0 yResolution])
legend('distorted image points to pixel','added pixel noise')
hold off

% Plot true point cloud, noisy point cloud, true camera pose and estimated
% camera pose
fig9 = figure(9);
camera.pointCloud3D.plotTruePointcloud();
axis equal
hold on
camera.visualizeTrueCamera(9)
camera.pointCloud3D.plotNoisyPointcloud('false');
camera.plotConfidenceIntervals();
camera.visualizeEstimatedCamera(9)
hold off