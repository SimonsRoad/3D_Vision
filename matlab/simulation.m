%% 3D Vision Group 1: Using Lines for Pose Estimation
clear all, close all, clc;

%% Load Parameters
parameterFile = 'parameters.mat';
load(parameterFile);

%% Set up experiments
limitOfX = 10;
numberOfExperiments = 50;

% rows: different number of lines starting from the smallest
% columns: error for DLT, error for nonlinear optimization with points,
% error for nonlinear optimization with points and lines
meanErrorInPosition = zeros(limitOfX, 3);
meanErrorInOrientation = zeros(limitOfX, 3);
varianceInPosition = zeros(limitOfX, 3);
varianceInOrientation = zeros(limitOfX, 3);

for i = 1:limitOfX
    
    % Initialize random number generator
    rng('shuffle','twister')
    
    numberOfLines = i;
    
for j = 1:numberOfExperiments
    
    errorInPosition = zeros(numberOfExperiments, 3);
    errorInOrientation = zeros(numberOfExperiments, 3);

%% generate 3D points
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

% Compute coordinates in camera frame of points and lines
[trueP, ~] = camera.getPose();
camera.pointCloud3D.computeCameraFrameCoordinates(trueP);
camera.lineCloud3D.computeCameraFrameCoordinates(trueP);

% Add noise to 3d points and lines
camera.pointCloud3D.addNoiseToAllPoints(trueP);
camera.lineCloud3D.addNoiseToAllLines(trueP);

%% Project true 3D points to 2D points
% Project 3d to 2d points
camera.projectPointsFrom3DTo2D();

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

% undistortion in image coordinates
camera.undistortion();

%% Project true 3D lines to 2D lines
% Project 3D to 2D pixel lines
camera.projectLinesFrom3DTo2D();

% figure(13)
camera.sampleLines(numberOfSamples);
camera.measuerementProcess(kappa, p, 'gaussian', linecloudMean, linecloudVariance);


%% Pose Estimation
% estimate camera pose with PnP algorithm
camera.setPnPAlgorithm(pnpAlgorithm);
camera.estimatePose();
camera.optimizePoseEstimation();
camera.optimizePoseEstimationWithLines();

[errorInPositionLin, errorInOrientationLin] = camera.computePoseError(camera.estimatedPose);
errorInPosition(j,1) = errorInPositionLin;
errorInOrientation(j,1) = errorInOrientationLin;
[errorInPositionPoints, errorInOrientationPoints] = camera.computePoseError(camera.optimizedEstimatedPose);
errorInPosition(j,2) = errorInPositionPoints;
errorInOrientation(j,2) = errorInOrientationPoints;
[errorInPositionLines, errorInOrientationLines] = camera.computePoseError(camera.optimizedEstimatedPoseWithLines);
errorInPosition(j,3) = errorInPositionLines;
errorInOrientation(j,3) = errorInOrientationLines;


% % Plot true point cloud, noisy point cloud, true camera pose and estimated
% % camera pose
% fig9 = figure(9);
% camera.pointCloud3D.plotTruePointcloud();
% axis vis3d
% hold on
% camera.visualizeTrueCamera(9);
% camera.pointCloud3D.plotNoisyPointcloud('false');
% camera.lineCloud3D.plotTrueLinecloud();
% camera.lineCloud3D.plotNoisyLinecloud();
% % camera.plotConfidenceIntervals();
% camera.visualizeEstimatedCamera(9);
% camera.visualizeOptimizedCamera(9);
% camera.visualizeOptimizedCameraWithLines(9);
% hold off

end

meanErrorInPosition(i, 1) = mean(errorInPosition(:,1));
meanErrorInPosition(i, 2) = mean(errorInPosition(:,2));
meanErrorInPosition(i, 3) = mean(errorInPosition(:,3));

varianceInPosition(i, 1) = var(errorInPosition(:,1));
varianceInPosition(i, 2) = var(errorInPosition(:,2));
varianceInPosition(i, 3) = var(errorInPosition(:,3));

meanErrorInOrientation(i,1) = mean(errorInOrientation(:,1));
meanErrorInOrientation(i,2) = mean(errorInOrientation(:,2));
meanErrorInOrientation(i,3) = mean(errorInOrientation(:,3));

varianceInOrientation(i, 1) = var(errorInOrientation(:,1));
varianceInOrientation(i, 2) = var(errorInOrientation(:,2));
varianceInOrientation(i, 3) = var(errorInOrientation(:,3));

end
disp('Mean error in position')
disp(meanErrorInPosition)
disp('Mean error in orientation')
disp(meanErrorInOrientation)
disp('Variance in position')
disp(varianceInPosition)
disp('variance in orientation')
disp(varianceInOrientation)

figure(10)
hold on
plot(1:limitOfX,meanErrorInPosition(:,1))
plot(1:limitOfX,meanErrorInPosition(:,2))
plot(1:limitOfX,meanErrorInPosition(:,3))
title('Mean Error In Position')
legend('DLT','Points','Points+Lines')
hold off

figure(11)
hold on
plot(1:limitOfX,meanErrorInOrientation(:,1))
plot(1:limitOfX,meanErrorInOrientation(:,2))
plot(1:limitOfX,meanErrorInOrientation(:,3))
title('Mean Error In Orientation')
legend('DLT','Points','Points+Lines')
hold off

%%

figure(12)
hold on
plot(1:limitOfX,varianceInPosition(:,1))
plot(1:limitOfX,varianceInPosition(:,2))
plot(1:limitOfX,varianceInPosition(:,3))
title('Variance In Position')
legend('DLT','Points','Points+Lines')
hold off
 %% 
figure(13)
colormap(linspecer)
hold on
plot(1:limitOfX,varianceInOrientation(:,1))
plot(1:limitOfX,varianceInOrientation(:,2))
plot(1:limitOfX,varianceInOrientation(:,3))
title('Variance In Orientation')
legend('DLT','Points','Points+Lines')
hold off


