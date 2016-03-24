%% 3D Vision Group 1: Using Lines for Pose Estimation
clear all, close all, clc

%% Load Parameters
parameterFile = 'parameters.mat';
load(parameterFile);

%% generate 3D points

% Initialize random number generator
rng(0,'twister')

% Generate 3D Pointcloud
truePointcloud3D = generateTruePointcloud3D(numberOfPoints,shape,scale);

% Plot the point cloud of the true points
visualizePointcloud3D(truePointcloud3D);


% initialize camera

% add noise to 3d points
% Note: We have to do this after we initialize the camera pose as we
% have to add the noise in the camera frame. This is because in a realistic
% setting, the uncertainty in depth (Z-direction of camera frame) than in
% the other two directions of the camera frame.

% project 3d to 2d points

% add pixel noise to 2d projection

% estimate camera pose with PnP algorithm