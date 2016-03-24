clear all

%% Define Parameters for the 3D Vision class project

%% Scene Parameters
scale = 1;
numberOfPoints = 500;

shape = 'cubic'; % Options: 'spherical', 'planar', 'cubic'
pointCloudRadius = 1;
cameraRadius = 2;
polarAngleMax = pi/3;

% Scale
pointCloudRadius = scale*pointCloudRadius;
cameraRadius = scale*cameraRadius;

%% PointIn3D
% Noise Models
pointIn3DNoiseModel = 'anisotropicGaussian';
% Anisotropic Gaussian Noise: coordinates [X; Y; Z] refer to camera frame
anisotropicGaussianMean = scale*[0.0; 0.0; 0.0];
anisotropicGaussianVariance = scale*[0.001; 0.001; 0.01];

%% Save Parameter File

save('parameters.mat')

clear all