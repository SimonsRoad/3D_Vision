clear all

%% Define Parameters for the 3D Vision class project

%% Scene Parameters
scale = 1;
numberOfPoints = 1000;
shape = 'planar'; % Options: 'spherical', 'planar', quibic
pointCloudRadius = 1;
cameraRadius = 2;

% Scale
pointCloudRadius = scale*pointCloudRadius;
cameraRadius = scale*cameraRadius;

%% PointIn3D
% Noise Models
pointIn3DNoiseModel = 'anisotropicGaussian';
% Anisotropic Gaussian Noise: coordinates [X; Y; Z] refer to camera frame
anisotropicGaussianMean = [0.0; 0.0; 0.0];
anisotropicGaussianVariance = [0.01; 0.01; 0.01];

%% Save Parameter File

save('parameters.mat')

clear all