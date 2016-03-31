clear all

%% Define Parameters for the 3D Vision class project

%% Scene Parameters
scale = 1;
numberOfPoints = 200;

shape = 'planar'; % Options: 'spherical', 'planar', 'cubic'
pointCloudRadius = 1;
cameraRadius = 1.5;
polarAngleMax = pi/3;

% Scale
pointCloudRadius = scale*pointCloudRadius;
cameraRadius = scale*cameraRadius;

%% Camera Parameters
focalLength = 0.5;
xResolution = 640; % pixel resolution in x-direction of camera frame
yResolution = 480; % pixel resolution in y-direction of camera frame
x0 = 0; % principle point in the middle of sensor
y0 = 0; % principle point in the middle of sensor
skew = 0;
kx = 1200;
ky = 1200;

%% PnP Parameters
pnpAlgorithm = 'LHM'; % Options: 'EPNP','EPNP-Gauss', 'DLT', 'LHM', 'RPNP'

%% PointIn3D
% Noise Models
pointIn3DNoiseModel = 'anisotropicGaussian';
% Anisotropic Gaussian Noise: coordinates [X; Y; Z] refer to camera frame
anisotropicGaussianMean = scale*[0.0; 0.0; 0.0];
anisotropicGaussianVariance = scale*[0.001; 0.001; 0.01];

%% Distortion parameters
kappa = [0;0;0];                % radial distortion
p = [0;0];                      % tangential distortion

%% Save Parameter File

save('parameters.mat')

clear all