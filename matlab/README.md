## Pose Estimation using Point and Line Correspondences


### Matlab
The code runs on Matlab

For the first run and to actualize the parameters run

**writeParameterFile.m**

### Modify constants in writeParameterFile.m

This section explains the constants with default settings which can be modified.

**scale = 1**

with **scale** you define the size of the scene

#### Scene Parameters
**numberOfPoints = 100**									
define the number of points used for the estimation

**numberOfLines = 4** 						

define the number of lines used for the estimation

**shape = 'spherical'**

shape decides in what shape range the points and lines are distributed, choose between 'spherical', 'planar' and 'cubic'

**pointCloudRadius = 1**

set the radius of the point cloud, which is basically the limit position of the outermost points in the world frame

**cameraRadius = 2, polarAngleMax = pi/3**

the initial camera position is modeled on a sphere with cameraRadius as radius and polarAngleMax as the maximum angle

#### Camera Parameters

**focalLength = 0.5**

set the focal length with focalLength

**xResolution = 640**								
pixel resolution in x-direction of camera frame

**yResolution = 480**

pixel resolution in y-direction of camera frame

**x0 = xResolution/2**

set the x-position of the principle point, DEFAULT: principle point in the middle of sensor

**y0 = yResolution/2**

set the y-position of the principle point
DEFAULT: principle point in the middle of sensor

**skew = 0**

set the shape of the pixel
DEFAULT: for square pixels skew = 0

**kx = 1200** 

set the inverse of the pixel length

**ky = 1200** 

set the inverse of the pixel height

#### PnP Algorithm

**pnpAlgorithm = 'DLT'** 

select a PnP algorithm between 'EPNP', 'EPNP-Gauss', 'DLT', 'LHM', 'RPNP'

#### Anisotropic Gaussian

Noisy 3D points and lines are modeled by anisotripic Gaussain model

**anisotropicGaussianMean = scale*[0.0; 0.0; 0.0]**

set the mean of the Gaussian model for points

**anisotropicGaussianVariance = scale*[0.01; 0.01; 0.1]**

set the variance of the Gaussian model for points

**linecloudMean = scale*[0.0; 0.0; 0.0]**

set the mean of the Gaussian model for lines

**linecloudVariance = scale*[0.0; 0.0; 0.2]**

set the variance of the Gaussian model for lines

**numberOfSamples = 10**

#### Distortion Parameters

**kappa = [1;1;0]**

set the paramerts for radial distortion [k_1, k_2, k_3]

**p = [0;0]**

set the paramerts for tangential distortion [p_1, p_2]

#### Pixel Noise

Pixel Noise is also Gaussian distributed. Set mean by pixelNoiseMean and variance by pixelNoiseVariance

**pixelNoiseMean = [0,0]**

**pixelNoiseVariance = [0,0]**









