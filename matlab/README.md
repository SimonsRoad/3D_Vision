## Pose Estimation using Point and Line Correspondences


### Running Code

The code runs on **Matlab**. For the first run and to actualize the parameters run

**writeParameterFile.m**

Afterwards, to see the results, run

**simulation.m**


### Modify constants in writeParameterFile.m

This section explains the constants with example settings in writeParameterFile.m, which can be modified.

#### Scene Parameters

**numberOfPoints = 100** number of points used for the estimation

**numberOfLines = 4** number of lines used for the estimation

**shape = 'spherical'** range of the position of 3D points and lines, selectable between 'spherical', 'planar' and 'cubic'

**pointCloudRadius = 1** radius of the ground truth point cloud, which is basically the limit position of the outermost points in the world frame

**cameraRadius = 2, polarAngleMax = pi/3** the camera ground truth position is modeled on a sphere with cameraRadius as radius and polarAngleMax as the maximum angle


#### Camera Parameters

**focalLength = 0.5** defines the focal length

**xResolution = 640** pixel resolution in x-direction of camera frame

**yResolution = 480** pixel resolution in y-direction of camera frame

**x0 = xResolution/2** x-position of the principle point 
DEFAULT: principle point in the middle of sensor

**y0 = yResolution/2** y-position of the principle point
DEFAULT: principle point in the middle of sensor

**skew = 0** set the shape of the pixel
DEFAULT: for square pixels skew = 0

**kx = 1200** inverse of the pixel length

**ky = 1200** inverse of the pixel height


#### PnP Algorithm

**pnpAlgorithm = 'DLT'** PnP algorithm, selectable between 'EPNP', 'DLT', 'LHM', 'RPNP'


#### Anisotropic Gaussian

Noisy 3D points and lines are modeled by anisotripic Gaussain

**anisotropicGaussianMean = [0.0; 0.0; 0.0]**  mean for points

**anisotropicGaussianVariance = [0.01; 0.01; 0.1]** variance for points

**linecloudMean = [0.0; 0.0; 0.0]** mean for lines

**linecloudVariance = [0.0; 0.0; 0.2]** variance for lines


#### Sampling lines

**numberOfSamples = 10** number of samples per line


#### Distortion Parameters

**kappa = [1;1;0]** parameters for radial distortion [k_1, k_2, k_3]

**p = [0;0]** parameters for tangential distortion [p_1, p_2]


#### Pixel Noise

Pixel Noise is also Gaussian distributed. Set mean and variance by changing

**pixelNoiseMean = [0,0], pixelNoiseVariance = [0,0]**


 









