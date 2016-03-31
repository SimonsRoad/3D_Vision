% =========================================================================
%> @brief Class Camera sets a camera in space and computes estimated camera pose from 3D-2D point correspondences
%>
%> 
%>
%> 
%>
% =========================================================================
classdef Camera < handle
   properties
       % Camera pose
       truePose                     %> @param truePose True pose of camera
       estimatedPose                %> @param estimatedPose Estimated pose of camera
       
       % Camera parameters
       f                            %> @param f Focal length
       kx                           %> @param kx Pixel per unit length in x-direction
       ky                           %> @param ky Pixel per unit length in y-direction
       xResolution                  %> @param xResolution Pixel resolution in x-direction
       yResolution                  %> @param yResolution Pixel resolution in y-direction
       x0                           %> @param x0 Pixel x-coordinate of principle point
       y0                           %> @param y0 Pixel y-coordinate of principle point
       skew                         %> @param skew Skew paramater of camera sensor
       imagetoPixelCoordinatesTrafo %> @param ImagetoPixelCoordinatesTrafo Matrix from u,y to x,y coordinates
       focalLenghtMatrix            %> @param focallengthMatrix Matrix [f 0 0; 0 f 0; 0 0 1]
       K                            %> @param K Calibration matrix
       distortionModel              %> @param distortionModel String Distortion model
       kappa                        %> @param kappa Radial distortion parameters
       p                            %> @param p tangential distortion parameters
       
       % Pose estimation algorithm
       perspectiveNPointAlgorithm   %> @param perspectiveNPointAlgorithm String of algorithm used for pose estimation
       
       % Point clouds
       pointCloud3D@Pointcloud3D    %> @param pointCloud3D Member of type Pointcloud3D
       pointCloud2D@Pointcloud2D    %> @param pointCloud2D Member of type Pointcloud2D
       
       % PnP Algorithm
       pnpAlgorithm@PnPAlgorithm;   %> @param pnpAlgorithm Perspective N Point algorithm
   end % Properties end
   
   methods
       %> @brief Constructor of class Camera
       %> 
       %> @param radius Radius of sphere the camera is on
       %> @param azimutalAngle Azimutal angle on sphere
       %> @param polarAngle Polar angle on sphere
       %> @param f Focal length of camera
       %> @param x0 Principle point offset in x-direction of camera frame
       %> @param y0 Principle point offset in y-direction of camera frame
       %> @param kx Pixel per unit length in x-direction of camera frame
       %> @param ky Pixel per unit length in y-direction of camera frame
       %> @param skew Skew paramter of camera
       %>
       %> @retval obj Object of type Camera
       function obj = Camera(radius, azimutalAngle, polarAngle, f, x0, y0, kx, ky, skew, kappa, p)
           % Properties
           obj.f = f;
           obj.x0 = x0;
           obj.y0 = y0;
           obj.kx = kx;
           obj.ky = ky;
           obj.skew = skew;
           
           % Camera translation vector w.r.t. camera frame t = R_CI * C_I
           t = [0; 0; -radius];
           
           % Compute helper rotation matrix
           R_BI = [cos(azimutalAngle), sin(azimutalAngle), 0; % z-rotation at phi
               -sin(azimutalAngle), cos(azimutalAngle), 0;
               0, 0, 1];
           R_KB = [cos(-pi/2+polarAngle), 0, -sin(-pi/2+polarAngle); % y-rotation at -phi/2+theta
               0, 1, 0;
               sin(-pi/2+polarAngle), 0, cos(-pi/2+polarAngle)];
           R_CK = [0, 1, 0; % camera z-axis points towards origin
               0, 0, -1;
               -1, 0, 0];
           
           % Compute the camera rotation matrix
           R_CI = (R_CK*R_KB*R_BI);
           
           % Fill in the camera truePose 
           obj.truePose = zeros(3,4);
           obj.truePose(1:3,1:3) = R_CI;
           obj.truePose(1:3,4) = t;
           
           % Declare estimated Pose
           obj.estimatedPose(1:3,1:3) = eye(3);
           obj.estimatedPose(1:3,4) = zeros(1,3);
           
           % Calculate Transformation Matrix from image to pixel coordinates
           obj.calculateUVtoPixelMatrix();
           % Calculate focallength Matrix [f 0 0; 0 f 0; 0 0 1]
           obj.calculateFocallengthMatrix
           
           % Calculate camera calibration matrix
           obj.calculateCalibrationMatrix();
           
           % Set distortion parameters
           obj.kappa = kappa;   % radial distortion
           obj.p = p;           % tangential distortion

       end % Camera() end
       
       
       %> @brief Visualizes the camera
       %>
       %> @param this Pointer of Camera object
       %> @param figureHandle Figure number
       %>
       %> @retval trueCam Handle to true camera pose plot
       %> @retval estimatedCam Handle to estimated camera pose plot
       function [trueCam, estimatedCam] = visualizeCamera(this, figureHandle)
           % Get true translation and rotation from truePose
           trueTranslation = this.truePose(1:3,4);
           trueRotation = this.truePose(1:3,1:3);
           
           % Get estimated translation and rotation from estimatedPose
           estimatedTranslation = this.estimatedPose(1:3,4);
           estimatedRotation = this.estimatedPose(1:3,1:3);
           
           % Plot the true pose
           figure(figureHandle)
           trueCam = plotCamera('Location',trueRotation' *trueTranslation,'Orientation',trueRotation,'Size',0.1,'Color',[0 0 1]);
           axis equal
           
           % Plot the estimated pose
           hold on
           estimatedCam = plotCamera('Location',estimatedRotation' *estimatedTranslation,'Orientation',estimatedRotation,'Size',0.1,'Color',[1 0 0]);
       end % visualizeCamera() end
       
       
       %> @brief Projects a pointcloud in 3D into a pointcloud in 2D
       %>
       %> @param this Pointer to Camera object
       function projectFrom3DTo2D(this)
           % Project noisy 3D points to 2D pixel space

           this.pointCloud2D = Pointcloud2D(this.pointCloud3D, this.K, this.imagetoPixelCoordinatesTrafo, this.focalLenghtMatrix, this.truePose, this.kappa, this.p);

       end % projectFrom3DTo2D() end
       
       
       %> @brief Plot projected and noisy 2D points
       %>
       %> @param this Pointer to object
       %> @param figureHandle Handle to figure number
       function plot2DPoints(this, figureHandle)
           % Open figure
           figure(figureHandle)
           
           % Plot projected 2D points
           this.pointCloud2D.plotProjectedPoints(figureHandle);
           
           % Plot noisy 2D points
           hold on
%            this.pointCloud2D.plotNoisyPoints(figureHandle);
           hold off
       end % plot2DPoints() end
       
       %> @brief Plot distorted 2D points in image coordinates (u,v)
       %>
       %> @param this Pointer to object
       %> @param figureHandle Handle to figure number
       function plotDistortedImage2DPoints(this, figureHandle)
           % Open figure
           figure(figureHandle)
           
           % Plot distortoted 2D points
           this.pointCloud2D.plotDistortedImagePoints(figureHandle);
           
       end % plot2DPoints() end
       
       %> @brief Plot distorted 2D points in pixel coordinates
       %>
       %> @param this Pointer to object
       %> @param figureHandle Handle to figure number
       function plotDistortedPixel2DPoints(this, figureHandle)
           % Open figure
           figure(figureHandle)
           
           % Plot distortoted 2D points
           this.pointCloud2D.plotDistortedPixelPoints(figureHandle);
           
       end % plot2DPoints() end
       
       
       %> @brief Calculates the transformation Matrix from UV to XY (pixel coordinates) [kx s x0; 0 ky y0; 0 0 1]
       %> 
       %> @param this Pointer to object
       function calculateUVtoPixelMatrix(this)
           this.imagetoPixelCoordinatesTrafo = [this.kx, this.skew, this.x0;
               0, this.ky, this.y0;
               0, 0, 1];
       end
       
       %> @brief Calculates the focallength matrix [f 0 0; 0 f 0; 0 0 1]
       %> 
       %> @param this Pointer to object
       function calculateFocallengthMatrix(this)
           this.focalLenghtMatrix = [this.f, 0, 0; 0, this.f, 0; 0, 0, 1];
       end
       
       %> @brief Calculates the camera calibration matrix
       %>
       %> @param this Pointer to object
       function calculateCalibrationMatrix(this)
           % Fill in the calibration matrix
           this.K = this.imagetoPixelCoordinatesTrafo * this.focalLenghtMatrix;
       end % calculateCalibrationMatrix() end
       
       %> @brief Estimate the camera pose with a pnp algorithm
       function estimatePose(this)
           [R,t] = this.pnpAlgorithm.estimatePose([this.f 0 0; 0 this.f 0; 0 0 1]);
           this.estimatedPose(1:3,1:3) = R;
           this.estimatedPose(1:3,4) = t;
       end
       
       %> @brief Calculate the error in the pose estimation
       %>
       %> @retval errorInTranslation This is the error in camera translation in percent
       %> @retval errorInOrientation This is the error in orientation. The error is calculated as the sum of the acos of the scalar products of the unit vectors of the coordinate frames (todo: come up with a better way to describe this)
       function [errorInTranslation, errorInOrientation] = computePoseError(this)
           xTrue = this.truePose(:,1);
           yTrue = this.truePose(:,2);
           zTrue = this.truePose(:,3);
           xEstimated = this.estimatedPose(:,1);
           yEstimated = this.estimatedPose(:,2);
           zEstimated = this.estimatedPose(:,3);
           scalarProducts = [xTrue'*xEstimated yTrue'*yEstimated zTrue'*zEstimated];
           % If *True = *Estimated their scalar product should be 1 as they
           % are normalized vectors. (The acos of 1 is 0)
           errorInOrientation = sum(abs(acos(scalarProducts)))*180/pi;
           trueTranslation = this.truePose(:,4);
           estimatedTranslation = this.estimatedPose(:,4);
           errorInTranslation = norm(trueTranslation-estimatedTranslation)/norm(trueTranslation)*100;
       end
       
       %> @brief Returns camera calibration matrix
       %> 
       %> @param this Pointer to Camera object
       %>
       %> @retval K calibration matrix this
       function K = getCalibrationMatrix(this)
           K = this.K;
       end % getCalibrationMatrix() end
       
       %> @brief Sets the PnP Algorithm
       %>
       %> @param algorithm Name of the PnP Algorithm
       function setPnPAlgorithm(this,algorithm_)
           this.pnpAlgorithm = PnPAlgorithm(this.pointCloud3D, this.pointCloud2D, algorithm_);
       end
       
       %> @brief getPose() returns true and estimated pose of a Camera object
       %>
       %> @param this Pointer to Camera object
       %>
       %> @retval truePose True pose of Camera object
       %> @retval estimatedPose Estimated pose of Camera object
       function [truePose, estimatedPose] = getPose(this)
          truePose = this.truePose;
          estimatedPose = this.estimatedPose;
       end % getPose() end
       
       
   end % methods end
end % classdef end