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
       truePose                     %> @param truePose True pose of camera
       estimatedPose                %> @param estimatedPose Estimated pose of camera
       f                            %> @param f focal length
       kx                           %> @param kx Pixel per unit length in x-direction
       ky                           %> @param ky Pixel per unit length in y-direction
       xResolution                  %> @param xResolution Pixel resolution in x-direction
       yResolution                  %> @param yResolution Pixel resolution in y-direction
       x0                           %> @param x0 Pixel x-coordinate of principle point
       y0                           %> @param y0 Pixel y-coordinate of principle point
       K                            %> @param K Calibration matrix
       distortionModel              %> @param distortionModel string Distortion model
       perspectiveNPointAlgorithm   %> @param perspectiveNPointAlgorithm string of algorithm used for pose estimation
       
       pointCloud3D@Pointcloud3D
       %pointCloud2D@PointIn2D
   end % Properties end
   
   methods
       %> @brief Constructor of class Camera
       %> 
       %> @param radius Radius of sphere the camera is on
       %> @param polarAngleMax Maximal polar angle on sphere
       %>
       %> @retval obj Object of type Camera
       function obj = Camera(radius, polarAngleMax)
           % Generate random azimutalAngle and polarAngle
           azimutalAngle = 2*pi*rand();
           polarAngle = polarAngleMax*rand();
           
           % Camera translation vector w.r.t. camera frame
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
           obj.truePose = eye(4);
           obj.truePose(1:3,1:3) = R_CI;
           obj.truePose(1:3,4) = t;%-(R_CI')*I_C;
           
           % Declare estimated Pose
           obj.estimatedPose = eye(4);   
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
           
           % plot the true pose
           figure(figureHandle)
           trueCam = plotCamera('Location',trueRotation' *trueTranslation,'Orientation',trueRotation,'Size',0.1,'Color',[0 0 1]);
           axis equal
           
           % plot the estimated pose
           hold on
           estimatedCam = plotCamera('Location',trueRotation' *estimatedTranslation,'Orientation',estimatedRotation,'Size',0.1,'Color',[1 0 0]);
       end % visualizeCamera() end
       
       
       %> @brief Returns kalibration matrix of the camera
       %> 
       %> @param this Pointer to Camera object
       %>
       %> @retval K Kalibration matrix this
       function K = getKalibrationMatrix(this)
           K = this.K;
       end % getKalibrationMatrix() end
       
       
       %> @brief getPose() returns true and estimated pose of a Camera object
       %>
       %> @param this Pointer to Camera object
       %>
       %> @retval truePose True pose of Camera object
       %> @retval estimatedPose Estimated pose of Camera object
       function [truePose, estimatedPose] = getPose(this)
          truePose = this.truePose;
          estimatedPose = this.estimatedPose;
       end % getPose end
   end % Methods end
end % Classdef end