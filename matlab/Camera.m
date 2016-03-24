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
       pointCloud2D@Pointcloud2D
   end % Properties end
   
   methods
       %> @brief Constructor of class Camera
       %> 
       %> @param radius Radius of sphere the camera is on
       %> @param polarAngleMax Maximal polar angle on sphere
       %>
       %> @retval obj
       function obj = Camera(radius, polarAngleMax)
           % Generate random azimutalAngle and polarAngle
           azimutalAngle = 2*pi*rand();
           polarAngle = polarAngleMax*rand();
           
           % Compute translation t of camera
           t = [radius*cos(azimutalAngle)*sin(polarAngle);
               radius*sin(azimutalAngle)*sin(polarAngle);
               radius*cos(polarAngle)];
           
           % Compute helper rotation matrix
           R_BI = [cos(azimutalAngle), sin(azimutalAngle), 0;
               -sin(azimutalAngle), cos(azimutalAngle), 0;
               0, 0, 1];
           R_KB = [cos(pi/2-polarAngle), 0, sin(pi/2-polarAngle);
               0, 1, 0;
               -sin(pi/2-polarAngle), 0, cos(pi/2-polarAngle)];
           R_CK = [1, 0, 0;
               0, cos(pi/2), sin(pi/2);
               0, -sin(pi/2), cos(pi/2)];
           
           % Compute the camera rotation matrix
           R_CI = (R_CK*R_KB*R_BI);
           R_IC = R_CI';
           
           % Fill in the camera truePose 
           obj.truePose = eye(4);
           obj.truePose(1:3,1:3) = R_IC;
           obj.truePose(1:3,4) = t;
           
       end % Camera end
       
       
       %> @brief Returns the plot handle of camera visualization
       %>
       %> @param figureHandle Figure number
       %> @param this Pointer of Camera object
       %>
       %> @retval plotHandle Handle to plot3 of visualization
       function plotHandle = visualizeCamera(figureHandle, this)
           % get true translation and rotation from truePose
           trueTranslation = this.truePose(1:3,4);
           trueRotation = this.truePose(1:3,1:3);
           
           % get estimated translation and rotation from estimatedPose
           estimatedTranslation = this.estimatedPose(1:3,4);
           estimatedRotation = this.estimatedPose(1:3,1:3);
       end
       
       
       %> @brief Returns kalibration matrix of the camera
       %> 
       %> @param this Pointer to Camera object
       %>
       %> @retval K Kalibration matrix this
       function K = getKalibrationMatrix(this)
           K = this.K;
       end
       
       
       %> @brief getTruePose() returns true pose of a Camera object
       %>
       %> @param obj
       %>
       %> @retval
       function P = getTruePose(this)
          P = this.truePose; 
       end % getTrueCameraMatrix end
       
       
       %> @brief getEstimatedPose() returns true pose of a Camera object
       %>
       %> @param obj
       %>
       %> @retval
       function P = getEstimatedPose(this)
          P = this.estimatedPose; 
       end  % getEstimatedCameraMatrix end
       
   end % Methods end
end % Classdef end