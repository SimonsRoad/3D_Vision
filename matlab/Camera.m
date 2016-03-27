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
       mx                           %> @param mx Pixel per unit length in x-direction
       my                           %> @param my Pixel per unit length in y-direction
       xResolution                  %> @param xResolution Pixel resolution in x-direction
       yResolution                  %> @param yResolution Pixel resolution in y-direction
       px                           %> @param px Pixel x-coordinate of principle point
       py                           %> @param py Pixel y-coordinate of principle point
       skew                         %> @param skew Skew paramater of camera sensor
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
       %> @param azimutalAngle Azimutal angle on sphere
       %> @param polarAngle Polar angle on sphere
       %> @param f Focal length of camera
       %> @param px Principle point offset in x-direction of camera frame
       %> @param py Principle point offset in y-direction of camera frame
       %> @param mx Pixel per unit length in x-direction of camera frame
       %> @param my Pixel per unit length in y-direction of camera frame
       %> @param skew Skew paramter of camera
       %>
       %> @retval obj Object of type Camera
       function obj = Camera(radius, azimutalAngle, polarAngle, f, px, py, mx, my, skew)
           % Properties
           obj.f = f;
           obj.px = px;
           obj.py = py;
           obj.mx = mx;
           obj.my = my;
           obj.skew = skew;
           
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
           obj.truePose(1:3,4) = t;
           
           % Declare estimated Pose
           obj.estimatedPose = eye(4);
           
           % Calculate camera calibration matrix
           obj.calculateCalibrationMatrix();
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
           estimatedCam = plotCamera('Location',trueRotation' *estimatedTranslation,'Orientation',estimatedRotation,'Size',0.1,'Color',[1 0 0]);
       end % visualizeCamera() end
       
       
       %> @brief Calculates the camera calibration matrix
       %>
       %> @param this Pointer to object
       function calculateCalibrationMatrix(this)
           % Fill in the calibration matrix
           this.K = [this.f * this.mx, this.skew, this.px * this.mx;
               0, this.f * this.my, this.py * this.my;
               0, 0, 1];
       end % calculateCalibrationMatrix() end
       
       
       %> @brief Returns camera calibration matrix
       %> 
       %> @param this Pointer to Camera object
       %>
       %> @retval K calibration matrix this
       function K = getCalibrationMatrix(this)
           K = this.K;
       end % getCalibrationMatrix() end
       
       
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