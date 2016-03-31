% =========================================================================
%> @brief Class PointIn2D stores projected coordinates of a corresponding PointIn3D and adds pixel noise to itself
%>
%>
%>
% =========================================================================
classdef PointIn2D < handle
    properties

        % Coordinates in camera frame
        projectedCoordinates                                    %> @param projectedCoordinates Coordinates of the 3D to 2D projection
        homogeneousProjectedCoordinates                         %> @param homogeneousProjectedCoordinates Homogeneous coordinates of the 3D to 2D projection
        noisyCoordinates                                        %> @param noisyCoordinates Coordinates with pixel noise
        homogeneousNoisyCoordinates                             %> @param homogeneousNoisyCoordinates Homogeneous coordinates with pixel noise
        
        noisyCoordinatesInCameraFrame                           %> @param noisyCoordinatesInCameraFrame u-v coordinates
        homogeneousNoisyCoordinatesInCameraFrame                %> @param noisyCoordinatesInCameraFrame Homogeneous u-v coordinates
        
        
        distortedNoisyCoordinatesInCameraFrame                  %> @param distortedNoisyCoordinatesInCameraFrame Distorted u-v coordinates
        homogeneousDistortedNoisyCoordinatesInCameraFrame       %> @param distortedNoisyCoordinatesInCameraFrame Homogeneous distorted u-v coordinates
        
        homogeneousDistortedPixelCoordinates                    %> @param homogeneousDistortedPixelCoordinates Homogeneous distorted pixel coordinates
        distortedPixelCoordinates                               %> @param distortedPixelCoordinates Distorted pixel coordinates (x,y)

        
        % Noise in pixel space
        mean                        %> @param Vector of means for the anisotropic Gaussian noise of the 2D point
        variance                    %> @param Vector of variances for the anisotropic Gaussian noise of the 2D point
    end % properties end
    
    methods
        %> @brief Constructor calculates directly coordinates of a 2D point from a noisy 3D correspondence
        %>
        %> Formula: x = K*[R|t]*X
        %> x := homogeneous coordinates of a 2D point
        %> X := homogeneous coordinates of a 3D point
        %> K := camera calibration matrix
        %> R := rotation matrix from world into camera frame
        %> t := translation of camera in camera frame
        %> Note: [R|t] is a 3x4 matrix, that is cameratruePose(1:3,:)
        %>
        %> @param noisyPointIn3D A noisy point in 3D
        %> @param calibrationMatrix Camera calibration matrix
        %> @param cameraTruePose Ground truth of camera
        %>
        %> @retval obj An object of class PointIn2D

        % First, do the coordinates in pixel space

        function obj = PointIn2D(noisyPointIn3D, calibrationMatrix, imageToPixelMatrix, focalLengthMatrix, cameraTruePose, kappa, p)
            % project noisy 3D point with x = K*[R|t]*X
            obj.homogeneousProjectedCoordinates = calibrationMatrix * cameraTruePose * noisyPointIn3D.homogeneousNoisyCoordinatesInWorldFrame;
            % consider the scale factor
            obj.homogeneousProjectedCoordinates = obj.homogeneousProjectedCoordinates / obj.homogeneousProjectedCoordinates(3);
            
            % Now for the coordinates in camera frame
            % Transform noisy 3D point with x = K_tilde*[R|t]*X, K_tilde is
            % the calibration matrix but without the conversion to pixel
            % space
            % project noisy 3D point in u,v coordinates
            obj.homogeneousNoisyCoordinatesInCameraFrame = focalLengthMatrix * cameraTruePose * noisyPointIn3D.homogeneousNoisyCoordinatesInWorldFrame;
            % normalization
            obj.homogeneousNoisyCoordinatesInCameraFrame = obj.homogeneousNoisyCoordinatesInCameraFrame / obj.homogeneousNoisyCoordinatesInCameraFrame(3);
            
            % euclidean coordinates
            obj.projectedCoordinates = obj.homogeneousProjectedCoordinates(1:2);
            obj.noisyCoordinatesInCameraFrame = obj.homogeneousNoisyCoordinatesInCameraFrame(1:2);
            
            % add distortion (radial and tangetial) to noisyCoordinatesInCameraFrame
            obj.addDistortion(kappa, p); % calculates obj.distortedNoisyCoordinatesInCameraFrame
            obj.homogeneousDistortedNoisyCoordinatesInCameraFrame = [obj.distortedNoisyCoordinatesInCameraFrame(1); obj.distortedNoisyCoordinatesInCameraFrame(2); 1];
            obj.homogeneousDistortedPixelCoordinates = imageToPixelMatrix * [obj.distortedNoisyCoordinatesInCameraFrame(1); obj.distortedNoisyCoordinatesInCameraFrame(2); 1];
            obj.distortedPixelCoordinates = obj.homogeneousDistortedPixelCoordinates(1:2);
            
        end % Constructor end
        
        
        %> @brief Returns coordinates of this object of PointIn2D
        %>
        %> @param this Pointer to object
        %>
        %> @retval projCoord Projected coordinates of 2D point
        %> @retval homProjCoord Homogeneous coordinates of 2D point
        %> @retval noisyCoord Noisy coordinates of 2D point
        %> @retval homNoisyCoord Homogeneous coordinates of noisy 2D point
        function [projCoord, homProjCoord, noisyCoord, homNoisyCoord] = getCoordinates(this)
            projCoord = this.projectedCoordinates;
            homProjCoord = this.homogeneousProjectedCoordinates;
            noisyCoord = this.noisyCoordinates;
            homNoisyCoord = this.homogeneousNoisyCoordinates;
        end % getCoordinates() end
        
        
        %> @brief Set the means for the noise of this point
        %>
        %> @param this Pointer to object
        %> @param mean Vector of means, in camera frame
        function setMean(this, mean)
            this.mean = mean;
        end % setMean() end
        

        %> @brief Set the variances for the noise of this point
        %>
        %> @param this Pointer to object
        %> @param mean Vector of variances
        function setVariance(this, variance)
            this.variance = variance;
        end % setVariance() end
        
        %> @brief Add pixel noise to this point
        %>
        %> @param this Pointer to object
        function addPixelNoise(this)
            %----TODO----
            %
            %----TODO----
        end % addPixelNoise() end
        
        %> @brief Add distortion to u,v coordinates
        %>
        %> @param this Pointer to object
        %> @param kappa Radial distortion parameters 3-dimensional vector
        %> @param p Tangential distortion parameters 2-dimensional vector
        function addDistortion(this, kappa, p)
      
           uvCoordinates = this.noisyCoordinatesInCameraFrame;
           %centerOfDistortion = [0;0];
           radiusSquared = uvCoordinates(1)^2 + uvCoordinates(2)^2;
           this.distortedNoisyCoordinatesInCameraFrame(1) = (1 + kappa(1) * radiusSquared + kappa(2) * radiusSquared^2 + kappa(3) * radiusSquared^3) * uvCoordinates(1) ...
               + 2 * p(1) * uvCoordinates(1) * uvCoordinates(2) + p(2) * (radiusSquared + 2 * uvCoordinates(1)^2);
           this.distortedNoisyCoordinatesInCameraFrame(2) = (1 + kappa(1) * radiusSquared + kappa(2) * radiusSquared^2 + kappa(3) * radiusSquared^3) * uvCoordinates(2) ...
               + 2 * p(2) * uvCoordinates(1) * uvCoordinates(2) + p(1) * (radiusSquared + 2 * uvCoordinates(2)^2);
           
        end % addDistortion() end
        
    end % methods end
end % classdef end