% =========================================================================
%> @brief Class PointIn2D stores projected coordinates of a corresponding PointIn3D and adds pixel noise to itself
%>
%>
%>
% =========================================================================
classdef PointIn2D < handle
    properties
        % Coordinates in camera frame
        projectedCoordinates                %> @param projectedCoordinates Coordinates of the 3D to 2D projection
        homogeneousProjectedCoordinates     %> @param homogeneousProjectedCoordinates Homogeneous coordinates of the 3D to 2D projection
        noisyCoordinates                    %> @param noisyCoordinates Coordinates with pixel noise
        homogeneousNoisyCoordinates         %> @param homogeneousNoisyCoordinates Homogeneous coordinates with pixel noise
        
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
        function obj = PointIn2D(noisyPointIn3D, calibrationMatrix, cameraTruePose)
            % project noisy 3D point with x = K*[R|t]*X
            obj.homogeneousProjectedCoordinates = calibrationMatrix * cameraTruePose(1:3,:) * noisyPointIn3D.homogeneousNoisyCoordinatesInWorldFrame;
            
            % euclidean coordinates
            obj.projectedCoordinates = obj.homogeneousProjectedCoordinates(1:3);
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
        
        %> @brief
        %>
        %> @param
        %> @param
        %> @param
        function point2Ddistorted = addDistortion(Point2D, k, Camera)
           K = Camera.calculateCalibrationMatrix();
           centerOfDistortion = [K(1,3); K(2,3)];
           radius = sqrt((Point2D(1,1) - centerOfDistortion(1,1))^2 + (Point2D(2,1) - centerOfDistortion(2,1))^2);
           point2Ddistorted = (1 + k(1) * radius^2 + k(2) * radius^4 + k(3) * radius^6) * Point2D;
        end % addDistortion() end
    end % methods end
end % classdef end