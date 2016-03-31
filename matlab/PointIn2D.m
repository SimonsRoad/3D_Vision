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
        noisyPixelCoordinates               %> @param noisyPixelCoordinates Pixel coordinates with pixel noise
        homogeneousNoisyPixelCoordinates    %> @param homogeneousNoisyPixelCoordinates Homogeneous pixel coordinates with pixel noise
        
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
            obj.homogeneousProjectedCoordinates = calibrationMatrix * cameraTruePose * noisyPointIn3D.homogeneousNoisyCoordinatesInWorldFrame;
            
            % consider the scale factor
            obj.homogeneousProjectedCoordinates = obj.homogeneousProjectedCoordinates / obj.homogeneousProjectedCoordinates(3);
            
            % pixel coordinates must be integer, therefore round coordinates
            obj.homogeneousProjectedCoordinates = floor(obj.homogeneousProjectedCoordinates);
            obj.homogeneousNoisyPixelCoordinates = obj.homogeneousProjectedCoordinates;
            
            % euclidean coordinates
            obj.projectedCoordinates = obj.homogeneousProjectedCoordinates(1:2);
            obj.noisyPixelCoordinates = obj.projectedCoordinates;
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
        %> @param noiseType String type of noise
        %> @param variance Variance of gaussian distribution
        %> @param pixelWindowInterval Half interval in pixel of window
        function addPixelNoise(this, noiseType, variance, pixelWindowInterval)
            % discrete uniformly distributed noise
            if strcmp(noiseType,'uniformly')
                % genereate noise in x- and y-direction
                noiseInPixelX = unidrnd(pixelWindowInterval) - pixelWindowInterval;
                noiseInPixelY = unidrnd(pixelWindowInterval) - pixelWindowInterval;
            % discrete binomial distributed noise
            elseif strcmp(noiseType,'binomial')
                % generate noise in x- and y-direction
                noiseInPixelX = this.generateDRV(variance, pixelWindowInterval);
                noiseInPixelY = this.generateDRV(variance, pixelWindowInterval);
            end %if end
            
            % add noise to pixel coordinates
            this.homogeneousNoisyPixelCoordinates(1) = this.homogeneousNoisyPixelCoordinates(1) + noiseInPixelX;
            this.homogeneousNoisyPixelCoordinates(2) = this.homogeneousNoisyPixelCoordinates(2) + noiseInPixelY;
            
            % extract euclidean noisy pixel coordinates
            this.noisyPixelCoordinates = this.homogeneousNoisyPixelCoordinates(1:2);
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
        
        
        %> @brief Helper function
        %>
        %> @param this Pointer to PointIn2D object
        %> @param variance Variance of gaussian distribution
        %> @param pixelWindowInterval Half interval in pixel of window
        function discreteRandomVariable = generateDRV(this, variance, pixelWindowInterval)
            % Generate random uniformly distributed variable
            u = rand();
            
            % Desired probability density function
            pdf = makedist('Normal',0,variance);
            
            % Cumulative distribution function
            F = cdf(pdf,-pixelWindowInterval:pixelWindowInterval);
            
            % Find discrete random variable
            DRV = find(F <= u);
            
            % find last index at which F <= u and u < F
            if (isempty(DRV))
                discreteRandomVariable = - pixelWindowInterval;
            else
                discreteRandomVariable = DRV(end) - pixelWindowInterval;
            end
        end % generateDRV() end
    end % methods end
end % classdef end