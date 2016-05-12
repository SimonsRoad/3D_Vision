classdef PointIn3D < handle
    
    % Properties
    
    properties
        
        X
        Y
        Z
        
        % Coordinates in world frame
        trueCoordinatesInWorldFrame;                %> @param trueCoordinatesInWorldFrame Coordinates of ground truth 3D point in world frame
        homogeneousTrueCoordinatesInWorldFrame;     %> @param homogeneousTrueCoordinatesInWorldFrame Homogeneous coordinates of ground truth 3D point in world frame
        noisyCoordinatesInWorldFrame;               %> @param noisyCoordinatesInWorldFrame Noise-ridden coordinates of 3D point in world frame
        homogeneousNoisyCoordinatesInWorldFrame;    %> @param homogeneousNoisyCoordinatesInWorldFrame Noise ridden homogeneous coordinates in world frame
        
        % Coordinates in camera frame
        trueCoordinatesInCameraFrame;               %> @param trueCoordinatesInWorldFrame Coordinates of ground truth 3D point in ca,era frame
        homogeneousTrueCoordinatesInCameraFrame;    %> @param homogeneousTrueCoordinatesInWorldFrame Homogeneous coordinates of ground truth 3D point in camera frame
        noisyCoordinatesInCameraFrame;              %> @param noisyCoordinatesInWorldFrame Noise-ridden coordinates of 3D point in camera frame
        homogeneousNoisyCoordinatesInCameraFrame;   %> @param homogeneousNoisyCoordinatesInWorldFrame Noise ridden homogeneous coordinates in camera frame
        
        % Noise parameters in camera frame
        anisotropicGaussianMean;                    %> @param Vector of means for the anisotropic Gaussian noise of the 3D point
        anisotropicGaussianVariance;                %> @param Vector of variances for the anisotropic Gaussian noise of the 3D point
    end
    
    % Methods
    
    methods
        
        %> @brief Constructor of class PointIn3D
        %>
        %> @param x Ground truth x-coordinate of the 3D point
        %> @param y Ground truth y-coordinate of the 3D point
        %> @param z Ground truth z-coordinate of the 3D point
        %>
        %> @retval obj Object of type PointIn3D
        function obj = PointIn3D(x,y,z)
            if nargin < 3
                error('PointIn3D has to be initialized with true coordinates')
            else
                obj.X = x;
                obj.Y = y;
                obj.Z = z;
                obj.trueCoordinatesInWorldFrame = [x;y;z];
                obj.homogeneousTrueCoordinatesInWorldFrame = [x;y;z;1];
                obj.setMean(-1);
                obj.setVariance(-1);
            end
        end
        
        function point2D = PointFrom3Dto2D(truePointIn3D, focalLengthMatrix)
             point2D = focalLengthMatrix * truePointIn3D.trueCoordinatesInCameraFrame;
            % Normalize to get homogeneous representation
             point2D =  point2D / point2D(3);
        end
        
        %> @brief Set the means for the noise of this point
        %>
        %> @param this Pointer to this PointIn3D object
        %> @param mean Vector of means, in camera frame

        function setMean(this,mean)
            this.anisotropicGaussianMean = mean;
        end
        

        %> @brief Set the variances for the noise of this point
        %>
        %> @param this Pointer to this PointIn3D object
        %> @param mean Vector of variances, in camera frame

        function setVariance(this,variance)
            this.anisotropicGaussianVariance = variance;
        end
        
        %> @brief
        %>
        %> @param this
        %> @param truePose
        function computeCameraFrameCoordinates(this, truePose)
            this.homogeneousTrueCoordinatesInCameraFrame = [truePose; 0 0 0 1]*this.homogeneousTrueCoordinatesInWorldFrame;
            this.trueCoordinatesInCameraFrame = this.homogeneousTrueCoordinatesInCameraFrame(1:3);
        end
        

        %> @brief Add noise to this point, given the camera pose
        %>
        %> @param this Pointer to this PointIn3D object
        %> @param T_WC Homogeneous transformation (in R^(3x4)) of the camera with respect to the world

        function addNoise(this,T_WC)
            
            % Initialize noisy coordinates in camera frame
            this.homogeneousNoisyCoordinatesInCameraFrame = this.homogeneousTrueCoordinatesInCameraFrame;
            
            % Add noise
            noiseInCameraX = normrnd(this.anisotropicGaussianMean(1),this.anisotropicGaussianVariance(1));
            this.homogeneousNoisyCoordinatesInCameraFrame(1) = this.homogeneousNoisyCoordinatesInCameraFrame(1)+noiseInCameraX;
            noiseInCameraY = normrnd(this.anisotropicGaussianMean(2),this.anisotropicGaussianVariance(2));
            this.homogeneousNoisyCoordinatesInCameraFrame(2) = this.homogeneousNoisyCoordinatesInCameraFrame(2)+noiseInCameraY;
            noiseInCameraZ = normrnd(this.anisotropicGaussianMean(3),this.anisotropicGaussianVariance(3));
            this.homogeneousNoisyCoordinatesInCameraFrame(3) = this.homogeneousNoisyCoordinatesInCameraFrame(3)+noiseInCameraZ;
            this.noisyCoordinatesInCameraFrame = this.homogeneousNoisyCoordinatesInCameraFrame(1:3);
            
            % Set noisy coordinates in world frame
            this.homogeneousNoisyCoordinatesInWorldFrame = [T_WC; 0, 0, 0, 1]\this.homogeneousNoisyCoordinatesInCameraFrame;
            this.noisyCoordinatesInWorldFrame = this.homogeneousNoisyCoordinatesInWorldFrame(1:3);
           
        end
        
        
    end
    
end