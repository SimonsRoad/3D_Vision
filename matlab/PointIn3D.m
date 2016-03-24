classdef PointIn3D < handle
    
    % Properties
    
    properties
        
        % Coordinates in world frame
        trueCoordinatesInWorldFrame;
        homogeneousTrueCoordinatesInWorldFrame;
        noisyCoordinatesInWorldFrame;
        homogeneousNoisyCoordinatesInWorldFrame;
        
        % Coordinates in camera frame
        trueCoordinatesInCameraFrame;
        homogeneousTrueCoordinatesInCameraFrame;
        noisyCoordinatesInCameraFrame;
        homogeneousNoisyCoordinatesInCameraFrame;
        
        % Noise parameters in camera frame
        anisotropicGaussianMean;
        anisotropicGaussianVariance;
    end
    
    % Methods
    
    methods
        
        % Default constructor
        function obj = PointIn3D(x,y,z)
            if nargin < 3
                error('PointIn3D has to be initialized with true coordinates')
            else
                obj.trueCoordinatesInWorldFrame = [x;y;z];
                obj.homogeneousTrueCoordinatesInWorldFrame = [x;y;z;1];
                obj.setMean(-1);
                obj.setVariance(-1);
            end
        end
        
        % Set mean
        function setMean(this,mean)
            this.anisotropicGaussianMean = mean;
        end
        
        % Set variance
        function setVariance(this,variance)
            this.anisotropicGaussianVariance = variance;
        end
        
        % Add noise to this point, given the camera pose
        function addNoise(this,T_WC)
            
            % Transform true coordinates to camera frame
            this.homogeneousTrueCoordinatesInCameraFrame = T_WC\this.homogeneousTrueCoordinatesInWorldFrame;
            
            % Initialize noisy coordinates in camera frame
            this.homogeneousNoisyCoordinatesInCameraFrame = this.homogeneousTrueCoordinatesInCameraFrame;
            
            % Add noise
            noiseInCameraX = normrnd(this.anisotropicGaussianMean(1),this.anisotropicGaussianVariance(1));
            this.homogeneousNoisyCoordinatesInCameraFrame(1) = this.homogeneousNoisyCoordinatesInCameraFrame(1)+noiseInCameraX;
            noiseInCameraY = normrnd(this.anisotropicGaussianMean(2),this.anisotropicGaussianVariance(2));
            this.homogeneousNoisyCoordinatesInCameraFrame(2) = this.homogeneousNoisyCoordinatesInCameraFrame(2)+noiseInCameraY;
            noiseInCameraZ = normrnd(this.anisotropicGaussianMean(3),this.anisotropicGaussianVariance(3));
            this.homogeneousNoisyCoordinatesInCameraFrame(3) = this.homogeneousNoisyCoordinatesInCameraFrame(3)+noiseInCameraZ;
            
            % Set noisy coordinates in world frame
            this.homogeneousNoisyCoordinatesInWorldFrame = T_WC*this.homogeneousNoisyCoordinatesInCameraFrame;
            this.noisyCoordinatesInWorldFrame = this.homogeneousNoisyCoordinatesInWorldFrame(1:3);
           
        end
        
        
    end
    
end