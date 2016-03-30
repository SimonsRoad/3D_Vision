classdef PnPAlgorithm < handle
    
    % Properties
    
    properties
        
        numberOfPoints;             %> @param numberOfPoints Number of points
        algorithm;                  %> @param algorithm Name of the algorithm for pose estimation
        matrixOfNoisy3DPoints       %> @param matrixOfNoisy3DPoints The noisy 3D points *in world frame* in matrix form
        matrixOfNoisy2DPoints       %> @param matrixOfNoisy2DPoints The noisy 2D points *in camera frame *in matrix form

        
    end
    
    % Methods
    
    methods
        

        %> @brief Constructor of PnPAlgorithm class
        %>
        %> @param pointCloud3D_ 3D Pointcloud
        %> @param pointCloud2D_ 2D Pointcloud
        %> @param algorithm_ Name of the algorithm
        %>
        %> @retval Object of type PnPAlgorithm
        function obj = PnPAlgorithm(pointCloud3D_,pointCloud2D_,algorithm_)
            if nargin < 3
                error('PnP Algorithm has to be initialized with three arguments: 3D Pointcloud, 2D Pointcloud, and an Algorithm Name')
                return
            else
                % Set local variables
                obj.algorithm = algorithm_;
                obj.numberOfPoints = pointCloud3D_.getNumberOfPoints();
                
                % Convert the pointclouds to matrices, as the pnp
                % algorithms require them in this form
                obj.matrixOfNoisy3DPoints = zeros(4,obj.numberOfPoints);
                obj.matrixOfNoisy2DPoints = zeros(3,obj.numberOfPoints);
                for i = 1:obj.numberOfPoints
                    obj.matrixOfNoisy3DPoints(:,i) = pointCloud3D_.pointsIn3D(i).homogeneousNoisyCoordinatesInWorldFrame;
                    obj.matrixOfNoisy2DPoints(:,i) = pointCloud2D_.pointsIn2D(i).homogeneousNoisyCoordinatesInCameraFrame;
                end
            end
        end
        
        %> @brief
        %>
        function [R,t] = estimatePose(this, intrinsicMatrix)
            if strcmp(this.algorithm, 'EPNP')
                addpath('EPnP')
                [R,t,~,~] = efficient_pnp(this.matrixOfNoisy3DPoints',this.matrixOfNoisy2DPoints',intrinsicMatrix);
            end
        end
        
        %> @brief
        %>
        %> @param this
        %> @param T_WC
        function addNoiseToAllPoints(this,T_WC)
            for i = 1:this.numberOfPoints
                this.pointsIn3D(i).addNoise(T_WC);
            end
        end
        

        
    end
end