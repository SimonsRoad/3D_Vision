classdef PnPAlgorithm < handle
    
    % Properties
    
    properties
        
        numberOfPoints;             %> @param numberOfPoints Number of points
        algorithm;                  %> @param algorithm Name of the algorithm for pose estimation
        matrixOfNoisy3DPoints
        matrixOfNoisy2DPoints
        matrixOfHomogeneousNoisy3DPoints       %> @param matrixOfNoisy3DPoints The noisy 3D points *in world frame* in matrix form
        matrixOfHomogeneousNoisy2DPoints       %> @param matrixOfNoisy2DPoints The noisy 2D points *in camera frame *in matrix form

        
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
                obj.matrixOfNoisy3DPoints = zeros(3,obj.numberOfPoints);
                obj.matrixOfNoisy2DPoints = zeros(2,obj.numberOfPoints);
                obj.matrixOfHomogeneousNoisy3DPoints = zeros(4,obj.numberOfPoints);
                obj.matrixOfHomogeneousNoisy2DPoints = zeros(3,obj.numberOfPoints);
                for i = 1:obj.numberOfPoints
                    obj.matrixOfNoisy3DPoints(:,i) = pointCloud3D_.pointsIn3D(i).noisyCoordinatesInWorldFrame;
                    obj.matrixOfNoisy3DPoints(1,i) = -obj.matrixOfNoisy3DPoints(1,i);
                    obj.matrixOfNoisy3DPoints(2,i) = -obj.matrixOfNoisy3DPoints(2,i);
                    obj.matrixOfNoisy2DPoints(:,i) = pointCloud2D_.pointsIn2D(i).distortedNoisyCoordinatesInCameraFrame;
                    %obj.matrixOfNoisy2DPoints(1,i) = -obj.matrixOfNoisy2DPoints(1,i);
                    obj.matrixOfHomogeneousNoisy3DPoints(:,i) = pointCloud3D_.pointsIn3D(i).homogeneousNoisyCoordinatesInWorldFrame;
                    obj.matrixOfHomogeneousNoisy2DPoints(:,i) = pointCloud2D_.pointsIn2D(i).homogeneousDistortedNoisyCoordinatesInCameraFrame;
                end
            end
        end
        
        %> @brief
        %>
        function [R,t] = estimatePose(this, intrinsicMatrix)
            if strcmp(this.algorithm, 'EPNP')
                addpath('EPnP')
                disp('Using EPnP for pose estimation.')
                [R,t,~,~] = efficient_pnp(this.matrixOfHomogeneousNoisy3DPoints.',this.matrixOfHomogeneousNoisy2DPoints.',intrinsicMatrix);
                t = -t;
            end
            if strcmp(this.algorithm, 'EPNP-Gauss')
                addpath('EPnP')
                disp('Using EPnP-Gauss for pose estimation.')
                error('EPnP-Gauss does not seem to work properly yet. Choose another pnp algorithm, or fix code :)')
                [R,t,~,~,~] = efficient_pnp_gauss(this.matrixOfHomogeneousNoisy3DPoints.',this.matrixOfHomogeneousNoisy2DPoints.',intrinsicMatrix);
            end
            if strcmp(this.algorithm, 'DLT')
                addpath('DLT')
                disp('Using DLT for pose estimation.')
                [R,t] = DLT(this.matrixOfNoisy3DPoints,this.matrixOfNoisy2DPoints/intrinsicMatrix(1,1));
                t = -t;
            end
            if strcmp(this.algorithm, 'LHM')
                addpath('LHM')
                disp('Using LHM for pose estimation.')
                [R,t] = LHM(this.matrixOfNoisy3DPoints,this.matrixOfNoisy2DPoints/intrinsicMatrix(1,1));
                t = -t;
            end
            if strcmp(this.algorithm, 'RPNP')
                addpath('RPNP')
                disp('Using RPNP for pose estimation.')
                [R,t] = RPnP(this.matrixOfNoisy3DPoints,this.matrixOfNoisy2DPoints/intrinsicMatrix(1,1));
                t = -t;
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