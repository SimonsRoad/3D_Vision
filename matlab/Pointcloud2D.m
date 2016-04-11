% =========================================================================
%> @brief
%>
%>
%>
% =========================================================================
classdef Pointcloud2D < handle
    properties
        pointsIn2D@PointIn2D                %> @param pointsIn2D Vector of type PointIn2D
        numberOfPoints                      %> @param numberOfPoints Number of points stored in vector pointsIn2D
    end
    
    methods
        %> @brief Constructor projects a 3D pointcloud to 2D pointcloud based on camera ground truth pose
        %>
        %> @param pointCloud3D 3D pointcloud
        %> @param focalLengthMatrix Transformation matrix to the focal plane
        %> @param cameraTruePose Ground truth pose of a camera
        %>
        %> @retval obj array with points in 2D


        %function obj = Pointcloud2D(pointCloud3D, calibrationMatrix, focalLengthMatrix, cameraTruePose)


        function obj = Pointcloud2D(pointCloud3D, focalLengthMatrix)

            % Get number of points in pointCloud3D
            obj.numberOfPoints = pointCloud3D.getNumberOfPoints();
            
            % Construct all the 2D correspondences with a loop
            for i = 1:obj.numberOfPoints

                %obj.pointsIn2D(i) = PointIn2D(pointCloud3D.pointsIn3D(i), calibrationMatrix, focalLengthMatrix, cameraTruePose);

                obj.pointsIn2D(i) = PointIn2D(pointCloud3D.pointsIn3D(i), focalLengthMatrix);

            end % for loop end
        end % Constructor end
        
        % 1. add distortion
        %> @brief
        %>
        %> @param this Pointer to Pointcloud2D object
        %> @param kappa radial distortion paramaeter dim(kappa) = 3
        %> @param p tangential distortion parameter dim(p) = 2
        function addDistortion(this, kappa, p)
            % for every 2D point
           for i = 1:this.numberOfPoints
               this.pointsIn2D(i).addDistortion(kappa,p);
           end
        end % addDistortion() end
        
        % 2. calculate homogeneous distorted pixel points
        %> @brief
        %>
        %> @param this Pointer to Pointcloud2D object
        %> @param imageToPixelMatrix [this.kx, this.skew, this.x0; 0, this.ky, this.y0; 0, 0, 1];
        function calculateHomoegenousDistortedPixelPoints(this, imageToPixelMatrix)
            % for every 2D point
           for i = 1:this.numberOfPoints
               this.pointsIn2D(i).calculateHomoegeneousDistortedPixelCoordinates(imageToPixelMatrix);
           end
        end % calculateHomoegenousDistortedPixelPoints() end
        
        % 3. calculate euclidean distorted pixel points from homoegenous distorted pixel points
        %> @brief
        %>
        %> @param this Pointer to Pointcloud2D object
        function setDistortedPixelCoordinatesFromHomogeneousCoordinates(this)
             % for every 2D point
           for i = 1:this.numberOfPoints
               this.pointsIn2D(i).setDistortedPixelCoordinatesFromHomogeneousCoordinates();
           end
        end % setDistortedPixelCoordinatesFromHomogeneousCoordinates() end
        
        % 4. add pixel noise
        %> @brief Adds pixel noise to pixel representation of the 2d projected point
        %>
        %> @param this Pointer to Pointcloud2D object
        %> @param noiseType String type of noise
        %> @param mean Mean of gaussian distribution in x- and y-direction
        %> @param variance Variance of gaussian distribution in x- and y-direction
        function addPixelNoise(this, noiseType, mean, variance)
            % for every 2D point
            for i = 1:this.numberOfPoints
                this.pointsIn2D(i).addPixelNoise(noiseType, mean, variance);
            end
        end % addPixelNoise() end
        
        % 5. back projection
        %> @brief
        %>
        %> @param this Pointer to Pointcloud2D object
        %> @param calibrationMatrix calibration matrix
        function transformFromPixelToImage(this, calibrationMatrix)
            % for every 2D point
            for i = 1:this.numberOfPoints
                this.pointsIn2D(i).transformFromPixelToImage(calibrationMatrix);
            end
        end % transformFromPixelToImage() end
        
        % 6. undistortion
       %%%%%% has to be done
        
        %> @brief Plots the projected 2D points
        %>
        %> @param this Pointer to object
        %> @param figureHandle Handle to figure
        function plotPixelPoints(this)
            % Declare X and Y vectors
            X = zeros(1,this.numberOfPoints);
            Y = zeros(1,this.numberOfPoints);
            
            % Loop over all points
            for i = 1:this.numberOfPoints
                X(i) = this.pointsIn2D(i).projectedCoordinates(1);
                Y(i) = this.pointsIn2D(i).projectedCoordinates(2);
            end % for loop end
            
            % plot projected 2D points
            plot(X,Y,'.','Color','red')
            xlim([-100 100])
            ylim([-100 100])
        end % plotProjectedPoints() end
        
        
        %> @brief Plots the noisy 2D points
        %>
        %> @param this Pointer to object
        %> @param figureHandle Handle to figure
        function plotNoisyPixelPoints(this)
            % Declare X and Y vectors
            X = zeros(1,this.numberOfPoints);
            Y = zeros(1,this.numberOfPoints);
            
            % Loop over all points
            for i = 1:this.numberOfPoints
                X(i) = this.pointsIn2D(i).noisyPixelCoordinates(1);
                Y(i) = this.pointsIn2D(i).noisyPixelCoordinates(2);
            end % for loop end
            
            % plot projected 2D points
            plot(X,Y,'.','Color','green')
            
        end % plotNoisyPoints() end
        
        
        
        %> @brief Plots the distorted 2D points in u,v coordinates
        %>
        %> @param this Pointer to object
        %> @param figureHandle Handle to figure
        function plotDistortedImagePoints(this)
            % Declare X and Y vectors
            X = zeros(1,this.numberOfPoints);
            Y = zeros(1,this.numberOfPoints);
            
            % Loop over all points
            for i = 1:this.numberOfPoints
                X(i) = this.pointsIn2D(i).distortedNoisyCoordinatesInCameraFrame(1);
                Y(i) = this.pointsIn2D(i).distortedNoisyCoordinatesInCameraFrame(2);
            end % for loop end
            
            % plot projected 2D points
            plot(X,Y,'.','Color','red')
            xlim([-100 100])
            ylim([-100 100])
        end % plotProjectedPoints() end
        
        
        %> @brief Plots the distorted 2D points in x,y coordinates
        %>
        %> @param this Pointer to object
        %> @param figureHandle Handle to figure
        function plotDistortedPixelPoints(this, figureHandle)
            % Declare X and Y vectors
            X = zeros(1,this.numberOfPoints);
            Y = zeros(1,this.numberOfPoints);
            
            % Loop over all points
            for i = 1:this.numberOfPoints
                X(i) = this.pointsIn2D(i).distortedPixelCoordinates(1);
                Y(i) = this.pointsIn2D(i).distortedPixelCoordinates(2);
            end % for loop end
            
            % plot projected 2D points
            figure(figureHandle)
            plot(X,Y,'.','Color','red')
            xlim([-100 100])
            ylim([-100 100])
        end % plotProjectedPoints() end
    end % methods end
end % classdef end