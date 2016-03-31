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
        %> @param calibrationMatrix The calibration matrix of a camera
        %> @param cameraTruePose Ground truth pose of a camera
        %> @param pointCloud3D 3D pointcloud
        %>
        %> @retval obj array with points in 2D
        function obj = Pointcloud2D(pointCloud3D, calibrationMatrix, imageToPixelMatrix, focalLengthMatrix, cameraTruePose, kappa, p)
            % Get number of points in pointCloud3D
            obj.numberOfPoints = pointCloud3D.getNumberOfPoints();
            
            % Construct all the 2D correspondences with a loop
            for i = 1:obj.numberOfPoints
                obj.pointsIn2D(i) = PointIn2D(pointCloud3D.pointsIn3D(i), calibrationMatrix, imageToPixelMatrix, focalLengthMatrix, cameraTruePose, kappa, p);
            end % for loop end
        end % Constructor end
        
        
        %> @brief Plots the projected 2D points
        %>
        %> @param this Pointer to object
        %> @param figureHandle Handle to figure
        function plotProjectedPoints(this, figureHandle)
            % Declare X and Y vectors
            X = zeros(1,this.numberOfPoints);
            Y = zeros(1,this.numberOfPoints);
            
            % Loop over all points
            for i = 1:this.numberOfPoints
                X(i) = this.pointsIn2D(i).projectedCoordinates(1);
                Y(i) = this.pointsIn2D(i).projectedCoordinates(2);
            end % for loop end
            
            % plot projected 2D points
            figure(figureHandle)
            plot(X,Y,'.','Color','red')
            xlim([-100 100])
            ylim([-100 100])
        end % plotProjectedPoints() end
        
        
        %> @brief Plots the noisy 2D points
        %>
        %> @param this Pointer to object
        %> @param figureHandle Handle to figure
        function plotNoisyPoints(this, figureHandle)
            % Declare X and Y vectors
            X = zeros(1,this.numberOfPoints);
            Y = zeros(1,this.numberOfPoints);
            
            % Loop over all points
            for i = 1:this.numberOfPoints
                X(i) = this.pointsIn2D(i).noisyCoordinates(1);
                Y(i) = this.pointsIn2D(i).noisyCoordinates(2);
            end % for loop end
            
            % plot projected 2D points
            figure(figureHandle)
            plot(X,Y,'.','Color','red')
            
        end % plotNoisyPoints() end
        
        
        
        %> @brief Plots the distorted 2D points in u,v coordinates
        %>
        %> @param this Pointer to object
        %> @param figureHandle Handle to figure
        function plotDistortedImagePoints(this, figureHandle)
            % Declare X and Y vectors
            X = zeros(1,this.numberOfPoints);
            Y = zeros(1,this.numberOfPoints);
            
            % Loop over all points
            for i = 1:this.numberOfPoints
                X(i) = this.pointsIn2D(i).distortedNoisyCoordinatesInCameraFrame(1);
                Y(i) = this.pointsIn2D(i).distortedNoisyCoordinatesInCameraFrame(2);
            end % for loop end
            
            % plot projected 2D points
            figure(figureHandle)
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