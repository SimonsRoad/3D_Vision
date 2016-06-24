%> @brief Pointcloud2D Class stores 2D points
classdef Pointcloud2D < handle
    properties
        pointsIn2D@PointIn2D                %> @param pointsIn2D Vector of type PointIn2D
        numberOfPoints                      %> @param numberOfPoints Number of points stored in vector pointsIn2D
    end % properties end
    
    methods
        %> @brief Copy Constructor
        %>
        %> @param pointcloud2D Matrix with points in 2D
        function obj = Pointcloud2D(pointcloud2D)
            obj.numberOfPoints = size(pointcloud2D,1);
            for i = 1:obj.numberOfPoints
                obj.pointsIn2D(i) = PointIn2D(pointcloud2D(i,1), pointcloud2D(i,2), pointcloud2D(i,3));
            end % for loop end
        end % Constructor end
        
        
        %> @brief Distorts every point in the 2D image plane
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
        
        
        %> @brief Calculates the homogeneous distorted points in uv coordinates
        %>
        %> @param this Pointer to Pointcloud2D object
        %> @param imageToPixelMatrix [this.kx, this.skew, this.x0; 0, this.ky, this.y0; 0, 0, 1];
        function calculateHomoegenousDistortedPixelPoints(this, imageToPixelMatrix)
            % for every 2D point
           for i = 1:this.numberOfPoints
               this.pointsIn2D(i).calculateHomoegeneousDistortedPixelCoordinates(imageToPixelMatrix);
           end
        end % calculateHomoegenousDistortedPixelPoints() end
        
        
        %> @brief Calculates the euclidean distorted points in uv coordinates (given homogeneous distorted pixel points)
        %>
        %> @param this Pointer to Pointcloud2D object
        function setDistortedPixelCoordinatesFromHomogeneousCoordinates(this)
             % for every 2D point
           for i = 1:this.numberOfPoints
               this.pointsIn2D(i).setDistortedPixelCoordinatesFromHomogeneousCoordinates();
           end
        end % setDistortedPixelCoordinatesFromHomogeneousCoordinates() end
        
        
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
        
        
        %> @brief Backtransforms a pixel coordinates into image plane coordinates
        %>
        %> @param this Pointer to Pointcloud2D object
        %> @param calibrationMatrix calibration matrix
        function transformFromPixelToImage(this, imagetoPixelCoordinatesTrafo)
            % for every 2D point
            for i = 1:this.numberOfPoints
                this.pointsIn2D(i).transformFromPixelToImage(imagetoPixelCoordinatesTrafo);
            end
        end % transformFromPixelToImage() end
        
        
        %> @brief Undistorts points in Pointcloud2D
        %>
        %> @param this Pointer to this object
        function undistortPointCloud2D(this)
            % for every 2D point
            A=[];
            b=[];
            for i = 1:this.numberOfPoints
               [y, M] = this.pointsIn2D(i).createLSforUndistortion();
               b = [b;y];
               A = [A;M];
            end
            x = A\b;

            kappa_ = [x(1); x(2); x(3)];
            p_ = [x(4); x(5)];

            for i = 1:this.numberOfPoints
               this.pointsIn2D(i).undistortion(kappa_,p_);
            end
        end % undistortion() end             
        
        
        %> @brief Fits a line trough a Pointcloud2D
        %>
        %> @param PointCloudin2D
        %>
        %> @retval estimatedSamples
        function estimatedSamples = linearRegression(PointCloudin2D)
            x = zeros(PointCloudin2D.numberOfPoints,1);
            y = zeros(PointCloudin2D.numberOfPoints,1);
            
            for i = 1:PointCloudin2D.numberOfPoints
               point = PointCloudin2D.pointsIn2D(i);
               x(i) = point.u;
               y(i) = point.v;
            end
            
            X = [x ones(size(x))];
            beta = (X' * X)\ X' * y;
            y_hat = X * beta;
            
            pointcloud2D = [x y ones(PointCloudin2D.numberOfPoints,1)];
            estimatedSamples = pointcloud2D;
            %estimatedSamples = Pointcloud2D(pointcloud2D);
            
%             plot(x,y,'Color','blue');
%             hold on
%             plot(x,y_hat,'x','Color','red');
            
        end % linearRegression() end
        
        
        %> @brief Plots the projected 2D points
        %>
        %> @param this Pointer to object
        function plotImagePoints(this)
            % Declare X and Y vectors
            X = zeros(1,this.numberOfPoints);
            Y = zeros(1,this.numberOfPoints);
            
            % Loop over all points
            for i = 1:this.numberOfPoints
                X(i) = this.pointsIn2D(i).projectedCoordinates(1);
                Y(i) = this.pointsIn2D(i).projectedCoordinates(2);
            end % for loop end
            
            % plot projected 2D points
            plot(X,Y,'o','Color','blue','DisplayName', 'True 3D Pointcloud Projection')
            alpha(.5)
        end % plotProjectedPoints() end
        
        
        %> @brief Plots the distorted 2D points in u,v coordinates
        %>
        %> @param this Pointer to object
        function plotDistortedImagePoints(this)
            % Declare X and Y vectors
            X = zeros(1,this.numberOfPoints);
            Y = zeros(1,this.numberOfPoints);
            
            % Loop over all points
            for i = 1:this.numberOfPoints
                X(i) = this.pointsIn2D(i).distortedCoordinatesInCameraFrame(1);
                Y(i) = this.pointsIn2D(i).distortedCoordinatesInCameraFrame(2);
            end % for loop end
            
            % plot projected 2D points
            plot(X,Y,'+','Color','green','DisplayName', 'Distorted True 3D Pointcloud Projection')
            alpha(.5)
        end % plotDistortedImagePoints() end
        
        
        %> @brief Plots the distorted 2D points in x,y coordinates
        %>
        %> @param this Pointer to object
        function plotDistortedPixelPoints(this)
            % Declare X and Y vectors
            X = zeros(1,this.numberOfPoints);
            Y = zeros(1,this.numberOfPoints);
            
            % Loop over all points
            for i = 1:this.numberOfPoints
                X(i) = this.pointsIn2D(i).distortedPixelCoordinates(1);
                Y(i) = this.pointsIn2D(i).distortedPixelCoordinates(2);
            end % for loop end
            
            % plot projected 2D points
            plot(X,Y,'+','Color','green','DisplayName', 'Distorted True 3D Pointcloud in Pixel Coordinates')
            alpha(.5)
        end % plotDistortedPixelPoints() end
        
        
        %> @brief Plots the noisy 2D points
        %>
        %> @param this Pointer to object
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
            plot(X,Y,'x','Color','magenta','DisplayName', 'Distorted True 3D Pointcloud in Pixel Coordinates with Pixelnoise')
            alpha(.5)
        end % plotNoisyPoints() end
        
        
        %> @brief Plots the noisy 2D points
        %>
        %> @param this Pointer to object
        function plotBackProjectedImagePoints(this)
            % Declare X and Y vectors
            X = zeros(1,this.numberOfPoints);
            Y = zeros(1,this.numberOfPoints);
            
            % Loop over all points
            for i = 1:this.numberOfPoints
                X(i) = this.pointsIn2D(i).backProjectionFromPixelToImageCoordinates(1);
                Y(i) = this.pointsIn2D(i).backProjectionFromPixelToImageCoordinates(2);
            end % for loop end
            
            % plot projected 2D points
            plot(X,Y,'x','Color','magenta','DisplayName', 'Distorted True 3D Pointcloud with Pixelnoise on Image Plane')
        end % plotBackProjectedImagePoints() end
        
        
        %> @brief Plots the projected 2D points
        %>
        %> @param this Pointer to object
        function plotUndistortedImagePoints(this)
            % Declare X and Y vectors
            X = zeros(1,this.numberOfPoints);
            Y = zeros(1,this.numberOfPoints);
            
            % Loop over all points
            for i = 1:this.numberOfPoints
                X(i) = this.pointsIn2D(i).undistortedCoordinatesInCameraFrame(1);
                Y(i) = this.pointsIn2D(i).undistortedCoordinatesInCameraFrame(2);
            end % for loop end
            
            % plot projected 2D points
            plot(X,Y,'*','Color','black','DisplayName', 'Undistorted True 3D Pointcloud with Pixelnoise on Image Plane')
            alpha(.5)
        end % plotProjectedPoints() end
    end % methods end
end % classdef end