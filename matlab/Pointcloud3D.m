%> @brief Pointcloud3D Class to store all the 3D points of the scene
classdef Pointcloud3D < handle
    properties
        pointsIn3D@PointIn3D;   %> @param pointsIn3D@PointIn3D Array of type PointIn3D to store the points of the 3D pointcloud
        numberOfPoints;         %> @param numberOfPoints Number of points in the 3D pointcloud
        scale;                  %> @param scale Absolute scale of the simulation
        shape;                  %> @param shape Shape of the pointcloud. Current options: cubic, spherical, planar
    end % properties end
    
    methods
        %> @brief Constructor of Pointcloud3D class
        %>
        %> @param numberOfPoints_ Number of points in the 3D pointcloud
        %> @param shape_ Shape of the pointcloud
        %> @param scale_ Absolute scale of the simulation
        %> @param mean_ Vector of means for the points
        %> @param variance_ Vector of variances for the points
        %>
        %> @retval Object of type Pointcloud3D
        function obj = Pointcloud3D(numberOfPoints_,shape_,scale_,mean_,variance_)
            if nargin < 3
                error('Pointcloud has to be initialized with three arguments: Number of Points, Shape of the pointcloud, and the scale')
                disp('Currently implemented shapes: cubic, planar, spherical')
                return
            else
                obj.numberOfPoints = numberOfPoints_;
                obj.shape = shape_;
                obj.scale = scale_;
            end
            if strcmp(obj.shape,'spherical') 
                for i = 1:obj.numberOfPoints
                    % Use spherical coordinates to generate random points in a sphere
                    % Algorithm from Matlab, cited is
                    % [1] Knuth, D. The Art of Computer Programming. Vol. 2, 3rd ed. Reading, MA: Addison-Wesley Longman, 1998, pp. 134?136.
                    rvals = 2*rand(1)-1;
                    elevation = asin(rvals);
                    azimuth = 2*pi*rand(1);
                    radius = obj.scale*rand(1).^(1/3);
                    % Convert to Cartesian coordinates
                    [x,y,z] = sph2cart(azimuth, elevation, radius);
                    obj.pointsIn3D(i) = PointIn3D(x,y,z);
                end
            elseif strcmp(obj.shape,'cubic')
                for i = 1:obj.numberOfPoints
                    P = obj.scale*rand(3,1)-obj.scale*0.5;
                    obj.pointsIn3D(i) = PointIn3D(P(1),P(2),P(3));
                end
            elseif strcmp(obj.shape,'planar')
                for i = 1:obj.numberOfPoints
                    P = obj.scale*rand(2,1)-obj.scale*0.5;
                    obj.pointsIn3D(i) = PointIn3D(P(1),P(2),0);
                end
            else
                error('No matching shape. Currently implemented shapes: cubic, planar, spherical')
                return
            end
            for i = 1:obj.numberOfPoints
                obj.pointsIn3D(i).setMean(mean_);
                obj.pointsIn3D(i).setVariance(variance_);
            end
        end % Pointcloud3D() end
        
        
        %> @brief Projects a 3D pointcloud onto the image plane
        %>
        %> @param pointCloud3D A 3D pointcloud with points, of type Pointcloud3D
        %> @param focalLengthMatrix The focal length matrix F = diag(f, f, 1)
        %>
        %> @retval pointCloud2D A matrix with corresponding 2D points
        function pointCloud2D = ProjectionFrom3Dto2D(pointCloud3D, focalLengthMatrix)
            numberofpoints = pointCloud3D.getNumberOfPoints();
            
            % Construct all the 2D correspondences with a loop
            pointCloud2D = [];
            for i = 1:numberofpoints
                point3D = pointCloud3D.pointsIn3D(i);
                point2D = PointFrom3Dto2D(point3D, focalLengthMatrix);
                pointCloud2D = [pointCloud2D; point2D(1) point2D(2) point2D(3)];  
            end % for loop end
        end % ProjectionFrom3Dto2D() end
        
        
        %> @brief Adds noise to all points in this object
        %>
        %> @param this Pointer to this object
        %> @param T_WC Transformation matrix from camera frame into the world frame
        function addNoiseToAllPoints(this,T_WC)
            for i = 1:this.numberOfPoints
                this.pointsIn3D(i).addNoise(T_WC);
            end
        end % addNoiseToAllPoints() end

        
        %> @brief Plot the points in their true coordinates
        %>
        %> @param this Pointer to this object
        function plotTruePointcloud(this)
            X = zeros(this.numberOfPoints,1);
            Y = zeros(this.numberOfPoints,1);
            Z = zeros(this.numberOfPoints,1);
            
            for i = 1:this.numberOfPoints
                    X(i) = this.pointsIn3D(i).trueCoordinatesInWorldFrame(1);
                    Y(i) = this.pointsIn3D(i).trueCoordinatesInWorldFrame(2);
                    Z(i) = this.pointsIn3D(i).trueCoordinatesInWorldFrame(3);
            end
            
            plot3(X',Y',Z','.','markers',10,'Color','blue')
        end % plotTruePointcloud() end
        
        
        %> @brief Computes the coordinates w.r.t. the camera frame of all 3D points in this Pointcloud3D
        %>
        %> @param this Pointer to this object
        %> @param truePose The true pose of the camera as [R_CW | t]
        function computeCameraFrameCoordinates(this, truePose)
            % Fill in the true coordinates in camera frame
           for i = 1:this.numberOfPoints
               this.pointsIn3D(i).computeCameraFrameCoordinates(truePose);
           end
        end % computeCameraFrameCoordinates() end

        
        %> @brief Plot the points in their noisy coordinates
        %>
        %> @param this Pointer to this pointcloud
        %> @param plotConfidenceInterval Option wheter to plot the confidence intervals for the respective points
        function plotNoisyPointcloud(this,plotConnectingLines)
            X = zeros(this.numberOfPoints,1);
            Y = zeros(this.numberOfPoints,1);
            Z = zeros(this.numberOfPoints,1);
            
            for i = 1:this.numberOfPoints
                    X(i) = this.pointsIn3D(i).noisyCoordinatesInWorldFrame(1);
                    Y(i) = this.pointsIn3D(i).noisyCoordinatesInWorldFrame(2);
                    Z(i) = this.pointsIn3D(i).noisyCoordinatesInWorldFrame(3);
            end
            
            plot3(X',Y',Z','.','markers',10,'Color','red')

            if strcmp(plotConnectingLines,'true')
                hold on
                for i = 1:this.numberOfPoints
                    pts = [this.pointsIn3D(i).noisyCoordinatesInWorldFrame'; this.pointsIn3D(i).trueCoordinatesInWorldFrame'];
                    line(pts(:,1),pts(:,2),pts(:,3),'Color','red','LineWidth',0.1)
                end
            end
        end % plotNoisyPointcloud() end
                
        
        %> @brief Return number of points
        %>
        %> @param this Pointer to this pointcloud
        %>
        %> @retval numberOfPoints The number of points in this pointcloud
        function numberOfPoints = getNumberOfPoints(this)
            numberOfPoints = this.numberOfPoints;
        end % getNumberOfPoints() end
    end % methods end
end % classdef end