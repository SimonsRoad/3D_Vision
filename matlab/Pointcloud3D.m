classdef Pointcloud3D < handle
    
    % Properties
    
    properties
        


        pointsIn3D@PointIn3D;   %> @param pointsIn3D@PointIn3D Array of type PointIn3D to store the points of the 3D pointcloud
        numberOfPoints;         %> @param numberOfPoints Number of points in the 3D pointcloud
        scale;                  %> @param scale Absolute scale of the simulation
        shape;                  %> @param shape Shape of the pointcloud. Current options: cubic, spherical, planar

        
    end
    
    % Methods
    
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
        

        %> @brief Plot the points in their true coordinates
        %>
        %> @param this Pointer to this pointcloud
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

        end
        

        %> @brief Plot the points in their noisy coordinates
        %>
        %> @param this Pointer to this pointcloud
        %> @param plotConfidenceInterval Option wheter to plot the confidence intervals for the respective points
        function plotNoisyPointcloud(this,plotConfidenceInterval)

            X = zeros(this.numberOfPoints,1);
            Y = zeros(this.numberOfPoints,1);
            Z = zeros(this.numberOfPoints,1);
            
            for i = 1:this.numberOfPoints
                    X(i) = this.pointsIn3D(i).noisyCoordinatesInWorldFrame(1);
                    Y(i) = this.pointsIn3D(i).noisyCoordinatesInWorldFrame(2);
                    Z(i) = this.pointsIn3D(i).noisyCoordinatesInWorldFrame(3);
            end
            
            plot3(X',Y',Z','.','markers',10,'Color','red')

            
            if strcmp(plotConfidenceInterval,'true')
                hold on
                for i = 1:this.numberOfPoints
                    [x_e,y_e,z_e] = ellipsoid(this.pointsIn3D(i).trueCoordinatesInWorldFrame(1),...
                        this.pointsIn3D(i).trueCoordinatesInWorldFrame(2),...
                        this.pointsIn3D(i).trueCoordinatesInWorldFrame(3),...
                        2.575*this.pointsIn3D(i).anisotropicGaussianVariance(1),...
                        2.575*this.pointsIn3D(i).anisotropicGaussianVariance(2),...
                        2.575*this.pointsIn3D(i).anisotropicGaussianVariance(3));
                    surf(x_e,y_e,z_e, 'FaceColor','red','EdgeColor','none')
                    alpha(0.1)
                end
                hold off
            end
        end
        
        
        %> @brief Return number of points
        %>
        %> @param this Pointer to this pointcloud
        %>
        %> retval numberOfPoints The number of points in this pointcloud
        function numberOfPoints = getNumberOfPoints(this)
            numberOfPoints = this.numberOfPoints;
        end
    end
end