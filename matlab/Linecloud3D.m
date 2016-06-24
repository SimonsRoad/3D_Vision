%> @brief Linecloud3D Class to store a 3D linecloud
classdef Linecloud3D < handle
    properties
        linesIn3D@LineIn3D          %> @param linesIn3D Array of type LineIn3D to store lines of the 3D linecloud
        numberOfLines               %> @param numberOfLines Number of lines in the 3D linecloud
        shape                       %> @param shape Shape of the pointcloud. Current options: cubic, spherical, planar
        scale                       %> @param scale Absolute scale of the simulation
    end % properties end
    
    methods
        %> @brief Linecloud3D Constructor
        %>
        %> @param numberOfLines The number of lines to be generated
        %> @param shape Shape of line cloud
        %> @param scale Scale of line cloud
        %> @param mean Vector with means for the noisy lines
        %> @param variance Vector with variances for the noisy lines
        %>
        %> @retval obj Object of Linecloud3D
        function obj = Linecloud3D(numberOfLines, shape, scale, mean, variance)
            obj.numberOfLines = numberOfLines;
            obj.shape = shape;
            obj.scale = scale;
            
            if strcmp(obj.shape,'spherical') 
                for i = 1:obj.numberOfLines
                    % Use spherical coordinates to generate random points in a sphere
                    % Algorithm from Matlab, cited is
                    % [1] Knuth, D. The Art of Computer Programming. Vol. 2, 3rd ed. Reading, MA: Addison-Wesley Longman, 1998, pp. 134?136.
                    rvals = 2*rand([1 2])-1;
                    elevation = asin(rvals);
                    azimuth = 2*pi*rand([1 2]);
                    radius = obj.scale*rand([1 2]).^(1/3);
                    
                    % Convert to Cartesian coordinates
                    [x1,y1,z1] = sph2cart(azimuth(1), elevation(1), radius(1));
                    [x2,y2,z2] = sph2cart(azimuth(2), elevation(2), radius(2));
                    obj.linesIn3D(i) = LineIn3D([x1; y1; z1], [x2; y2; z2]);
                end
            elseif strcmp(obj.shape,'cubic')
                for i = 1:obj.numberOfLines
                    P = obj.scale*rand(3,2)-obj.scale*0.5;
                    obj.linesIn3D(i) = LineIn3D([P(1,1);P(1,2);P(1,3);1], [P(2,1);P(2,2);P(2,3);1]);
                end
            elseif strcmp(obj.shape,'planar')
                for i = 1:obj.numberOfLines
                    P = obj.scale*rand(2,2)-obj.scale*0.5;
                    obj.linesIn3D(i) = LineIn3D([P(1,1);P(1,2);0;1],[P(2,1);P(2,2);0;1]);
                end
            else
                error('No matching shape. Currently implemented shapes: cubic, planar, spherical')
                return
            end
            
            % Set mean and variance for each line
            for i = 1:obj.numberOfLines
                obj.linesIn3D(i).setMean(mean);
                obj.linesIn3D(i).setVariance(variance);
            end
        end % Constructor Linecloud3D end
        
        
        %> @brief Computes the line coordinates w.r.t. the camera frame
        %>
        %> @param this Pointer to this object
        %> @param truePose True pose of camera as [R | t]
        function computeCameraFrameCoordinates(this, truePose)
            % Fill in the true coordinates in camera frame
            for i = 1:this.numberOfLines
                this.linesIn3D(i).computeCameraFrameCoordinates(truePose);
            end
        end % computeCameraFrameCoordinates() end
        
        
        %> @brief Adds noise to all lines stored in this linecloud
        %>
        %> @param this Pointer to this object
        %> @param T_CW Transformation matrix from world frame to camera frame
        function addNoiseToAllLines(this, T_CW)
            for i = 1:this.numberOfLines
                this.linesIn3D(i).addNoise(T_CW);
            end
        end % addNoiseToAllLines() end
        
        
        %> @brief Plots the ground truth linecloud
        %>
        %> @param this Pointer to this object
        function plotTrueLinecloud(this)
            X = zeros(this.numberOfLines,2);
            Y = zeros(this.numberOfLines,2);
            Z = zeros(this.numberOfLines,2);
            % Plot every line seperately
            for i = 1:this.numberOfLines
                %this.linesIn3D(i).plotTrueLine();
                X(i,:) = [this.linesIn3D(i).startPoint.trueCoordinatesInWorldFrame(1),this.linesIn3D(i).endPoint.trueCoordinatesInWorldFrame(1)];
                Y(i,:) = [this.linesIn3D(i).startPoint.trueCoordinatesInWorldFrame(2),this.linesIn3D(i).endPoint.trueCoordinatesInWorldFrame(2)];
                Z(i,:) = [this.linesIn3D(i).startPoint.trueCoordinatesInWorldFrame(3),this.linesIn3D(i).endPoint.trueCoordinatesInWorldFrame(3)];
            end
            
            % Plot the line
            plot3(X', Y', Z','Color','blue','DisplayName', 'True 3D Line');
        end % plotTrueLinecloud() end
        
        
        %> @brief Plots the noisy linecloud (measured lines)
        %>
        %> @param this Pointer to this object
        function plotNoisyLinecloud(this)
            X = zeros(this.numberOfLines,2);
            Y = zeros(this.numberOfLines,2);
            Z = zeros(this.numberOfLines,2);
            % Plot every line seperately
            for i = 1:this.numberOfLines
                %this.linesIn3D(i).plotNoisyLine();
                X(i,:) = [this.linesIn3D(i).startPoint.noisyCoordinatesInWorldFrame(1),this.linesIn3D(i).endPoint.noisyCoordinatesInWorldFrame(1)];
                Y(i,:) = [this.linesIn3D(i).startPoint.noisyCoordinatesInWorldFrame(2),this.linesIn3D(i).endPoint.noisyCoordinatesInWorldFrame(2)];
                Z(i,:) = [this.linesIn3D(i).startPoint.noisyCoordinatesInWorldFrame(3),this.linesIn3D(i).endPoint.noisyCoordinatesInWorldFrame(3)];
            end
            
            % Plot the line
            plot3(X', Y', Z','Color','red','DisplayName', 'Noisy 3D Line');
        end % plotTrueNoisycloud() end
        
        
        %> @brief Returns number of lines stored in this linecloud
        %>
        %> @param this Pointer to this linecloud
        %>
        %> @retval numberOfPoints The number of points in this pointcloud
        function numberOfPoints = getNumberOfLines(this)
            numberOfPoints = this.numberOfLines;
        end
    end % methods end
end % classdef end