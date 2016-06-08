%> @brief LineIn3D Class to store 3D line information
classdef LineIn3D < handle
    properties
        % Two points represent a line
        startPoint@PointIn3D            %> @param startPoint Starting point of 3D line, of type PointIn3D
        endPoint@PointIn3D              %> @param endPoint Ending point of 3D line, of type PointIn3D
        
        % Noise parameters
        mean;                           %> @param mean Vector with means for the noise of the 3D line
        variance;                       %> @param variance Vector with variances for the noise of the 3D line
    end % properties end
   
	methods
        %> @brief Constructor of a 3D line
        %> 
        %> @param pointOne First 3D point in homogeneous coordinates to generate a 3D line
        %> @param pointTwo Second 3D point in homogeneous coordinates to generate a 3D line
        %>
        %> @retval obj Object of LineIn3D
        function obj = LineIn3D(startPointInWorldFrame, endPointInWorldFrame)
            % Set homogeneous coordinates
            obj.startPoint = PointIn3D(startPointInWorldFrame(1), startPointInWorldFrame(2), startPointInWorldFrame(3));
            obj.endPoint = PointIn3D(endPointInWorldFrame(1), endPointInWorldFrame(2), endPointInWorldFrame(3));
            
            % Initialize noise parameter
            obj.setMean(-1);
            obj.setVariance(-1);
        end % Constructor LineIn3D end
        
        
        %> @brief Sets noise mean of this LineIn3D
        %>
        %> @param this Pointer to this object
        %> @param mean Mean of noise
        function setMean(this, mean)
            this.mean = mean;
            this.startPoint.setMean(mean);
            this.endPoint.setMean(mean);
        end % setMean() end
        
        
        %> @brief Sets noise variance of this LineIn3D
        %>
        %> @param this Pointer to this object
        %> @param variance Variance of noise
        function setVariance(this, variance)
            this.variance = variance;
            this.startPoint.setVariance(variance);
            this.endPoint.setVariance(variance);
        end % setVariance() end
        
        
        %> @brief Gets the starting point of this LineIn3D
        %>
        %> @param this Pointer to this object
        function [trueCoordinatesInWorldFrame, noisyCoordinatesInWorldFrame, trueCoordinatesInCameraFrame, noisyCoordinatesInCameraFrame] = getStartingPoint(this)
            % Get starting points of line
            trueCoordinatesInWorldFrame = this.startPoint.trueCoordinatesInWorldFrame;
            noisyCoordinatesInWorldFrame = this.startPoint.noisyCoordinatesInWorldFrame;
            trueCoordinatesInCameraFrame = this.startPoint.trueCoordinatesInCameraFrame;
            noisyCoordinatesInCameraFrame = this.startPoint.noisyCoordinatesInCameraFrame;
        end % getStartingPoint() end
        
        
        %> @brief Gets the ending point of this LineIn3D
        %>
        %> @param this Pointer to this object
        function [trueCoordinatesInWorldFrame, noisyCoordinatesInWorldFrame, trueCoordinatesInCameraFrame, noisyCoordinatesInCameraFrame] = getEndPoint(this)
            % Get end points of line
            trueCoordinatesInWorldFrame = this.endPoint.trueCoordinatesInWorldFrame;
            noisyCoordinatesInWorldFrame = this.endPoint.noisyCoordinatesInWorldFrame;
            trueCoordinatesInCameraFrame = this.endPoint.trueCoordinatesInCameraFrame;
            noisyCoordinatesInCameraFrame = this.endPoint.noisyCoordinatesInCameraFrame;
        end % getEndPoint() end
        
        
        %> @brief Gets the direction (from starting to ending point) of this LineIn3D
        %>
        %> @param this Pointer to this object
        function [trueDirectionInWorldFrame, noisyDirectionInWorldFrame, trueDirectionInCameraFrame, noisyDirectionInCameraFrame] = getDirectionOfLine(this)
            % Get direction of all point doubles, r_AB = r_B - r_A
            trueDirectionInWorldFrame = this.endPoint.trueCoordinatesInWorldFrame - this.startPoint.trueCoordinatesInWorldFrame;
            noisyDirectionInWorldFrame = this.endPoint.noisyCoordinatesInWorldFrame - this.startPoint.noisyCoordinatesInWorldFrame;
            trueDirectionInCameraFrame = this.endPoint.trueCoordinatesInCameraFrame - this.startPoint.trueCoordinatesInCameraFrame;
            noisyDirectionInCameraFrame = this.endPoint.noisyCoordinatesInCameraFrame - this.startPoint.noisyCoordinatesInCameraFrame;
        end % getDirectionOfLine() end
        
        
        %> @brief Computes the line coordinates w.r.t. the camera frame
        %>
        %> @param this Pointer to this object
        %> @param truePose True pose of the camera
        function computeCameraFrameCoordinates(this, truePose)
            % Transform into camera frame coordinates
            this.startPoint.computeCameraFrameCoordinates(truePose);
            this.endPoint.computeCameraFrameCoordinates(truePose);
        end % computeCameraFrameCoordinates() end
        
        
        %> @brief Adds noise to this LineIn3D
        %>
        %> @param this Pointer to this object
        %> @param T_CW Transformation (in R^(3x4)) from world frame into the camera frame
        function addNoise(this, T_CW)
            % Add noise to start and end point
            this.startPoint.addNoise(T_CW);
            this.endPoint.addNoise(T_CW);
        end % addNoise() end
        
        
        %> @brief Plots the ground truth of 3D line
        %>
        %> @param this Pointer to this object
        function plotTrueLine(this)
            % Concatenate the starting and end point
            X = [this.startPoint.trueCoordinatesInWorldFrame(1), this.endPoint.trueCoordinatesInWorldFrame(1)];
            Y = [this.startPoint.trueCoordinatesInWorldFrame(2), this.endPoint.trueCoordinatesInWorldFrame(2)];
            Z = [this.startPoint.trueCoordinatesInWorldFrame(3), this.endPoint.trueCoordinatesInWorldFrame(3)];
            
            % Plot the line
            plot3(X, Y, Z,'Color','blue');
        end % plotTrueLine() end
        
        
        %> @brief Plots the noise corrupted 3D line (measured line)
        %>
        %> @param this Pointer to this object
        function plotNoisyLine(this)
            % Concatenate the starting and end point
            X = [this.startPoint.noisyCoordinatesInWorldFrame(1), this.endPoint.noisyCoordinatesInWorldFrame(1)];
            Y = [this.startPoint.noisyCoordinatesInWorldFrame(2), this.endPoint.noisyCoordinatesInWorldFrame(2)];
            Z = [this.startPoint.noisyCoordinatesInWorldFrame(3), this.endPoint.noisyCoordinatesInWorldFrame(3)];
            
            % Plot the line
            plot3(X ,Y ,Z ,'Color','red');
        end % plotNoisyLine() end
    end % methods end
end % classdef end