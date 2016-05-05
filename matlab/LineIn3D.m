% =========================================================================
%> @brief Class LineIn3D
%>
%> 
%>
%> 
%>
% =========================================================================
classdef LineIn3D < handle
    properties
        % Coordinates in world frame
        startingPointTrueCoordinatesInWorldFrame;               %> @param
        startingPointTrueHomogeneousCoordinatesInWorldFrame;	%> @param
        startingPointNoisyCoordinatesInWorldFrame;              %> @param
        startingPointNoisyHomogeneousCoordinatesInWorldFrame;   %> @param
        
        endPointTrueCoordinatesInWorldFrame;                    %> @param 
        endPointTrueHomogeneousCoordinatesInWorldFrame;         %> @param
        endPointNoisyCoordinatesInWorldFrame;                   %> @param
        endPointNoisyHomogeneousCoordinatesInWorldFrame         %> @param
        
        % Coordinates in camera frame
        startingPointTrueCoordinatesInCameraFrame;              %> @param
        startingPointTrueHomogeneousCoordinatesInCameraFrame;	%> @param
        startingPointNoisyCoordinatesInCameraFrame;             %> @param
        startingPointNoisyHomogeneousCoordinatesInCameraFrame;	%> @param
        
        endPointTrueCoordinatesInCameraFrame;                   %> @param
        endPointTrueHomogeneousCoordinatesInCameraFrame;        %> @param
        endPointNoisyCoordinatesInCameraFrame;                  %> @param
        endPointNoisyHomogeneousCoordinatesInCameraFrame;       %> @param
        
        % Noise parameters
        mean;                                                   %> @param mean Vector with means for the noise of the 3D line
        variance;                                               %> @param variance Vector with variances for the noise of the 3D line
    end % properties end
   
	methods
        %> @brief Constructor of a 3D line
        %> 
        %> @param pointOne First 3D point in homogeneous coordinates to generate a 3D line
        %> @param pointTwo Second 3D point in homogeneous coordinates to generate a 3D line
        %>
        %> @retval obj Object of LineIn3D
        function obj = LineIn3D(startingPointInWorldFrame, endPointInWorldFrame)
            % Set homogeneous coordinates
            obj.startingPointTrueHomogeneousCoordinatesInWorldFrame = startingPointInWorldFrame;
            obj.endPointTrueHomogeneousCoordinatesInWorldFrame = endPointInWorldFrame;
            
            % Set euclidean coordinates
            obj.startingPointTrueCoordinatesInWorldFrame = obj.startingPointTrueHomogeneousCoordinatesInWorldFrame(1:3);
            obj.endPointTrueCoordinatesInWorldFrame = obj.endPointTrueHomogeneousCoordinatesInWorldFrame(1:3);
        end % Constructor LineIn3D end
        
        
        %> @brief
        %>
        %> @param this
        %> @param mean
        function setMean(this, mean)
            this.mean = mean;
        end % setMean() end
        
        
        %> @brief
        %>
        %> @param this
        %> @param variance
        function setVariance(this, variance)
            this.variance = variance;
        end % setVariance() end
        
        
        %> @brief
        %>
        %> @param this Pointer to object
        function [trueCoordinatesInWorldFrame, noisyCoordinatesInWorldFrame, trueCoordinatesInCameraFrame, noisyCoordinatesInCameraFrame] = getStartingPoint(this)
            % Get starting points of line
            trueCoordinatesInWorldFrame = this.startingPointTrueCoordinatesInWorldFrame;
            noisyCoordinatesInWorldFrame = this.startingPointNoisyCoordinatesInWorldFrame;
            trueCoordinatesInCameraFrame = this.startingPointTrueCoordinatesInCameraFrame;
            noisyCoordinatesInCameraFrame = this.startingPointNoisyCoordinatesInCameraFrame;
        end % getStartingPoint() end
        
        
        %> @brief
        %>
        %> @param this Pointer to object
        function [trueCoordinatesInWorldFrame, noisyCoordinatesInWorldFrame, trueCoordinatesInCameraFrame, noisyCoordinatesInCameraFrame] = getEndPoint(this)
            % Get end points of line
            trueCoordinatesInWorldFrame = this.endPointTrueCoordinatesInWorldFrame;
            noisyCoordinatesInWorldFrame = this.endPointNoisyCoordinatesInWorldFrame;
            trueCoordinatesInCameraFrame = this.endPointTrueCoordinatesInCameraFrame;
            noisyCoordinatesInCameraFrame = this.endPointNoisyCoordinatesInCameraFrame;
        end % getEndPoint() end
        
        
        %> @brief
        %>
        %> @param this Pointer to object
        function [trueDirectionInWorldFrame, trueDirectionInCameraFrame, noisyDirectionInWorldFrame, noisyDirectionInCameraFrame] = getDirectionOfLine(this)
            % Get direction of all point doubles, r_AB = r_B - r_A
            trueDirectionInWorldFrame = this.endPointTrueCoordinatesInWorldFrame - this.startingPointTrueCoordinatesInWorldFrame;
            trueDirectionInCameraFrame = this.endPointTrueCoordinatesInCameraFrame - this.startingPointTrueCoordinatesInCameraFrame;
            noisyDirectionInWorldFrame = this.endPointNoisyCoordinatesInWorldFrame - this.startingPointNoisyCoordinatesInWorldFrame;
            noisyDirectionInCameraFrame = this.endPointNoisyCoordinatesInCameraFrame - this.startingPointNoisyCoordinatesInCameraFrame;
        end % getDirectionOfLine() end
        
        
        %> @brief
        %>
        %> @param this
        %> @param truePose
        function computeCameraFrameCoordinates(this, truePose)
            % Transform into camera frame coordinates
            this.startingPointTrueHomogeneousCoordinatesInCameraFrame = [truePose; 0, 0, 0, 1]*this.startingPointTrueHomogeneousCoordinatesInWorldFrame;
            this.startingPointTrueCoordinatesInCameraFrame = this.startingPointTrueHomogeneousCoordinatesInCameraFrame(1:3);

            this.endPointTrueHomogeneousCoordinatesInCameraFrame = [truePose; 0, 0, 0, 1]*this.endPointTrueHomogeneousCoordinatesInWorldFrame;
            this.endPointTrueCoordinatesInCameraFrame = this.endPointTrueHomogeneousCoordinatesInCameraFrame(1:3);
        end % computeCameraFrameCoordinates() end
        
        
        %> @brief
        %>
        %> @param this
        %> @param T_CW Transformation (in R^(3x4)) from world frame into the camera frame
        function addNoise(this, T_CW)
            % Initialize noisy coordinates in camera frame
            this.startingPointNoisyHomogeneousCoordinatesInCameraFrame = this.startingPointTrueHomogeneousCoordinatesInCameraFrame;
            this.endPointNoisyHomogeneousCoordinatesInCameraFrame = this.endPointTrueHomogeneousCoordinatesInCameraFrame;
            
            % Add noise to starting point
            noiseInCameraX = normrnd(this.mean(1),this.variance(1));
            noiseInCameraY = normrnd(this.mean(2),this.variance(2));
            noiseInCameraZ = normrnd(this.mean(3),this.variance(3));
            this.startingPointNoisyHomogeneousCoordinatesInCameraFrame = this.startingPointNoisyHomogeneousCoordinatesInCameraFrame + [noiseInCameraX; noiseInCameraY; noiseInCameraZ; 0];
            this.startingPointNoisyCoordinatesInCameraFrame = this.startingPointNoisyHomogeneousCoordinatesInCameraFrame(1:3);
            
            % Add noise to end point
            noiseInCameraX = normrnd(this.mean(1),this.variance(1));
            noiseInCameraY = normrnd(this.mean(2),this.variance(2));
            noiseInCameraZ = normrnd(this.mean(3),this.variance(3));
            this.endPointNoisyHomogeneousCoordinatesInCameraFrame = this.endPointNoisyHomogeneousCoordinatesInCameraFrame + [noiseInCameraX; noiseInCameraY; noiseInCameraZ; 0];
            this.endPointNoisyCoordinatesInCameraFrame = this.endPointNoisyHomogeneousCoordinatesInCameraFrame(1:3);
            
            % Set noisy coordinates in world frame
            this.startingPointNoisyHomogeneousCoordinatesInWorldFrame = [T_CW; 0, 0, 0, 1]\this.startingPointNoisyHomogeneousCoordinatesInCameraFrame;
            this.startingPointNoisyCoordinatesInWorldFrame = this.startingPointNoisyHomogeneousCoordinatesInWorldFrame(1:3);
            
            this.endPointNoisyHomogeneousCoordinatesInWorldFrame = [T_CW; 0, 0, 0, 1]\this.endPointNoisyHomogeneousCoordinatesInCameraFrame;
            this.endPointNoisyCoordinatesInWorldFrame = this.endPointNoisyHomogeneousCoordinatesInWorldFrame(1:3);
        end % addNoise() end
        
        
        %> @brief
        %>
        %> @param
        function plotTrueLine(this)
            % Concatenate the starting and end point
            X = [this.startingPointTrueCoordinatesInWorldFrame(1), this.endPointTrueCoordinatesInWorldFrame(1)];
            Y = [this.startingPointTrueCoordinatesInWorldFrame(2), this.endPointTrueCoordinatesInWorldFrame(2)];
            Z = [this.startingPointTrueCoordinatesInWorldFrame(3), this.endPointTrueCoordinatesInWorldFrame(3)];
            
            % Plot the line
            plot3(X, Y, Z,'Color','blue');
        end % plotTrueLine() end
        
        
        %> @brief
        %>
        %> @param
        function plotNoisyLine(this)
            % Concatenate the starting and end point
            X = [this.startingPointNoisyCoordinatesInWorldFrame(1), this.endPointNoisyCoordinatesInWorldFrame(1)];
            Y = [this.startingPointNoisyCoordinatesInWorldFrame(2), this.endPointNoisyCoordinatesInWorldFrame(2)];
            Z = [this.startingPointNoisyCoordinatesInWorldFrame(3), this.endPointNoisyCoordinatesInWorldFrame(3)];
            
            % Plot the line
            plot3(X ,Y ,Z ,'Color','red');
        end % plotNoisyLine() end
    end % methods end
end % classdef end