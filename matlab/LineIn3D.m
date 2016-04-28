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
        truePlueckerMatrixInWorldFrame;             %> @param truePlueckerCoordinates Plücker representation of ground truth 3D line in world frame (4x4 skew symmetric matrix)
        truePlueckerLineCoordinatesInWorldFrame;    %> @param truePlueckerLineCoordinates Plücker line representation of ground truth 3D line in world frame (6x1 vector)
        noisyPlueckerMatrixInWorldFrame;            %> @param noisyCoordinatesInWorldFrame Noise-ridden coordinates of 3D line in world frame
        noisyPlueckerLineCoordinatesInWorldFrame;   %> @param noisyHomogeneousCoordinatesInWorldFrame Homogeneous noise-ridden coordinates of 3D line in world frame
        
        % Coordinates in camera frame
        truePlueckerMatrixInCameraFrame;            %> @param trueCoordinatesInCameraFrame Coordinates of ground truth 3D line in camera frame
        truePlueckerLineCoordinatesInCameraFrame;   %> @param homogenousCoordinatesInCameraFrame Homogeneous coordinates of ground truth 3D line in camera frame
        noisyPlueckerMatrixInCameraFrame;           %> @param noisyCoordinatesInCameraFrame Noise-ridden coordinates of 3D line in camera frame
        noisyPlueckerLineCoordinatesInCameraFrame;  %> @param noisyHomogeneousCoordinatesInCameraFrame Homogeneous noise-ridden coordinates of 3D line in camera frame
        
        % Noise parameters
        mean;                                       %> @param mean Vector with means for the noise of the 3D line
        variance;                                   %> @param variance Vector with variances for the noise of the 3D line
    end % properties end
   
	methods
        %> @brief Constructor of a 3D line
        %> 
        %> @param pointOne First 3D point in homogeneous coordinates to generate a 3D line
        %> @param pointTwo Second 3D point in homogeneous coordinates to generate a 3D line
        %>
        %> @retval obj Object of LineIn3D
        function obj = LineIn3D(pointOne, pointTwo)
            % Plücker representation
            obj.truePlueckerMatrixInWorldFrame = hatOperator(pointOne, pointTwo);
            
            % Plücker line representation
            obj.truePlueckerLineCoordinatesInWorldFrame = plueckerMatrixToPlueckerLine(obj.truePlueckerMatrixInWorldFrame);
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
        %> @param this
        function addNoise(this)
            %------TODO----------------------------------------------------
        end
    end % methods end
end % classdef end

%% Helper functions

%> @brief
%>
%> @param plueckerMatrix
%>
%> @retval plueckerLine
function plueckerLine = plueckerMatrixToPlueckerLine(plueckerMatrix)
    l_12 = plueckerMatrix(1,2);
	l_13 = plueckerMatrix(1,3);
	l_14 = plueckerMatrix(1,4);
	l_23 = plueckerMatrix(2,3);
	l_24 = plueckerMatrix(2,4);
	l_34 = plueckerMatrix(3,4);
    
    plueckerLine = [l_12; l_13; l_14; l_23; l_24; l_34];
end

%> @brief
%>
%> @param pointOne First point on a 3D line
%> @param pointTwo Second point on a 3D line
%>
%> @retval plueckerLineMatrix Plücker Matrix representation of a 3D line
function plueckerMatrix = hatOperator(pointOne, pointTwo)
    plueckerMatrix = pointOne*pointTwo' - pointTwo*pointOne';
end