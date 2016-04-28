% =========================================================================
%> @brief Class LineIn2D
%>
%> 
%>
%> 
%>
% =========================================================================
classdef LineIn2D
    properties
        pixelCoordinates
    end
    
    methods
        %> @brief
        %>
        %> @retval obj Object of class LineIn2D
        function obj = LineIn2D(lineIn3D, lineProjectionMatrix)
            % Initialize projected line
            obj.pixelCoordinates = zeros(3,1);
            
            % Project the 3D line to a 2D line with the side operator
            for i = 1:3
                obj.pixelCoordinates(i) = sideOperator(lineProjectionMatrix(i,:)', lineIn3D.truePlueckerLineCoordinatesInWorldFrame);
            end
            
            % Normalize homogeneous
            obj.pixelCoordinates = obj.pixelCoordinates / obj.pixelCoordinates(3);
        end % Constructor LineIn2D end
    end % methods end
end % classdef LineIn2D end


%% Helper functions

%> @brief
%>
%> @param pointOne First point on a 3D line
%> @param pointTwo Second point on a 3D line
%>
%> @retval plueckerLineMatrix Plücker Matrix representation of a 3D line
function plueckerLineMatrix = hatOperator(pointOne, pointTwo)
    plueckerLineMatrix = pointOne*pointTwo' - pointTwo*pointOne';
end


%> @brief
%>
%> @param plueckerLineLeft
%> @param plueckerLineRight
%>
%> @retval l 2D line representation
function l = sideOperator(plueckerLineLeft, plueckerLineRight)
    l = plueckerLineLeft(1)*plueckerLineRight(6) + plueckerLineLeft(6)*plueckerLineRight(1) ...
        - plueckerLineLeft(2)*plueckerLineRight(5) - plueckerLineLeft(5)*plueckerLineRight(2) ...
        + plueckerLineLeft(3)*plueckerLineRight(4) + plueckerLineLeft(4)*plueckerLineRight(3);
end