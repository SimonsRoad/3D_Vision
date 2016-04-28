% =========================================================================
%> @brief Class Linecloud2D
%>
%> 
%>
%> 
%>
% =========================================================================
classdef Linecloud2D
    properties
        linesIn2D@LineIn2D
        numberOfLines
    end % properties end
    
    methods
        %> @brief Constructor of LineCloud2D
        %>
        %> @param 
        %>
        %> @retval obj Object of LineCloud2D
        function obj = Linecloud2D(lineCloud3D, trueLineProjectionMatrix)
            % Get number of lines
            obj.numberOfLines = lineCloud3D.numberOfLines;
            
            % Project all lines to the pixel plane
            for i = 1:obj.numberOfLines
                obj.linesIn2D(i) = LineIn2D(lineCloud3D.linesIn3D(i), trueLineProjectionMatrix);
            end % for end
            
            
        end % Constructor Linecloud2D end
    end % properties end
end % classdef end