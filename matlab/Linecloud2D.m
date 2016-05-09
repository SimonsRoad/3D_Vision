% =========================================================================
%> @brief Class Linecloud2D
%>
%> 
%>
%> 
%>
% =========================================================================
classdef Linecloud2D < handle
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
        function obj = Linecloud2D(lineCloud3D, focalLenghtMatrix)
            % Get number of lines
            obj.numberOfLines = lineCloud3D.getNumberOfLines();
            
            % Project all lines to the pixel plane
            for i = 1:obj.numberOfLines
                obj.linesIn2D(i) = LineIn2D(lineCloud3D.linesIn3D(i), focalLenghtMatrix);
            end % for end
            
            
        end % Constructor Linecloud2D end
        
        function samplingLines(this, numberOfSamples)
            for i = 1:this.numberOfLines
                this.linesIn2D(i).samplingline(numberOfSamples);
            end            
        end
        
        function measurementProcessing(this, kappa, p, imageToPixelMatrix, noiseType, mean, variance)
            for i = 1:this.numberOfLines
                this.linesIn2D(i).measurementprocessing(kappa, p, imageToPixelMatrix, noiseType, mean, variance);
            end            
        end
        
        %> @brief
        %>
        %> @param this
        function plotProjectedLines(this)
            % Plot every line seperately
            for i = 1:this.numberOfLines
                this.linesIn2D(i).plotProjectedLine();
            end
        end % plotTrueLinecloud() end
        
        
    end % properties end
end % classdef end