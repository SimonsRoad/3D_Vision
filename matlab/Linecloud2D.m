%> @brief Linecloud2D Class to store 2D lines
classdef Linecloud2D < handle
    properties
        linesIn2D@LineIn2D      %> @param linesIn2D Array of type LineIn2D with lines in linecloud
        numberOfLines           %> @param numberOfLines Number of lines in linecloud
    end % properties end
    
    methods
        %> @brief Constructor of LineCloud2D
        %>
        %> @param lineCloud3D A 3D linecloud to be projected
        %> @param focalLengthMatrix Focal length matrix as diag(f,f,1)
        %>
        %> @retval obj Object of LineCloud2D
        function obj = Linecloud2D(lineCloud3D, focalLengthMatrix)
            % Get number of lines
            obj.numberOfLines = lineCloud3D.getNumberOfLines();
            % Project all lines to the pixel plane
            for i = 1:obj.numberOfLines
                obj.linesIn2D(i) = LineIn2D(lineCloud3D.linesIn3D(i), focalLengthMatrix);
            end % for end
            
            
        end % Constructor Linecloud2D end
        
        
        %> @brief Samples the lines of this linecloud
        %>
        %> @param this Pointer to this object
        %> @param numberOfSamples Number of samples between start and end point of each line
        function samplingLines(this, numberOfSamples)
            for i = 1:this.numberOfLines
                this.linesIn2D(i).samplingline(numberOfSamples);
            end            
        end % samplingLines() end
        
        
        %> @brief
        %>
        %> @param this Pointer to this object
        %> @param kappa
        %> @param p
        %> @param imageToPixelMatrix
        %> @param noiseType
        %> @param mean
        %> @param variance
        function measurementProcessing(this, kappa, p, imageToPixelMatrix, noiseType, mean, variance)
            for i = 1:this.numberOfLines
                this.linesIn2D(i).measurementprocessing(kappa, p, imageToPixelMatrix, noiseType, mean, variance);
            end            
        end % measurementProcessing() end
        
        
        %> @brief Plots projected lines of linecloud
        %>
        %> @param this Pointer to this object
        function plotProjectedLines(this)
            % Plot every line seperately
            hold on
            for i = 1:this.numberOfLines
                this.linesIn2D(i).plotProjectedLine();
            end
            hold off
        end % plotTrueLinecloud() end
        
        
        %> @brief Plots noisy lines of linecloud
        %>
        %> @param this Pointer to this object
        function plotNoisyLines(this)
            hold on
            for i = 1:this.numberOfLines
                this.linesIn2D(i).plotNoisyLine();
            end
            hold off
        end % plotNoisyLines() end
    end % properties end
end % classdef end