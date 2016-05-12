% =========================================================================
%> @brief Class LineIn2D
%>
%> 
%>
%> 
%>
% =========================================================================
classdef LineIn2D < handle
    properties
        startPointIPTrue             % IP stands for image plane
        endPointIPTrue
        
        startPointIPNoisy
        endPointIPNoisy
        
        sampledPoints@Pointcloud2D
        
    end
    
    methods
        %> @brief
        %>
        %> @retval obj Object of class LineIn2D
        function obj = LineIn2D(lineIn3D, focalLengthMatrix)
            homogeneousStartPointIP = focalLengthMatrix * lineIn3D.startPoint.trueCoordinatesInWorldFrame;
            homogeneousEndPointIP = focalLengthMatrix * lineIn3D.endPoint.trueCoordinatesInWorldFrame;
            
            obj.startPointIPTrue = homogeneousStartPointIP(1:2);
            obj.endPointIPTrue = homogeneousEndPointIP(1:2);
        end % Constructor LineIn2D end
        
        
        %> @brief
        %>
        %> @param
        function samplingline(this, numberOfSamples)
    
            % creation of samples of a line
            line = this.endPointIPTrue - this.startPointIPTrue;
            linelength = norm(line);
            direction = 1 / linelength * line;

            lengthstep = linelength / numberOfSamples;
            %this.sampledPoints = assignPointToPointCloud(numberOfSamples);
            sampledpoints = [];
            for i= 1:numberOfSamples+1
               sampledPoint = this.startPointIPTrue + (i-1) * lengthstep * direction;
               sampledpoints = [sampledpoints;sampledPoint' 1];
            end
            
            this.sampledPoints = Pointcloud2D(sampledpoints);
            
            
        end % sampling LineIn2D end
        
        function measurementprocessing(this, kappa, p, imageToPixelMatrix, noiseType, mean, variance)
            
            % add distortion and pixel noise to these samples
            this.sampledPoints.addDistortion(kappa,p);
            this.sampledPoints.calculateHomoegenousDistortedPixelPoints(imageToPixelMatrix);
            this.sampledPoints.setDistortedPixelCoordinatesFromHomogeneousCoordinates();
            this.sampledPoints.addPixelNoise(noiseType, mean, variance);
            
            % back projection and undistortion of these sampels
            this.sampledPoints.transformFromPixelToImage(imageToPixelMatrix);
            this.sampledPoints.undistortPointCloud2D();

            % fit a line through these samples
            estimatedSamples = linearRegression(this.sampledPoints);
            
            this.startPointIPNoisy = [estimatedSamples(1,1); estimatedSamples(1,2)];
            this.endPointIPNoisy = [estimatedSamples(end,1); estimatedSamples(end,2)];
            
        end
        
        %> @brief
        %>
        %> @param
        function plotProjectedLine(this)
            % Concatenate the starting and end point
            X = [this.startPointIPTrue(1), this.endPointIPTrue(1)];
            Y = [this.startPointIPTrue(2), this.endPointIPTrue(2)];
            % Plot the line
            plot(X ,Y ,'Color','black');
        end % plotNoisyLine() end
        
    end % methods end
end % classdef LineIn2D end


        

