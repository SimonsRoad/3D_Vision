%> @brief LineIn2D Class to store a 2D line
classdef LineIn2D < handle
    properties
        startPointIPTrue            %> @param startPointIPTrue Start point of 2D line w.r.t. the image plane
        endPointIPTrue              %> @param endPointIPTrue End point of 2D line w.r.t. the image plane    
        
        startPointIPNoisy           %> @param startPointIPNoisy Noisy start point of 2D line w.r.t. the image plane
        endPointIPNoisy             %> @param endPointIPNoisy Noisy end point of 2D line w.r.t. the image plane
        
        sampledPoints@Pointcloud2D  %> @param sampledPoints Points between start and end point of line, of type Pointcloud2D
    end
    
    methods
        %> @brief Constructor
        %>
        %> @param lineIn3D A 3D line
        %> @param focalLengthMatrix Focal length matrix as diag(f, f, 1)
        %>
        %> @retval obj Object of class LineIn2D
        function obj = LineIn2D(lineIn3D, focalLengthMatrix)
            homogeneousStartPointIP = focalLengthMatrix/lineIn3D.startPoint.trueCoordinatesInCameraFrame(3) * lineIn3D.startPoint.trueCoordinatesInCameraFrame;
            homogeneousEndPointIP = focalLengthMatrix/lineIn3D.endPoint.trueCoordinatesInCameraFrame(3) * lineIn3D.endPoint.trueCoordinatesInCameraFrame;
            
            obj.startPointIPTrue = homogeneousStartPointIP(1:2);
            obj.endPointIPTrue = homogeneousEndPointIP(1:2);
        end % Constructor LineIn2D end
        
        
        %> @brief Samples points between start and end point of 2D line
        %>
        %> @param this Pointer to this object
        %> @param numberOfSamples Number of line samples between starting and ending point
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
        end % samplingline() end
        
        
        %> @brief
        %>
        %> @param this Pointer to this object
        %> @param kappa
        %> @param p
        %> @param imageToPixelMatrix
        %> @param noiseType
        %> @param mean
        %> @param variance
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
        end % measurementprocessing() end
        
        
        %> @brief Plots projected 2D line
        %>
        %> @param this Pointer to this object
        function plotProjectedLine(this)
            % Concatenate the starting and end point
            X = [this.startPointIPTrue(1), this.endPointIPTrue(1)];
            Y = [this.startPointIPTrue(2), this.endPointIPTrue(2)];
            % Plot the line
            plot(X ,Y ,'Color','blue');
        end % plotNoisyLine() end
        
        
        %> @brief Plots noisy 2D line
        %>
        %> @param this Pointer to this object
        function plotNoisyLine(this)
            X = [this.startPointIPNoisy(1), this.endPointIPNoisy(1)];
            Y = [this.startPointIPNoisy(2), this.endPointIPNoisy(2)];
            plot(X, Y, 'Color', 'red')
        end % plotNoisyLine() end
    end % methods end
end % classdef LineIn2D end


        

