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
        startPointIP@PointIn2D % IP stands for image plane
        endPointIP@PointIn2D
        sampledPoints@Pointcloud2D
        
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
%> @retval plueckerLineMatrix Pl?cker Matrix representation of a 3D line
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

function linesampling(this, nrOfSamples, kappa, p, imageToPixelMatrix, noiseType, mean, variance)
    
    % creation of samples of a line
    line = this.endPointIP - this.startPointIP;
    linelength = norm(line);
    direction = 1 / linelength * line;
    
    lengthstep = linelength / nrOfSamples;

    for i= 0:nrOfSamples
       sampledPoint = this.startPointIP + i * lengthstep * direction;
       this.sampledPoints(i) = sampledPoint;       
    end
    
    % add distortion and pixel noise to these samples
    this.sampledPoints.addDistortion(kappa,p);
    this.sampledPoints.calculateHomoegenousDistortedPixelPoints(imageToPixelMatrix);
    this.sampledPoints.setDistortedPixelCoordinatesFromHomogeneousCoordinates();
    this.sampledPoints.addPixelNoise(noiseType,mean, variance);
    % back projection and undistortion of these sampels
    this.sampledPoints.transformFromPixelToImage(imagetoPixelCoordinatesTrafo);
    this.sampledPoints.undistortPointCloud2D();
    
    % fit a line through these samples
    [y_hat, x, y] = linearRegression(this.sampledPoints);
    plot(x,y,'.','Color','blue');
    hold on
    plot(x,y_hat,'.','Color','red');
    hold off
end

function [y_hat, x, y] = linearRegression(PointCloudin2D)
    
    for i = 1:size(PointCloudin2D,1)
       point = PointCloudin2D.pointsIn2D(i);
       x(i) = point(1);
       y(i) = point(2);
    end
    
    X = [x ones(size(x))];
    beta = (X' * X)\ X' * y;
    y_hat = X * beta;
end

