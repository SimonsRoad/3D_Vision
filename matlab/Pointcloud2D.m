classdef Pointcloud2D < handle
    properties
        pointsIn2D@PointIn2D
        numberOfPoints
    end
    
    methods
        %> @brief Constructor projects a 3D pointcloud to 2D pointcloud based on camera ground truth pose
        %>
        %> @param calibrationMatrix The calibration matrix of a camera
        %> @param cameraTruePose Ground truth pose of a camera
        %> @param pointCloud3D 3D pointcloud
        %>
        %> @retval obj array with points in 2D
        function obj = Pointcloud2D(calibrationMatrix, cameraTruePose, pointCloud3D)
            obj.numberOfPoints = pointCloud3D.numberOfPoints;
        end % Constructor end
    end % methods end
end % classdef end