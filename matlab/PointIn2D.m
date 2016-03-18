classdef PointIn2D < handle
    
    % Properties
    
    properties
        
        % Coordinates
        projectedCoordinates2D = [x; y];
        noisyCoordinates2D = [x; y];
        id;
        noiseModel;
        
        % Flags
        
    end
    
    % Methods
    
    methods
        
        % constructor: input a noisy 3D point, constructs directly the
        % 2D correspondence
        function projectedPoint2D = PointIn2D(noisyPoint3D)
            projectedPoint2D.projectedCoordinates2D = calculateProjectedCoordinates(noisyPoint3D);
        end
        
        getProjectedCoordinates()
        
        % input a 2D point, output Coordinates of this 2D point
        function noisyPoint2D = getNoisyCoordinates(Point2D)
            noisyPoint2D = Point2D.noisyCoordinates2D;
        end
        
        function projectedPoints2D = calculateProjectedCoordinates(noisyPoints3D)
            
        end
        
        loadParameters(paramterFileDirectory)
        
    end
    
end