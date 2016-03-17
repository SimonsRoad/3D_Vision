classdef PointIn3D
    
    % Properties
    
    properties
        
        % Coordinates
        trueCoordinates = [x; y; z];
        noisyCoordinates = [x; y; z];
        id;
        noiseModel;
        
        % Flags
        
    end
    
    % Methods
    
    methods
        
        % Default constructor
        function obj = PointIn3D()
            
        end
        function addNoise(noiseModel)
            
        end
    end
    
end