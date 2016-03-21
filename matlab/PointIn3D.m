classdef PointIn3D
    
    % Properties
    
    properties
        
        % Coordinates
        trueCoordinates;
        noisyCoordinates;
        
    end
    
    % Methods
    
    methods
        
        % Default constructor
        function obj = PointIn3D(x,y,z)
            if nargin < 3
                error('PointIn3D has to be initialized with true coordinates')
            else
                obj.trueCoordinates = [x;y;z];
            end
        end
    end
    
end