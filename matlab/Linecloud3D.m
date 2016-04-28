% =========================================================================
%> @brief Class Linecloud3D
%>
%> 
%>
%> 
%>
% =========================================================================
classdef Linecloud3D < handle
    properties
        linesIn3D@LineIn3D
        numberOfLines
        shape
        scale
    end % properties end
    
    methods
        %> @brief Linecloud3D Constructor
        %>
        %> @param numberOfLines The number of lines to be generated
        %> @param shape Shape of line cloud
        %> @param scale Scale of line cloud
        %> @param mean Vector with means for the noisy lines
        %> @param variance Vector with variances for the noisy lines
        %>
        %> @retval obj Object of Linecloud3D
        function obj = Linecloud3D(numberOfLines, shape, scale, mean, variance)
            obj.numberOfLines = numberOfLines;
            obj.shape = shape;
            obj.scale = scale;
            
            if strcmp(obj.shape,'spherical') 
                for i = 1:obj.numberOfLines
                    % Use spherical coordinates to generate random points in a sphere
                    % Algorithm from Matlab, cited is
                    % [1] Knuth, D. The Art of Computer Programming. Vol. 2, 3rd ed. Reading, MA: Addison-Wesley Longman, 1998, pp. 134?136.
                    rvals = 2*rand([1 2])-1;
                    elevation = asin(rvals);
                    azimuth = 2*pi*rand([1 2]);
                    radius = obj.scale*rand([1 2]).^(1/3);
                    
                    % Convert to Cartesian coordinates
                    [x1,y1,z1] = sph2cart(azimuth(1), elevation(1), radius(1));
                    [x2,y2,z2] = sph2cart(azimuth(2), elevation(2), radius(2));
                    obj.linesIn3D(i) = LineIn3D([x1; y1; z1; 1], [x2; y2; z2; 1]);
                end
            elseif strcmp(obj.shape,'cubic')
                for i = 1:obj.numberOfLines
                    P = obj.scale*rand(3,2)-obj.scale*0.5;
                    obj.linesIn3D(i) = LineIn3D([P(1,1);P(1,2);P(1,3);1], [P(2,1);P(2,2);P(2,3);1]);
                end
            elseif strcmp(obj.shape,'planar')
                for i = 1:obj.numberOfLines
                    P = obj.scale*rand(2,2)-obj.scale*0.5;
                    obj.linesIn3D(i) = LineIn3D([P(1,1);P(1,2);0;1],[P(2,1);P(2,2);0;1]);
                end
            else
                error('No matching shape. Currently implemented shapes: cubic, planar, spherical')
                return
            end
            for i = 1:obj.numberOfLines
                obj.linesIn3D(i).setMean(mean);
                obj.linesIn3D(i).setVariance(variance);
            end
        end % Constructor Linecloud3D end
        
        
        %> @brief
        %>
        %> @param this
        function plotTrueLinecloud(this)
            %------TODO----------------------------------------------------
        end
    end % methods end
end % classdef end