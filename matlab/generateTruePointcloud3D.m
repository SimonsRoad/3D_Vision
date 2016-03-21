function truePointcloud3D = generateTruePointcloud3D(numberOfPoints,shape,scale)
% Input: Number of points, shape and scale
% Output: Randomly (uniformly) distributed points in requested shape

    % Spherical Pointcloud
    if strcmp(shape,'spherical')
        for i = 1:numberOfPoints
            % Use spherical coordinates to generate random points in a sphere
            % Algorithm from Matlab, cited is
            % [1] Knuth, D. The Art of Computer Programming. Vol. 2, 3rd ed. Reading, MA: Addison-Wesley Longman, 1998, pp. 134?136.
            rvals = 2*rand(1)-1;
            elevation = asin(rvals);
            azimuth = 2*pi*rand(1);
            radius = scale*rand(1).^(1/3);
            % Convert to Cartesian coordinates
            [x,y,z] = sph2cart(azimuth, elevation, radius);
            truePointcloud3D(i) = PointIn3D(x,y,z);
        end
        return 
    end
    
    % Qubic Pointcloud
    if strcmp(shape,'qubic')
        for i = 1:numberOfPoints
            P = scale*rand(3,1);
            truePointcloud3D(i) = PointIn3D(P(1),P(2),P(3));
        end
        return
    end
    
    % Planar Pointcloud (plane normal z)
    if strcmp(shape,'planar')
        for i = 1:numberOfPoints
            P = scale*rand(2,1);
            truePointcloud3D(i) = PointIn3D(P(1),P(2),0);
        end
        return
    end
    
    error('Invalid shape argument. Options are: spherical, qubic, planar ')
    
end