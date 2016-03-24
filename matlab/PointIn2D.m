classdef PointIn2D < handle
    
    % Properties
    
    properties
        
        % Coordinates
        projectedCoordinates2D;
        noisyCoordinates2D;
        id;
        noiseModel;
        
        % Flags
        
    end
    
    % Methods
    
    methods
        
        % constructor: input a noisy 3D point and a fully calibrated Camera, constructs directly the
        % 2D correspondence
        function obj = PointIn2D(noisyPoint3D, Camera)
            obj.projectedCoordinates2D = calculateProjectedCoordinates(noisyPoint3D, Camera);
            % obj.noisyCoordinates2D = addNoiseToPoint2D();
            % obj.id =
            % obj.noiseModel =
        end
        
        % input a 2D point, output Coordinates of this 2D point
        function noisyPoint2D = getNoisyCoordinates()
            noisyPoint2D = this.noisyCoordinates2D();
        end
        
        % calculate the corresponding 2D coordinates of a noisy 3D point
        % p_A = K * R_AI * (P_I - C_I)
        % A : Camera frame
        % I : Inertial / World frame
        % p_A : pixel coordinates in Camera Frame
        % K : calibration matrix
        % R_AI : Rotation form Inertial to Camera Frame
        % P_I : 3D noisy point in Inertial Frame
        % C_I : Camera position in Inertial Frame
        % t_A : R_AI * C_I is already in the Transformation matrix
        function projectedPoint2D = calculateProjectedCoordinates(noisyPoint3D_I, Camera)
            K = Camera.calculateCalibrationMatrix();
            T_IA = Camera.getTransformationMatrix();
            R_IA = T_IA(1:3,1:3);
            R_AI = R_IA';
            t_A = T_IA(:,1:3); 
            this.projectedCoordinates2D = K * (R_AI * noisyPoint3D_I - t_A);
            projectedPoint2D = this.projectedCoordinates2D;
        end
        
        function point2Ddistorted = addDistortion(Point2D, k, Camera)
           K = Camera.calculateCalibrationMatrix();
           centerOfDistortion = [K(1,3); K(2,3)];
           radius = sqrt((Point2D(1,1) - centerOfDistortion(1,1))^2 + (Point2D(2,1) - centerOfDistortion(2,1))^2);
           point2Ddistorted = (1 + k(1) * radius^2 + k(2) * radius^4 + k(3) * radius^6) * Point2D;
        end
        % loadParameters(paramterFileDirectory)
        
    end
    
end