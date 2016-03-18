classdef Camera < handle 
    
		properties
            truePose	% true pose of the camera, 4x4 matrix
    		estimatedPose % pose estimated by 3D-2D point correspondencies, 4x4 matrix
			f	% focal length
			kx	% pixel per unit length in x-direction
			ky	% pixel per unit length in y-direction
			xResolution
			yResolution
			x0	% pixel x-coordinate of principle point
			y0	% pixel y-coordinate of principle point
			%K	% calibration matrix
			distortionModel	% [r;kappaVector]
			perspectiveNPointAlgorithm
					
			pointCloud3D@PointIn3D	% array of type PointIn3D, stores all 3d points
			pointCloud2D@PointIn2D	% array of type PointIn2D, stores all 2d points
		
		methods	
            function Position = Camera(Pointcloud)	% constructor, sets camera randomly around point cloud
					
            end
            
            function K = calculateCalibrationMatrix(f,kx,ky,x0,y0)
                K = [f*kx 0 x0; 0 f*ky y0; 0 0 1];
            end
            
            loadParameters(paramterFileDirectory)	% load parameters from xml file
			
            projectFrom3DTo2D()	% already considers distortion, input: pointCloud3D; output: pointCloud2D
			
            addPixelNoise()	% adds pixel noise to the 2D points
			
            calcutePoseEstimation()	% calculates the estimated pose with respective PnP algorithm stored in perspectiveNPointAlgorithm
end