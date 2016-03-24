% =========================================================================
%> @brief Class Camera sets a camera in space and computes P from 3D-2D
%> point correspondences
%>
%> 
%>
% =========================================================================
classdef Camera < handle
   properties
       truePose
       estimatedPose
       f
       kx
       ky
       xResolution
       yResolution
       x0
       y0
       K
       distortionModel
       perspectiveNPointAlgorithm
       
       pointCloud3D@PointIn3D
       pointCloud2D@PointIn2D
   end
   
   methods
       %> @brief Constructor of class Camera
       %> 
       %> @param radius
       %>
       %> @retval obj
       function obj = Camera(radius)
           % generate random theta and random phi
           theta = 2*Pi*rand;
           phi = Pi*rand;
           
           % compute camera translation t with radius, theta and phi
           translation = [radius*sin(phi)*cos(theta); radius*sin(phi)*cos(theta); radius*cos(phi)];
           
           % point principal axis towards 
       end
       
   end
end