% generate 3D points

% initialize camera

% add noise to 3d points
% Note: We have to do this after we initialize the camera pose as we
% have to add the noise in the camera frame. This is because in a realistic
% setting, the uncertainty in depth (Z-direction of camera frame) than in
% the other two directions of the camera frame.

% project 3d to 2d points

% add pixel noise to 2d projection

% estimate camera pose with PnP algorithm