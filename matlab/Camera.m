%> @brief Camera Class sets a camera in space and computes estimated camera pose from 3D-2D point correspondences
classdef Camera < handle
   properties
       % Camera pose
       truePose                         %> @param truePose True pose of camera in world coordinates
       estimatedPose                    %> @param estimatedPose Initial estimated pose of camera in world coordinates
       optimizedEstimatedPose           %> @param optimizedEstimatedPose Non-linearly optimized initial estimation
       estimationConfidence             %> @param estimationConfidence The confidence of the estimation
       optimizedEstimatedPoseWithLines  %> @param optimizedEstimatedPoseWithLines Estimated Pose after nonlinear optimization with points and lines
       estimationConfidenceWithLines    %> @param estimationConfidenceWithLines The confidence of optimzied estimation with points and lines
       
       % Camera projection matrices
       trueCameraProjectionMatrix       %> @param trueCameraProjectionMatrix P = K[R t]
       
       % Camera parameters
       f                                %> @param f Focal length
       kx                               %> @param kx Pixel per unit length in x-direction
       ky                               %> @param ky Pixel per unit length in y-direction
       xResolution                      %> @param xResolution Pixel resolution in x-direction
       yResolution                      %> @param yResolution Pixel resolution in y-direction
       x0                               %> @param x0 Pixel x-coordinate of principle point
       y0                               %> @param y0 Pixel y-coordinate of principle point
       skew                             %> @param skew Skew paramater of camera sensor
       imagetoPixelCoordinatesTrafo     %> @param ImagetoPixelCoordinatesTrafo Matrix from u,y to x,y coordinates
       focalLenghtMatrix                %> @param focallengthMatrix Matrix [f 0 0; 0 f 0; 0 0 1]
       K                                %> @param K Calibration matrix
       distortionModel                  %> @param distortionModel String Distortion model
       kappa                            %> @param kappa Radial distortion parameters
       p                                %> @param p tangential distortion parameters
       
       % Pose estimation algorithm
       perspectiveNPointAlgorithm       %> @param perspectiveNPointAlgorithm String of algorithm used for pose estimation
       
       % Point clouds
       pointCloud3D@Pointcloud3D        %> @param pointCloud3D Member of type Pointcloud3D
       pointCloud2D@Pointcloud2D        %> @param pointCloud2D Member of type Pointcloud2D
       
       % Line clouds
       lineCloud3D@Linecloud3D          %> @param lineCloud3D Member of type Linecloud3D
       lineCloud2D@Linecloud2D          %> @param lineCloud2D Member of type Linecloud2D
       
       % PnP Algorithm
       pnpAlgorithm@PnPAlgorithm;       %> @param pnpAlgorithm Perspective N Point algorithm
   end % Properties end
   
   methods
       %> @brief Constructor of class Camera
       %> 
       %> @param radius Radius of sphere the camera is on
       %> @param azimutalAngle Azimutal angle on sphere
       %> @param polarAngle Polar angle on sphere
       %> @param f Focal length of camera
       %> @param x0 Principle point offset in x-direction of camera frame
       %> @param y0 Principle point offset in y-direction of camera frame
       %> @param kx Pixel per unit length in x-direction of camera frame
       %> @param ky Pixel per unit length in y-direction of camera frame
       %> @param skew Skew paramter of camera
       %>
       %> @retval obj Object of type Camera
       function obj = Camera(radius, azimutalAngle, polarAngle, f, x0, y0, kx, ky, skew, xResolution, yResolution)
           % Properties
           obj.f = f;
           obj.x0 = x0;
           obj.y0 = y0;
           obj.kx = kx;
           obj.ky = ky;
           obj.skew = skew;
           obj.xResolution = xResolution;
           obj.yResolution = yResolution;
           
           % Camera translation vector w.r.t. camera frame t = R_CI * C_I
           t = [0; 0; -radius];
           
           % Compute helper rotation matrix
           R_BI = [cos(azimutalAngle), sin(azimutalAngle), 0; % z-rotation at phi
               -sin(azimutalAngle), cos(azimutalAngle), 0;
               0, 0, 1];
           R_KB = [cos(-pi/2+polarAngle), 0, -sin(-pi/2+polarAngle); % y-rotation at -phi/2+theta
               0, 1, 0;
               sin(-pi/2+polarAngle), 0, cos(-pi/2+polarAngle)];
           R_CK = [0, 1, 0; % camera z-axis points towards origin
               0, 0, -1;
               -1, 0, 0];
           
           % Compute the camera rotation matrix
           R_CI = (R_CK*R_KB*R_BI);
           
           % Fill in the camera truePose 
           obj.truePose = zeros(3,4);
           obj.truePose(1:3,1:3) = R_CI;
           obj.truePose(1:3,4) = t;
           
           % Declare estimated Pose
           obj.estimatedPose(1:3,1:3) = eye(3);
           obj.estimatedPose(1:3,4) = zeros(1,3);
           
           % Calculate Transformation Matrix from image to pixel coordinates
           obj.calculateUVtoPixelMatrix();
           % Calculate focallength Matrix [f 0 0; 0 f 0; 0 0 1]
           obj.calculateFocallengthMatrix();
           
           % Calculate camera calibration matrix
           obj.calculateCalibrationMatrix();
           
           % Calculate camera projection matrix
           obj.trueCameraProjectionMatrix = obj.K*obj.truePose;
       end % Camera() end
       
       
       %> @brief Visualizes the true camera
       %>
       %> @param this Pointer of Camera object
       %> @param figureHandle Figure number
       %>
       %> @retval trueCam Handle to true camera pose plot
       function trueCam = visualizeTrueCamera(this, figureHandle)
           % Get true translation and rotation from truePose
           trueTranslation = this.truePose(1:3,4);
           trueRotation = this.truePose(1:3,1:3);
           
           % Plot the true pose
           figure(figureHandle)
           trueCam = plotCamera('Location',trueRotation' *trueTranslation,'Orientation',trueRotation,'Size',0.1,'Color',[0 0 1]);
           
           % Camera Centers
           truePosition = trueRotation' * trueTranslation;
           plot3(truePosition(1), truePosition(2), truePosition(3),'o','Color',[0 0 1],'DisplayName', 'True Camera Pose');
           
           % Camera frame of true pose
           P1 = truePosition;
           P2 = truePosition+trueRotation'*[1; 0; 0];
           P = [P1'; P2'];
           line(P(:,1), P(:,2), P(:,3), 'HandleVisibility','off')
           P2 = truePosition+trueRotation'*[0; 1; 0];
           P = [P1'; P2'];
           line(P(:,1), P(:,2), P(:,3), 'HandleVisibility','off')
           P2 = truePosition+trueRotation'*[0; 0; 1];
           P = [P1'; P2'];
           line(P(:,1), P(:,2), P(:,3), 'HandleVisibility','off')
       end % visualizeCamera() end
       
       
       %> @brief Visualizes the estimated camera
       %>
       %> @param this Pointer of Camera object
       %> @param figureHandle Figure number
       %>
       %> @retval estimatedCam Handle to estimated camera pose plot
       function estimatedCam = visualizeEstimatedCamera(this, figureHandle)
           % Get estimated translation and rotation from estimatedPose
           estimatedTranslation = this.estimatedPose(1:3,4);
           estimatedRotation = this.estimatedPose(1:3,1:3);
           
           % Plot the estimated pose
           figure(figureHandle)
           estimatedCam = plotCamera('Location',estimatedRotation' *estimatedTranslation,'Orientation',estimatedRotation,'Size',0.1,'Color',[1 0 0]);
           
           % Camera center
           estimatedPosition = estimatedRotation' * estimatedTranslation;
           plot3(estimatedPosition(1), estimatedPosition(2), estimatedPosition(3),'x','Color',[1 0 0],'DisplayName', 'Estimated Camera Pose');
           
           % Camera frame of estimated pose
           P1 = estimatedPosition;
           P2 = estimatedPosition+estimatedRotation'*[1; 0; 0];
           P = [P1'; P2'];
           line(P(:,1), P(:,2), P(:,3), 'color', 'red', 'HandleVisibility','off')
           P1 = estimatedPosition;
           P2 = estimatedPosition+estimatedRotation'*[0; 1; 0];
           P = [P1'; P2'];
           line(P(:,1), P(:,2), P(:,3), 'color', 'red', 'HandleVisibility','off')
           P1 = estimatedPosition;
           P2 = estimatedPosition+estimatedRotation'*[0; 0; 1];
           P = [P1'; P2'];
           line(P(:,1), P(:,2), P(:,3), 'color', 'red', 'HandleVisibility','off')
       end % visualizeEstimatedCamera() end
       
       
       %> @brief Visualizes the optimized estimated camera
       %>
       %> @param this Pointer of Camera object
       %> @param figureHandle Figure number
       %>
       %> @retval optimizedCam Handle to optimized estimated camera pose plot
       function optimizedCam = visualizeOptimizedCamera(this, figureHandle)
           % Get estimated translation and rotation from estimatedPose
           optimizedTranslation = this.optimizedEstimatedPose(1:3,4);
           optimizedRotation = this.optimizedEstimatedPose(1:3,1:3);
           
           % Plot the estimated pose
           figure(figureHandle)
           optimizedCam = plotCamera('Location',optimizedRotation' *optimizedTranslation,'Orientation',optimizedRotation,'Size',0.1,'Color',[0 1 0]);
           
           % Camera center
           optimizedPosition = optimizedRotation' * optimizedTranslation;
           plot3(optimizedPosition(1), optimizedPosition(2), optimizedPosition(3),'x','Color',[0 1 0],'DisplayName', 'Optimized Estimated Camera Pose');
           
           % Camera frame of estimated pose
           P1 = optimizedPosition;
           P2 = optimizedPosition+optimizedRotation'*[1; 0; 0];
           P = [P1'; P2'];
           line(P(:,1), P(:,2), P(:,3), 'Color', [0 1 0], 'HandleVisibility','off')
           P1 = optimizedPosition;
           P2 = optimizedPosition+optimizedRotation'*[0; 1; 0];
           P = [P1'; P2'];
           line(P(:,1), P(:,2), P(:,3), 'Color', [0 1 0], 'HandleVisibility','off')
           P1 = optimizedPosition;
           P2 = optimizedPosition+optimizedRotation'*[0; 0; 1];
           P = [P1'; P2'];
           line(P(:,1), P(:,2), P(:,3), 'Color', [0 1 0], 'HandleVisibility','off')
       end % visualizeOptimizedCamera() end
       
       
       %> @brief Visualizes the estimated camera optimized with lines
       %>
       %> @param this Pointer of Camera object
       %> @param figureHandle Figure number
       %>
       %> @retval optimizedCamWithLines Handle to optimized estimated camera pose plot
       function optimizedCamWithLines = visualizeOptimizedCameraWithLines(this, figureHandle)
           % Get estimated translation and rotation from estimatedPose
           optimizedTranslation = this.optimizedEstimatedPoseWithLines(1:3,4);
           optimizedRotation = this.optimizedEstimatedPoseWithLines(1:3,1:3);
           
           % Plot the estimated pose
           figure(figureHandle)
           optimizedCamWithLines = plotCamera('Location',optimizedRotation' *optimizedTranslation,'Orientation',optimizedRotation,'Size',0.1,'Color',[0 1 1]);
           
           % Camera center
           optimizedPosition = optimizedRotation' * optimizedTranslation;
           plot3(optimizedPosition(1), optimizedPosition(2), optimizedPosition(3),'x','Color',[0 1 1],'DisplayName', 'Optimized Estimated Camera Pose with Lines');
           
           % Camera frame of estimated pose
           P1 = optimizedPosition;
           P2 = optimizedPosition+optimizedRotation'*[1; 0; 0];
           P = [P1'; P2'];
           line(P(:,1), P(:,2), P(:,3), 'Color', [0 1 1], 'HandleVisibility','off')
           P1 = optimizedPosition;
           P2 = optimizedPosition+optimizedRotation'*[0; 1; 0];
           P = [P1'; P2'];
           line(P(:,1), P(:,2), P(:,3), 'Color', [0 1 1], 'HandleVisibility','off')
           P1 = optimizedPosition;
           P2 = optimizedPosition+optimizedRotation'*[0; 0; 1];
           P = [P1'; P2'];
           line(P(:,1), P(:,2), P(:,3), 'Color', [0 1 1], 'HandleVisibility','off')
       end % visualizeOptimizedCameraWithLines() end
       
       
       %> @brief Projects a pointcloud in 3D into a pointcloud in 2D
       %>
       %> @param this Pointer to Camera object
       function projectPointsFrom3DTo2D(this)
           % Project noisy 3D points to 2D pixel space
           pointcloud2D = ProjectionFrom3Dto2D(this.pointCloud3D, this.focalLenghtMatrix);
           this.pointCloud2D = Pointcloud2D(pointcloud2D);
       end % projectPointsFrom3DTo2D() end
       
       
       %> @brief Add distortion to u,v coordinates
       %>
       %> @param this Pointer to object
       %> @param kappa Radial distortion parameters 3-dimensional vector
       %> @param p Tangential distortion parameters 2-dimensional vector
       function addDistortion(this, kappa, p)
           this.pointCloud2D.addDistortion(kappa, p);
       end % addDistortion() end
       
       
       %> @brief Calculate the homogeneous distorted points in uv coordinates
       %>
       %> @param this Pointer to object (this.imagetoPixelCoordinatesTrafo) is needed
       function calculateHomoegenousDistortedPixelPoints(this)
           this.pointCloud2D.calculateHomoegenousDistortedPixelPoints(this.imagetoPixelCoordinatesTrafo);
       end % calculateHomoegenousDistortedPixelPoints() end
       
       
       %> @brief Calculate the euclidean distorted points in uv coordinates (given homogeneous distorted pixel points)
       %>
       %> @param this Pointer to object
       function setDistortedPixelCoordinatesFromHomogeneousCoordinates(this)
          this.pointCloud2D.setDistortedPixelCoordinatesFromHomogeneousCoordinates(); 
       end % setDistortedPixelCoordinatesFromHomogeneousCoordinates() end
       
       
       %> @brief Adds pixel noise to all points
       %>
       %> @param noiseType String type of noise. Options are 'noNoise', 'uniformly', 'gaussian'
       %> @param this Pointer to Camera object
       %> @param mean Mean of gaussian distribution in x- and y-direction
       %> @param variance Variance of gaussian distribution in x- and y-direction
       function addPixelNoise(this, noiseType, mean, variance)
           if strcmp(noiseType,'noNoise')
               return
           elseif (strcmp(noiseType,'uniformly') || strcmp(noiseType,'gaussian'))
               this.pointCloud2D.addPixelNoise(noiseType, mean, variance);
           else
               error('addPixelNoise() must be called with the options: noNoise, uniformly, gaussian');
           end
       end % addPixelNoise() end
       
       
       %> @brief Transform from xy (pixel) to uv (image) coordinates
       %>
       %> @param this Pointer to Camera object
       function transformFromPixelToImage(this)
           this.pointCloud2D.transformFromPixelToImage(this.imagetoPixelCoordinatesTrafo); 
       end % transformFromPixelToImage() end
       
       
       %> @brief Undistorts the backprojected points from pixel to image plane
       %>
       %> @param this Pointer to this object
       function undistortion(this)
           this.pointCloud2D.undistortPointCloud2D();
       end % undistortion() end
       
       
       %> @brief Projects 3D lines into the image plane
       %>
       %> @param this Pointer to this object
       function projectLinesFrom3DTo2D(this)
           this.lineCloud2D = Linecloud2D(this.lineCloud3D, this.focalLenghtMatrix);
       end % projectLinesFrom3DTo3D() end
       
       
       %> @brief Samples lines
       %>
       %> @param this Pointer to this object
       %> @param numberOfSamples
       function sampleLines(this, numberOfSamples)
          this.lineCloud2D.samplingLines(numberOfSamples); 
       end % sampleLines() end
       
       
       %> @brief
       %>
       %> @param this Pointer to this object
       %> @param kappa
       %> @param p
       %> @param noiseType
       %> @mean
       %> @variance
       function measuerementProcess(this, kappa, p, noiseType, mean, variance)
          this.lineCloud2D.measurementProcessing(kappa, p, this.imagetoPixelCoordinatesTrafo, noiseType, mean, variance); 
       end % measuerementProcess() end
       
       
       %> @brief Calculates the transformation Matrix from UV to XY (pixel coordinates) [kx s x0; 0 ky y0; 0 0 1]
       %> 
       %> @param this Pointer to object
       function calculateUVtoPixelMatrix(this)
           this.imagetoPixelCoordinatesTrafo = [this.kx, this.skew, this.x0;
                                                      0,   this.ky, this.y0;
                                                      0,         0,       1];
       end % calculateUVtoPixelMatrix() end
       
       
       %> @brief Calculates the focallength matrix diag([f,f,1])
       %> 
       %> @param this Pointer to object
       function calculateFocallengthMatrix(this)
           this.focalLenghtMatrix = [this.f, 0, 0; 0, this.f, 0; 0, 0, 1];
       end % calculateFocallengthMatrix() end
       
       
       %> @brief Calculates the camera calibration matrix
       %>
       %> @param this Pointer to object
       function calculateCalibrationMatrix(this)
           % Fill in the calibration matrix
           this.K = this.imagetoPixelCoordinatesTrafo * this.focalLenghtMatrix;
       end % calculateCalibrationMatrix() end
       
       
       %> @brief Estimate the camera pose with a pnp algorithm
       %>
       %> @param this Pointer to this object
       function estimatePose(this)
           [R,t] = this.pnpAlgorithm.estimatePose([this.f 0 0; 0 this.f 0; 0 0 1]);
           this.estimatedPose(1:3,1:3) = R;
           t(3) = -t(3);
           this.estimatedPose(1:3,4) = t;
       end % estimatePose() end
       
       
       %> @brief Optimize the estimated camera pose using points
       %>
       %> @param this Pointer to this object
       function optimizePoseEstimation(this)
           [this.optimizedEstimatedPose, this.estimationConfidence] = nonlinearOptimization(this.estimatedPose,this.pointCloud3D,this.pointCloud2D,this.f);
       end % optimizePoseEstimation() end
       
       
       %> @brief Optimize the estimated camera pose using points and lines
       %>
       %> @param this Pointer to this object
       function optimizePoseEstimationWithLines(this)
           [this.optimizedEstimatedPoseWithLines, this.estimationConfidenceWithLines] = nonlinearOptimizationWithLines(this.estimatedPose, this.pointCloud3D, this.pointCloud2D, this.lineCloud3D, this.lineCloud2D, this.f);
       end % optimizePoseEstimationWithLines() end
       
       
       %> @brief Calculate the error in the pose estimation
       %>
       %> @retval errorInTranslation This is the error in camera translation in percent
       %> @retval errorInOrientation This is the error in orientation. The error is calculated as the sum of the acos of the scalar products of the unit vectors of the coordinate frames (todo: come up with a better way to describe this)
       function [errorInTranslation, errorInOrientation] = computePoseError(this,pose)
           xTrue = this.truePose(:,1);
           yTrue = this.truePose(:,2);
           zTrue = this.truePose(:,3);
           xEstimated = pose(:,1);
           yEstimated = pose(:,2);
           zEstimated = pose(:,3);
           scalarProducts = [xTrue'*xEstimated yTrue'*yEstimated zTrue'*zEstimated];
           % If *True = *Estimated their scalar product should be 1 as they
           % are normalized vectors. (The acos of 1 is 0)
           errorInOrientation = sum(abs(acos(scalarProducts)))*180/pi;
           trueTranslation = this.truePose(:,4);
           estimatedTranslation = pose(:,4);
           errorInTranslation = norm(trueTranslation-estimatedTranslation)/norm(trueTranslation)*100;
       end % computePoseError() end
       
       
       %> @brief Returns camera calibration matrix
       %> 
       %> @param this Pointer to Camera object
       %>
       %> @retval K calibration matrix this
       function K = getCalibrationMatrix(this)
           K = this.K;
       end % getCalibrationMatrix() end
       
       
       %> @brief Sets the PnP Algorithm
       %>
       %> @param algorithm Name of the PnP Algorithm
       function setPnPAlgorithm(this,algorithm_)
           this.pnpAlgorithm = PnPAlgorithm(this.pointCloud3D, this.pointCloud2D, algorithm_);
       end % setPnPAlgorithm() end
       
       
       %> @brief Gets true and estimated pose of a Camera object
       %>
       %> @param this Pointer to Camera object
       %>
       %> @retval truePose True pose of Camera object
       %> @retval estimatedPose Estimated pose of Camera object
       function [truePose, estimatedPose] = getPose(this)
          truePose = this.truePose;
          estimatedPose = this.estimatedPose;
       end % getPose() end
       
       
       %> @brief Plots the confidence interval of each point
       %>
       %> @param this Pointer to this object
       function plotConfidenceIntervals(this)
           for i = 1:this.pointCloud3D.numberOfPoints
               [x, y, z] = ellipsoid(this.pointCloud3D.pointsIn3D(i).trueCoordinatesInWorldFrame(1),...
                   this.pointCloud3D.pointsIn3D(i).trueCoordinatesInWorldFrame(2),...
                   this.pointCloud3D.pointsIn3D(i).trueCoordinatesInWorldFrame(3),...
                                       2*this.pointCloud3D.pointsIn3D(i).anisotropicGaussianVariance(1),...
                                       2*this.pointCloud3D.pointsIn3D(i).anisotropicGaussianVariance(2),...
                                       2*this.pointCloud3D.pointsIn3D(i).anisotropicGaussianVariance(3));
               r = vrrotmat2vec(this.truePose(:,1:3)');
               direction = r(1:3);
               theta = r(4)*180/pi;
               origin = [this.pointCloud3D.pointsIn3D(i).trueCoordinatesInWorldFrame(1) this.pointCloud3D.pointsIn3D(i).trueCoordinatesInWorldFrame(2) this.pointCloud3D.pointsIn3D(i).trueCoordinatesInWorldFrame(3)];
               ellipse = surf(x,y,z,'EdgeColor','none','FaceColor','red');
               alpha(0.1)
               rotate(ellipse,direction,theta,origin);
                drawnow
               hold on
           end
       end % plotConfidenceIntervals() end
   end % methods end
end % classdef end