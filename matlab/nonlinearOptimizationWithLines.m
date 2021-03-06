%> @brief Optimizes camera pose estimate with initial guess using 3D-2D point and line correspondences
%>
%> @param initialEstimation Initial Pose Estimate for optimization algorithm
%> @param pointCloud3D A 3D pointcloud storing 3D points
%> @param pointCloud2D A 2D pointcloud storing 2D points
%> @param focalLength Focal length of the camera lense
%>
%> @retval optimizedEstimatedPose Optimized pose estimate of a camera
%> @retval confidenceMatrix
function [optimizedEstimatedPose, confidenceMatrix] = nonlinearOptimizationWithLines(initialEstimation, pointCloud3D, pointCloud2D, lineCloud3D, lineCloud2D, focalLength)

% Set parameters
TOL = 10e-3;
MAX_ITER = 1000;

% Define helper variables
nrOfPoints = pointCloud3D.numberOfPoints;
nrOfLines = lineCloud3D.numberOfLines;
f = focalLength;

% Convert the 3D pointcloud to a 3Nx1 matrix
matrixOf3DPoints = zeros(3*nrOfPoints,1);
for i = 1:nrOfPoints
    matrixOf3DPoints(3*(i-1)+1:3*(i-1)+3) = pointCloud3D.pointsIn3D(i).noisyCoordinatesInWorldFrame;
end

% Convert the 2D pointcloud to a 2Nx1 matrix
matrixOf2DPoints = zeros(2*nrOfPoints,1);
for i = 1:nrOfPoints
    matrixOf2DPoints(2*(i-1)+1:2*(i-1)+2) = pointCloud2D.pointsIn2D(i).backProjectionFromPixelToImageCoordinates;
end

% Convert the 3D linecloud to a 3Nx2 matrix (1st column: start points, 2nd
% column: end points)
matrixOf3DLines = zeros(3*nrOfLines,1);
for i = 1:nrOfLines
    matrixOf3DLines(3*(i-1)+1:3*(i-1)+3, 1) = lineCloud3D.linesIn3D(i).startPoint.noisyCoordinatesInWorldFrame;
    matrixOf3DLines(3*(i-1)+1:3*(i-1)+3, 2) = lineCloud3D.linesIn3D(i).endPoint.noisyCoordinatesInWorldFrame;
end

% Convert the 2D linecloud to a 2Nx2 matrix (1st column: start points, 2nd
% column: end points)
matrixOf2DLines = zeros(2*nrOfLines,2);
for i = 1:nrOfLines
    matrixOf2DLines(2*(i-1)+1:2*(i-1)+2, 1) = lineCloud2D.linesIn2D(i).startPointIPNoisy;
    matrixOf2DLines(2*(i-1)+1:2*(i-1)+2, 2) = lineCloud2D.linesIn2D(i).endPointIPNoisy;
end

% Convert initial estimation
x0 = initialEstimation(1,4);
y0 = initialEstimation(2,4);
z0 = initialEstimation(3,4);
rotation = initialEstimation(:,1:3);

% http://www.staff.city.ac.uk/~sbbh653/publications/euler.pdf
if (rotation(3,1) ~= 1 && rotation(3,1) ~= -1)
    beta1 = -asin(rotation(3,1));
    beta2 = pi-beta1;
    alpha1 = atan2(rotation(3,2)/cos(beta1), rotation(3,3)/cos(beta1));
    alpha2 = atan2(rotation(3,2)/cos(beta2), rotation(3,3)/cos(beta2));
    gamma1 = atan2(rotation(2,1)/cos(beta1), rotation(1,1)/cos(beta1));
    gamma2 = atan2(rotation(2,1)/cos(beta2), rotation(1,1)/cos(beta2));
    matrixOfReprojectedLines1 = reprojectLines(x0,y0,z0,alpha1,beta1,gamma1,lineCloud3D,f);
    errorPose1 = computeLineReprojectionError(matrixOf2DLines, matrixOfReprojectedLines1);
    matrixOfReprojectedLines2 = reprojectLines(x0,y0,z0,alpha2,beta2,gamma2,lineCloud3D,f);
    errorPose2 = computeLineReprojectionError(matrixOf2DLines, matrixOfReprojectedLines2);
    index = min(norm(errorPose1),norm(errorPose2));
    if (index == 1)
        alpha0 = alpha1;
        beta0 = beta1;
        gamma0 = gamma1;
    else
        alpha0 = alpha2;
        beta0 = beta2;
        gamma0 = gamma2;
    end
else
    gamma0 = 0;
    if (rotation(3,1) == -1)
        beta0 = pi/2;
        alpha0 = gamma0 + atan2(rotation(1,2), rotation(1,3));
    else
        beta0 = -pi/2;
        alpha0 = -gamma0 + atan2(-rotation(1,2), -rotation(1,3));
    end
end

pose = [x0; y0; z0; alpha0; beta0; gamma0];

matrixOfReprojectedPoints = reprojectPoints(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6),pointCloud3D,f);
matrixOfPointReprojectionError = matrixOfReprojectedPoints - matrixOf2DPoints;
disp(['Norm of reprojection error of initial estimation: ' num2str(norm(matrixOfPointReprojectionError))])

figure(2)

updateStep = Inf;
iter = 1;
while( norm(updateStep) > TOL && iter <= MAX_ITER )
    
    disp(['Iteration: ' num2str(iter)])
    
    %% Points
    % Calculate Jacobian (J_p(x) \in \R^{2N x 6})
    J_p = jacobian_p(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6),pointCloud3D,f);
    
    % Calculate Reprojection (z(x) \in \R^{2N x 1})
    matrixOfReprojectedPoints = reprojectPoints(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6),pointCloud3D,f);
    
    % Calculate Reprojection Error (Delta z = z(x) - z \in \R^{2N x 1})
    matrixOfPointReprojectionError = matrixOfReprojectedPoints - matrixOf2DPoints;
    
    disp('Mean of point reprojection error')
    disp(mean(matrixOfPointReprojectionError))
    
    %% Lines
    % Calculate the Jacobian (J_l(x) \in \R^{2N x 6})
    J_l = jacobian_l(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6),lineCloud3D,lineCloud2D,f);
    
    % Calculate the Reprojection of the lines
    matrixOfReprojectedLines = reprojectLines(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6),lineCloud3D,f);
    
    % Calculate the Reprojection Error of the lines
    matrixOfLineReprojectionError = computeLineReprojectionError(matrixOf2DLines, matrixOfReprojectedLines);
    
    disp('Mean of line reprojection error')
    disp(mean(matrixOfLineReprojectionError))
    
    % Calculate gradient
    g = matrixOfPointReprojectionError'*J_p + (pointCloud3D.numberOfPoints)/(lineCloud3D.numberOfLines)*matrixOfLineReprojectionError'*J_l;
    
    % Calculate approximation of Hessian
    H = J_p'*J_p + J_l'*J_l;
    
    % Calculate the update step
    updateStep = -pinv(H)*g';
    
    % Update the pose
    pose = pose+0.5*updateStep;
    
    % Update iteration counter
    iter = iter + 1;
    
    %% Debug: Plot reprojected lines for each iteration
    hold on
    hgrey = zeros(lineCloud2D.numberOfLines,1);
    for i = 1:lineCloud2D.numberOfLines
        X = [matrixOfReprojectedLines(2*(i-1)+1,1), matrixOfReprojectedLines(2*(i-1)+1,2)];
        Y = [matrixOfReprojectedLines(2*(i-1)+2,1), matrixOfReprojectedLines(2*(i-1)+2,2)];
        hgrey(i) = plot(X, Y, 'Color', [0.9, 0.9, 0.9],'DisplayName', 'Iteration Step Line');
    end
end

hold on
matrixOfInitialReprojection = reprojectLines(x0, y0, z0, alpha0, beta0, gamma0,lineCloud3D,f);
hred = zeros(lineCloud2D.numberOfLines,1);
hblue = zeros(lineCloud2D.numberOfLines,1);
hcyan = zeros(lineCloud2D.numberOfLines,1);
for i = 1:lineCloud2D.numberOfLines
    X = [matrixOfInitialReprojection(2*(i-1)+1,1), matrixOfInitialReprojection(2*(i-1)+1,2)];
    Y = [matrixOfInitialReprojection(2*(i-1)+2,1), matrixOfInitialReprojection(2*(i-1)+2,2)];
    hred(i) = plot(X, Y, 'Color', 'red','DisplayName', 'Initial Position Line');
end
for i = 1:lineCloud2D.numberOfLines
    X = [matrixOf2DLines(2*(i-1)+1,1), matrixOf2DLines(2*(i-1)+1,2)];
    Y = [matrixOf2DLines(2*(i-1)+2,1), matrixOf2DLines(2*(i-1)+2,2)];
    hblue(i) = plot(X, Y, 'Color', 'blue','DisplayName', 'True Position Line');
end
matrixOfReprojectedLines = reprojectLines(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6),lineCloud3D,f);
for i = 1:lineCloud2D.numberOfLines
    X = [matrixOfReprojectedLines(2*(i-1)+1,1), matrixOfReprojectedLines(2*(i-1)+1,2)];
    Y = [matrixOfReprojectedLines(2*(i-1)+2,1), matrixOfReprojectedLines(2*(i-1)+2,2)];
    hcyan(i) = plot(X, Y, 'Color', 'cyan','DisplayName', 'Optimized Position Line');
end
legend([hgrey(1) hred(1) hblue(1) hcyan(1)],{'Iteration Step Line','Initial Position Line','True Position Line','Optimized Position Line'});
title('Iteration Steps of Optimization')
hold off

if (iter > MAX_ITER)
    disp(['Nonlinear optimization did not converge in ' num2str(MAX_ITER) ' iterations!']);
else
    disp(['Nonlinear optimization converged in ' num2str(iter-1) ' iterations']);
end

disp(['Norm of reprojection error of optimized estimation: ' num2str(norm(matrixOfPointReprojectionError))])

R = rotz(pose(6)*180/pi)*roty(pose(5)*180/pi)*rotx(pose(4)*180/pi);
t = [pose(1); pose(2); pose(3)];

optimizedEstimatedPose = [R t];
confidenceMatrix = inv(J_p'*J_p);
end


%> @brief Comptutes the Jacobian of the point reprojection error w.r.t. the camera pose estimate
%>
%> @param x 1st coordinate of camera position, w.r.t. the world frame
%> @param y 2nd coordinate of camera position, w.r.t. the world frame
%> @param z 3rd coordinate of camera position, w.r.t. the world frame
%> @param alpha Euler angle rotation in x-direction, w.r.t. the world frame
%> @param beta Euler angle rotation in y-direction, w.r.t. the world frame
%> @param gamma Euler angle rotation in z-direction, w.r.t. the world frame
%> @param pointCloud3D A 3D pointcloud storing all 3D points
%> @param f Focal length of the camera lense
%>
%> @retval J_p Jacobian of the reprojection error
function J_p = jacobian_p(x,y,z,alpha,beta,gamma,pointCloud3D,f)
    J_p = zeros(2*pointCloud3D.numberOfPoints, 6);
    for j = 1:pointCloud3D.numberOfPoints
        z1j = pointCloud3D.pointsIn3D(j).noisyCoordinatesInWorldFrame(1);
        z2j = pointCloud3D.pointsIn3D(j).noisyCoordinatesInWorldFrame(2);
        z3j = pointCloud3D.pointsIn3D(j).noisyCoordinatesInWorldFrame(3);
        
        J_pj = [   [ f/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)),                                                                           0, -(f*(x - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2,   (f*(z2j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z3j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta))))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) - (f*(z2j*cos(alpha)*cos(beta) - z3j*cos(beta)*sin(alpha))*(x - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, (f*(z3j*cos(alpha)*cos(beta)*cos(gamma) - z1j*cos(gamma)*sin(beta) + z2j*cos(beta)*cos(gamma)*sin(alpha)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) + (f*(z1j*cos(beta) + z3j*cos(alpha)*sin(beta) + z2j*sin(alpha)*sin(beta))*(x - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, -(f*(z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))]
                   [                                                                           0, f/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)), -(f*(y + z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, - (f*(z2j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z3j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma))))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) - (f*(z2j*cos(alpha)*cos(beta) - z3j*cos(beta)*sin(alpha))*(y + z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, (f*(z3j*cos(alpha)*cos(beta)*sin(gamma) - z1j*sin(beta)*sin(gamma) + z2j*cos(beta)*sin(alpha)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) + (f*(z1j*cos(beta) + z3j*cos(alpha)*sin(beta) + z2j*sin(alpha)*sin(beta))*(y + z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2,  (f*(z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))]];
        J_p(2*(j-1)+1:2*(j-1)+2,:) = J_pj;
    end
end


%> @brief Comptutes the Jacobian of the line reprojection error w.r.t. the camera pose estimate
%>
%> @param x 1st coordinate of camera position, w.r.t. the world frame
%> @param y 2nd coordinate of camera position, w.r.t. the world frame
%> @param z 3rd coordinate of camera position, w.r.t. the world frame
%> @param alpha Euler angle rotation in x-direction, w.r.t. the world frame
%> @param beta Euler angle rotation in y-direction, w.r.t. the world frame
%> @param gamma Euler angle rotation in z-direction, w.r.t. the world frame
%> @param linecloud3D A 3D linecloud storing all 3D lines
%> @param linecloud2D A 2D linecloud storing all 2D lines
%> @param f Focal length of the camera lense
%>
%> @retval J_l Jacobian of the reprojection error
function J_l = jacobian_l(x,y,z,alpha,beta,gamma,lineCloud3D,lineCloud2D,f)
    J_l = zeros(lineCloud3D.numberOfLines, 6);
    for j = 1:lineCloud3D.numberOfLines
        z1j = lineCloud3D.linesIn3D(j).startPoint.noisyCoordinatesInWorldFrame(1);
        z2j = lineCloud3D.linesIn3D(j).startPoint.noisyCoordinatesInWorldFrame(2);
        z3j = lineCloud3D.linesIn3D(j).startPoint.noisyCoordinatesInWorldFrame(3);

        dr1_dphi = [   [ f/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)),                                                                           0, -(f*(x - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2,   (f*(z2j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z3j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta))))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) - (f*(z2j*cos(alpha)*cos(beta) - z3j*cos(beta)*sin(alpha))*(x - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, (f*(z3j*cos(alpha)*cos(beta)*cos(gamma) - z1j*cos(gamma)*sin(beta) + z2j*cos(beta)*cos(gamma)*sin(alpha)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) + (f*(z1j*cos(beta) + z3j*cos(alpha)*sin(beta) + z2j*sin(alpha)*sin(beta))*(x - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, -(f*(z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))]
                       [                                                                           0, f/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)), -(f*(y + z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, - (f*(z2j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z3j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma))))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) - (f*(z2j*cos(alpha)*cos(beta) - z3j*cos(beta)*sin(alpha))*(y + z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, (f*(z3j*cos(alpha)*cos(beta)*sin(gamma) - z1j*sin(beta)*sin(gamma) + z2j*cos(beta)*sin(alpha)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) + (f*(z1j*cos(beta) + z3j*cos(alpha)*sin(beta) + z2j*sin(alpha)*sin(beta))*(y + z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2,  (f*(z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))]];

        z1j = lineCloud3D.linesIn3D(j).endPoint.noisyCoordinatesInWorldFrame(1);
        z2j = lineCloud3D.linesIn3D(j).endPoint.noisyCoordinatesInWorldFrame(2);
        z3j = lineCloud3D.linesIn3D(j).endPoint.noisyCoordinatesInWorldFrame(3);

        dr2_dphi = [   [ f/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)),                                                                           0, -(f*(x - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2,   (f*(z2j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z3j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta))))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) - (f*(z2j*cos(alpha)*cos(beta) - z3j*cos(beta)*sin(alpha))*(x - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, (f*(z3j*cos(alpha)*cos(beta)*cos(gamma) - z1j*cos(gamma)*sin(beta) + z2j*cos(beta)*cos(gamma)*sin(alpha)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) + (f*(z1j*cos(beta) + z3j*cos(alpha)*sin(beta) + z2j*sin(alpha)*sin(beta))*(x - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, -(f*(z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))]
                       [                                                                           0, f/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)), -(f*(y + z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, - (f*(z2j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z3j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma))))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) - (f*(z2j*cos(alpha)*cos(beta) - z3j*cos(beta)*sin(alpha))*(y + z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, (f*(z3j*cos(alpha)*cos(beta)*sin(gamma) - z1j*sin(beta)*sin(gamma) + z2j*cos(beta)*sin(alpha)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) + (f*(z1j*cos(beta) + z3j*cos(alpha)*sin(beta) + z2j*sin(alpha)*sin(beta))*(y + z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2,  (f*(z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))]];

        reprojectedLines = reprojectLines(x,y,z,alpha,beta,gamma,lineCloud3D,f);

        % Helper coordinates
        %   r1x : x coordinate of reprojected start point
        %   r1y : y coordinate of reprojected start point
        %   r2x : x coordinate of reprojected end point
        %   r2y : y coordinate of reprojected end point
        %   o1x : x coordinate of observed start point
        %   o1y : y coordinate of observed start point
        %   o2x : x coordinate of observed end point
        %   o2y : y coordinate of observed end point
        %   l   : length of observed line
        r1x = reprojectedLines(1,1);
        r1y = reprojectedLines(1,2);
        r2x = reprojectedLines(2,1);
        r2y = reprojectedLines(2,2);
        o1x = lineCloud2D.linesIn2D(j).startPointIPNoisy(1);
        o1y = lineCloud2D.linesIn2D(j).startPointIPNoisy(2);
        o2x = lineCloud2D.linesIn2D(j).endPointIPNoisy(1);
        o2y = lineCloud2D.linesIn2D(j).endPointIPNoisy(2);
        o1xp = o1x - o2y + o1y;
        o1yp = o1y + o2x - o1x;
        o2xp = o2x - o2y + o1y;
        o2yp = o2y + o2x - o1x;
        l = sqrt((o1x - o2x)^2 + (o1y - o2y)^2);

        H1x = ((o1x*o1yp - o1y*o1xp)*(r1x - r2x) - (o1x - o1xp)*(r1x*r2y - r1y*r2x))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x));
        H1y = ((o1x*o1yp - o1y*o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x*r2y - r1y*r2x))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x));
        H2x = ((o2x*o2yp - o2y*o2xp)*(r1x - r2x) - (o2x - o2xp)*(r1x*r2y - r1y*r2x))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x));
        H2y = ((o2x*o2yp - o2y*o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x*r2y - r1y*r2x))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x));
        
        h1 = sqrt((o1x - H1x)^2 + (o1y - H1y)^2);
        h2 = sqrt((o2x - H2x)^2 + (o2y - H2y)^2);
        
        % Evaluate Derivatives
        dH1_dr1 = [   [ (((r1x - r2x)*(o1x*o1yp - o1xp*o1y) - (o1x - o1xp)*(r1x*r2y - r2x*r1y))*(o1y - o1yp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x))^2 - (o1xp*o1y - o1x*o1yp + r2y*(o1x - o1xp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x)),                       (r2x*(o1x - o1xp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x)) - (((r1x - r2x)*(o1x*o1yp - o1xp*o1y) - (o1x - o1xp)*(r1x*r2y - r2x*r1y))*(o1x - o1xp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x))^2];...
                      [                       (((r1y - r2y)*(o1x*o1yp - o1xp*o1y) - (o1y - o1yp)*(r1x*r2y - r2x*r1y))*(o1y - o1yp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x))^2 - (r2y*(o1y - o1yp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x)), (o1x*o1yp - o1xp*o1y + r2x*(o1y - o1yp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x)) - (((r1y - r2y)*(o1x*o1yp - o1xp*o1y) - (o1y - o1yp)*(r1x*r2y - r2x*r1y))*(o1x - o1xp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x))^2]];

        dH1_dr2 = [   [ (o1xp*o1y - o1x*o1yp + r1y*(o1x - o1xp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x)) - (((r1x - r2x)*(o1x*o1yp - o1xp*o1y) - (o1x - o1xp)*(r1x*r2y - r2x*r1y))*(o1y - o1yp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x))^2,                       (((r1x - r2x)*(o1x*o1yp - o1xp*o1y) - (o1x - o1xp)*(r1x*r2y - r2x*r1y))*(o1x - o1xp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x))^2 - (r1x*(o1x - o1xp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x))];...
                      [                       (r1y*(o1y - o1yp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x)) - (((r1y - r2y)*(o1x*o1yp - o1xp*o1y) - (o1y - o1yp)*(r1x*r2y - r2x*r1y))*(o1y - o1yp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x))^2, (((r1y - r2y)*(o1x*o1yp - o1xp*o1y) - (o1y - o1yp)*(r1x*r2y - r2x*r1y))*(o1x - o1xp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x))^2 - (o1x*o1yp - o1xp*o1y + r1x*(o1y - o1yp))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x))]];

        dH2_dr1 = [   [ (((r1x - r2x)*(o2x*o2yp - o2xp*o2y) - (o2x - o2xp)*(r1x*r2y - r2x*r1y))*(o2y - o2yp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x))^2 - (o2xp*o2y - o2x*o2yp + r2y*(o2x - o2xp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x)),                       (r2x*(o2x - o2xp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x)) - (((r1x - r2x)*(o2x*o2yp - o2xp*o2y) - (o2x - o2xp)*(r1x*r2y - r2x*r1y))*(o2x - o2xp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x))^2];...
                      [                       (((r1y - r2y)*(o2x*o2yp - o2xp*o2y) - (o2y - o2yp)*(r1x*r2y - r2x*r1y))*(o2y - o2yp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x))^2 - (r2y*(o2y - o2yp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x)), (o2x*o2yp - o2xp*o2y + r2x*(o2y - o2yp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x)) - (((r1y - r2y)*(o2x*o2yp - o2xp*o2y) - (o2y - o2yp)*(r1x*r2y - r2x*r1y))*(o2x - o2xp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x))^2]];
 
        dH2_dr2 = [   [ (o2xp*o2y - o2x*o2yp + r1y*(o2x - o2xp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x)) - (((r1x - r2x)*(o2x*o2yp - o2xp*o2y) - (o2x - o2xp)*(r1x*r2y - r2x*r1y))*(o2y - o2yp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x))^2,                       (((r1x - r2x)*(o2x*o2yp - o2xp*o2y) - (o2x - o2xp)*(r1x*r2y - r2x*r1y))*(o2x - o2xp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x))^2 - (r1x*(o2x - o2xp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x))];...
                      [                       (r1y*(o2y - o2yp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x)) - (((r1y - r2y)*(o2x*o2yp - o2xp*o2y) - (o2y - o2yp)*(r1x*r2y - r2x*r1y))*(o2y - o2yp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x))^2, (((r1y - r2y)*(o2x*o2yp - o2xp*o2y) - (o2y - o2yp)*(r1x*r2y - r2x*r1y))*(o2x - o2xp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x))^2 - (o2x*o2yp - o2xp*o2y + r1x*(o2y - o2yp))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x))]];
 
        dh1_dH1 = [ (2*H1x - 2*o1x)/(2*((H1x - o1x)^2 + (H1y - o1y)^2)^(1/2)), (2*H1y - 2*o1y)/(2*((H1x - o1x)^2 + (H1y - o1y)^2)^(1/2))];
 
        dh2_dH2 = [ (2*H2x - 2*o2x)/(2*((H2x - o2x)^2 + (H2y - o2y)^2)^(1/2)), (2*H2y - 2*o2y)/(2*((H2x - o2x)^2 + (H2y - o2y)^2)^(1/2))];
        
        dDeltal_dh1 = (l*(2*h1 + h2))/3;
        
        dDeltal_dh2 = (l*(h1 + 2*h2))/3;
        
        J_lj = dDeltal_dh1*dh1_dH1*(dH1_dr1*dr1_dphi + dH1_dr2*dr2_dphi) + dDeltal_dh2*dh2_dH2*(dH2_dr1*dr1_dphi + dH2_dr2*dr2_dphi);
        
        J_l(j,:) = J_lj;
    end
end


%> @brief Computes the line reprojection error
%>
%> @param matrixOf2Dlines Matrix of 2D line coordinates
%> @param matrixOfReprojectedLines Matrix of reprojected 2D line coordinates
%>
%> @retval matrixOfLineReprojectionError Matrix of reprojection error
function matrixOfLineReprojectionError = computeLineReprojectionError(matrixOf2DLines, matrixOfReprojectedLines)
    matrixOfLineReprojectionError = zeros(size(matrixOf2DLines,1)/2,1);
    for j = 1:size(matrixOf2DLines,1)/2;
        r1x = matrixOfReprojectedLines(2*(j-1)+1,1);
        r1y = matrixOfReprojectedLines(2*(j-1)+2,1);
        r2x = matrixOfReprojectedLines(2*(j-1)+1,2);
        r2y = matrixOfReprojectedLines(2*(j-1)+2,2);
        o1x = matrixOf2DLines(2*(j-1)+1,1);
        o1y = matrixOf2DLines(2*(j-1)+2,1);
        o2x = matrixOf2DLines(2*(j-1)+1,2);
        o2y = matrixOf2DLines(2*(j-1)+2,2);
        o1xp = o1x - o2y + o1y;
        o1yp = o1y + o2x - o1x;
        o2xp = o2x - o2y + o1y;
        o2yp = o2y + o2x - o1x;
        H1x = ((o1x*o1yp - o1y*o1xp)*(r1x - r2x) - (o1x - o1xp)*(r1x*r2y - r1y*r2x))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x));
        H1y = ((o1x*o1yp - o1y*o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x*r2y - r1y*r2x))/((o1x - o1xp)*(r1y - r2y) - (o1y - o1yp)*(r1x - r2x));
        H2x = ((o2x*o2yp - o2y*o2xp)*(r1x - r2x) - (o2x - o2xp)*(r1x*r2y - r1y*r2x))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x));
        H2y = ((o2x*o2yp - o2y*o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x*r2y - r1y*r2x))/((o2x - o2xp)*(r1y - r2y) - (o2y - o2yp)*(r1x - r2x));
        h1 = sqrt((o1x - H1x)^2 + (o1y - H1y)^2);
        h2 = sqrt((o2x - H2x)^2 + (o2y - H2y)^2);
        l = sqrt((o1x - o2x)^2 + (o1y - o2y)^2);
        matrixOfLineReprojectionError(j) = l/3*(h1^2 + h1*h2 + h2^2);
    end
end


%> @brief Reprojects a 3D point onto the image plane w.r.t. the camera pose
%>
%> @param x 1st coordinate of camera position, w.r.t. the world frame
%> @param y 2nd coordinate of camera position, w.r.t. the world frame
%> @param z 3rd coordinate of camera position, w.r.t. the world frame
%> @param alpha Euler angle rotation in x-direction, w.r.t. the world frame
%> @param beta Euler angle rotation in y-direction, w.r.t. the world frame
%> @param gamma Euler angle rotation in z-direction, w.r.t. the world frame
%> @param z1j
%> @param z2j
%> @param z3j
%> @param f Focal length of the camera lense
%>
%> @retval reprojectedPoint Vector with 2D coordinates of corresponding 3D point
function reprojectedPoint = reprojectPoint(x,y,z,alpha,beta,gamma,z1j,z2j,z3j,f)
    reprojectedPoint = [    f / (-sin(beta)*z1j + cos(beta)*sin(alpha)*z2j + cos(alpha)*cos(beta)*z3j + z) * (cos(beta)*cos(gamma)*z1j + (cos(gamma)*sin(alpha)*sin(beta)-cos(alpha)*sin(gamma))*z2j + (sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta))*z3j + x);...
                            f / (-sin(beta)*z1j + cos(beta)*sin(alpha)*z2j + cos(alpha)*cos(beta)*z3j + z) * (cos(beta)*sin(gamma)*z1j + (cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma))*z2j + (cos(alpha)*sin(beta)*sin(gamma) - cos(gamma)*sin(alpha))*z3j + y)];
end


%> @brief Reprojects 3D points onto the image plane w.r.t. the camera pose
%>
%> @param x 1st coordinate of camera position, w.r.t. the world frame
%> @param y 2nd coordinate of camera position, w.r.t. the world frame
%> @param z 3rd coordinate of camera position, w.r.t. the world frame
%> @param alpha Euler angle rotation in x-direction, w.r.t. the world frame
%> @param beta Euler angle rotation in y-direction, w.r.t. the world frame
%> @param gamma Euler angle rotation in z-direction, w.r.t. the world frame
%> @param pointCloud3D A 3D pointcloud storing all 3D points
%> @param f Focal length of the camera lense
%>
%> @retval reprojectedPoints Matrix with 2D coordinates of corresponding 3D points
function reprojectedPoints = reprojectPoints(x,y,z,alpha,beta,gamma,pointCloud3D,f)
    reprojectedPoints = zeros(2*pointCloud3D.numberOfPoints, 1);
    for j = 1:pointCloud3D.numberOfPoints
        z1j = pointCloud3D.pointsIn3D(j).noisyCoordinatesInWorldFrame(1);
        z2j = pointCloud3D.pointsIn3D(j).noisyCoordinatesInWorldFrame(2);
        z3j = pointCloud3D.pointsIn3D(j).noisyCoordinatesInWorldFrame(3);
        reprojectedCoordinates_j = reprojectPoint(x,y,z,alpha,beta,gamma,z1j,z2j,z3j,f);
        reprojectedPoints(2*(j-1)+1:2*(j-1)+2) = reprojectedCoordinates_j;
    end
end


%> @brief Reprojects a 3D linecloud to the corresponding 2D lines in the image plane w.r.t. the camera 
%>
%> @param x 1st coordinate of camera position, w.r.t. the world frame
%> @param y 2nd coordinate of camera position, w.r.t. the world frame
%> @param z 3rd coordinate of camera position, w.r.t. the world frame
%> @param alpha Euler angle rotation in x-direction, w.r.t. the world frame
%> @param beta Euler angle rotation in y-direction, w.r.t. the world frame
%> @param gamma Euler angle rotation in z-direction, w.r.t. the world frame
%> @param lineCloud3D A 3D linecloud storing all 3D lines
%> @param f Focal length of the camera lense
%>
%> @retval reprojectedLines Matrix with 2D coordinates of corresponding 3D line
function reprojectedLines = reprojectLines(x,y,z,alpha,beta,gamma,lineCloud3D,f)
    reprojectedLines = zeros(2*lineCloud3D.numberOfLines, 2);
    for j = 1:lineCloud3D.numberOfLines
        z1j = lineCloud3D.linesIn3D(j).startPoint.noisyCoordinatesInWorldFrame(1);
        z2j = lineCloud3D.linesIn3D(j).startPoint.noisyCoordinatesInWorldFrame(2);
        z3j = lineCloud3D.linesIn3D(j).startPoint.noisyCoordinatesInWorldFrame(3);
        reprojectedStartPoint_j = reprojectPoint(x,y,z,alpha,beta,gamma,z1j,z2j,z3j,f);
        z1j = lineCloud3D.linesIn3D(j).endPoint.noisyCoordinatesInWorldFrame(1);
        z2j = lineCloud3D.linesIn3D(j).endPoint.noisyCoordinatesInWorldFrame(2);
        z3j = lineCloud3D.linesIn3D(j).endPoint.noisyCoordinatesInWorldFrame(3);
        reprojectedEndPoint_j = reprojectPoint(x,y,z,alpha,beta,gamma,z1j,z2j,z3j,f);
        reprojectedLines(2*(j-1)+1:2*(j-1)+2, 1) = reprojectedStartPoint_j;
        reprojectedLines(2*(j-1)+1:2*(j-1)+2, 2) = reprojectedEndPoint_j;
    end
end % reprojectLines() end