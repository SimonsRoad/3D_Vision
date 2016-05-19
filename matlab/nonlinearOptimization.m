function [optimizedEstimatedPose, confidenceMatrix] = nonlinearOptimization(initialEstimation, pointCloud3D, pointCloud2D, focalLength)

% Set parameters
TOL = 10e-3;
MAX_ITER = 1000;

% Define helper variables
nrOfPoints = pointCloud3D.numberOfPoints;
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
    index = min(norm(matrixOf2DPoints-reproject(x0,y0,z0,alpha1,beta1,gamma1,pointCloud3D,f)), norm(matrixOf2DPoints-reproject(x0,y0,z0,alpha2,beta2,gamma2,pointCloud3D,f)));
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

matrixOfReprojectedCoordinates = reproject(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6),pointCloud3D,f);
matrixOfReprojectionError = matrixOfReprojectedCoordinates - matrixOf2DPoints;
disp(['Norm of reprojection error of initial estimation: ' num2str(norm(matrixOfReprojectionError))])

updateStep = Inf;
iter = 1;
while( norm(updateStep) > TOL && iter <= MAX_ITER )
    
    % Calculate Jacobian (J(x) \in \R^{2N x 6})
    J = jac(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6),pointCloud3D,f);
    
    % Calculate Reprojection (z(x) \in \R^{2N x 1})
    matrixOfReprojectedCoordinates = reproject(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6),pointCloud3D,f);
    
    % Calculate Reprojection Error (Delta z = z - z(x) \in \R^{2N x 1})
    matrixOfReprojectionError = matrixOfReprojectedCoordinates - matrixOf2DPoints;
    
    % Calculate the update step (delta x = -(J^T W J)^-1 * J^T W Delta z)
    updateStep = -pinv(J,10e-3)*matrixOfReprojectionError;
    
    % Update the pose
    pose = pose+updateStep;
    
    % Update iteration counter
    iter = iter + 1;
end

if (iter > MAX_ITER)
    disp(['Nonlinear optimization did not converge in ' num2str(MAX_ITER) ' iterations!']);
else
    disp(['Nonlinear optimization converged in ' num2str(iter-1) ' iterations']);
end

disp(['Norm of reprojection error of optimized estimation: ' num2str(norm(matrixOfReprojectionError))])

R = rotz(pose(6)*180/pi)*roty(pose(5)*180/pi)*rotx(pose(4)*180/pi);
t = [pose(1); pose(2); pose(3)];

optimizedEstimatedPose = [R t];
confidenceMatrix = inv(J'*J);
return

function J = jac(x,y,z,alpha,beta,gamma,pointCloud3D,f)
    J = zeros(2*pointCloud3D.numberOfPoints, 6);
    for j = 1:pointCloud3D.numberOfPoints
        z1j = pointCloud3D.pointsIn3D(j).noisyCoordinatesInWorldFrame(1);
        z2j = pointCloud3D.pointsIn3D(j).noisyCoordinatesInWorldFrame(2);
        z3j = pointCloud3D.pointsIn3D(j).noisyCoordinatesInWorldFrame(3);
        
        J_j = [   [ f/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)),                                                                           0, -(f*(x - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2,   (f*(z2j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z3j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta))))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) - (f*(z2j*cos(alpha)*cos(beta) - z3j*cos(beta)*sin(alpha))*(x - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, (f*(z3j*cos(alpha)*cos(beta)*cos(gamma) - z1j*cos(gamma)*sin(beta) + z2j*cos(beta)*cos(gamma)*sin(alpha)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) + (f*(z1j*cos(beta) + z3j*cos(alpha)*sin(beta) + z2j*sin(alpha)*sin(beta))*(x - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, -(f*(z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))]
                  [                                                                           0, f/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)), -(f*(y + z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, - (f*(z2j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z3j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma))))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) - (f*(z2j*cos(alpha)*cos(beta) - z3j*cos(beta)*sin(alpha))*(y + z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2, (f*(z3j*cos(alpha)*cos(beta)*sin(gamma) - z1j*sin(beta)*sin(gamma) + z2j*cos(beta)*sin(alpha)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha)) + (f*(z1j*cos(beta) + z3j*cos(alpha)*sin(beta) + z2j*sin(alpha)*sin(beta))*(y + z2j*(cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma)) - z3j*(cos(gamma)*sin(alpha) - cos(alpha)*sin(beta)*sin(gamma)) + z1j*cos(beta)*sin(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))^2,  (f*(z3j*(sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta)) - z2j*(cos(alpha)*sin(gamma) - cos(gamma)*sin(alpha)*sin(beta)) + z1j*cos(beta)*cos(gamma)))/(z - z1j*sin(beta) + z3j*cos(alpha)*cos(beta) + z2j*cos(beta)*sin(alpha))]];
        J(2*(j-1)+1:2*(j-1)+2,:) = J_j;
    end
return

function reprojectedCoordinates = reproject(x,y,z,alpha,beta,gamma,pointCloud3D,f)
    reprojectedCoordinates = zeros(2*pointCloud3D.numberOfPoints, 1);
    for j = 1:pointCloud3D.numberOfPoints
        z1j = pointCloud3D.pointsIn3D(j).noisyCoordinatesInWorldFrame(1);
        z2j = pointCloud3D.pointsIn3D(j).noisyCoordinatesInWorldFrame(2);
        z3j = pointCloud3D.pointsIn3D(j).noisyCoordinatesInWorldFrame(3);
        reprojectedCoordinates_j = [    f / (-sin(beta)*z1j + cos(beta)*sin(alpha)*z2j + cos(alpha)*cos(beta)*z3j + z) * (cos(beta)*cos(gamma)*z1j + (cos(gamma)*sin(alpha)*sin(beta)-cos(alpha)*sin(gamma))*z2j + (sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)*sin(beta))*z3j + x);...
                                        f / (-sin(beta)*z1j + cos(beta)*sin(alpha)*z2j + cos(alpha)*cos(beta)*z3j + z) * (cos(beta)*sin(gamma)*z1j + (cos(alpha)*cos(gamma) + sin(alpha)*sin(beta)*sin(gamma))*z2j + (cos(alpha)*sin(beta)*sin(gamma) - cos(gamma)*sin(alpha))*z3j + y)];
        reprojectedCoordinates(2*(j-1)+1:2*(j-1)+2) = reprojectedCoordinates_j;
    end
return

