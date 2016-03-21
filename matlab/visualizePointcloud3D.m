function visualizePointcloud3D(pointcloud)
% Input: Array of type PointIn3D
% Output: none, just creates a plot of the pointcloud

numberOfPoints = size(pointcloud,2);
X = zeros(numberOfPoints,1);
Y = zeros(numberOfPoints,1);
Z = zeros(numberOfPoints,1);

for i = 1:numberOfPoints
    for j = 1:3
        X(i) = pointcloud(i).trueCoordinates(1);
        Y(i) = pointcloud(i).trueCoordinates(2);
        Z(i) = pointcloud(i).trueCoordinates(3);
    end
end

figure
plot3(X',Y',Z','.','markers',10)
axis vis3d

end