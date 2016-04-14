function [R,t]= DLT_marlin(matrixOfPointsIn3D, matrixOfPointsIn2D)

numberOfCorrespondences = size(matrixOfPointsIn2D,2);
D = zeros(numberOfCorrespondences*2,12);
for i = 1:numberOfCorrespondences
    xi = matrixOfPointsIn3D(1,i);
    yi = matrixOfPointsIn3D(2,i);
    zi = matrixOfPointsIn3D(3,i);
    ui = matrixOfPointsIn2D(1,i);
    vi = matrixOfPointsIn2D(2,i);
    D_ = [  xi  yi  zi  0   0   0   -ui*xi  -ui*yi  -ui*zi  1   0   -ui;
            0   0   0   xi  yi  zi  -vi*xi  -vi*yi  -vi*zi  0   1   -vi];
    D(i*2-1:i*2,:)= D_;
end

DD = D.'*D;
[V,~] = eig(DD);

v = V(:,1);
v = v/norm(v(7:9));
v = v*sign(v(12));

R = reshape(v(1:9),3,3).';
t = v(10:12);

XXc = R*matrixOfPointsIn3D+repmat(t,1,size(matrixOfPointsIn3D,2));
[R,t]= calcampose(XXc,matrixOfPointsIn3D);

return

function [R2,t2] = calcampose(XXc,XXw)

n = length(XXc);

X= XXw;
Y= XXc;

K= eye(n)-ones(n,n)/n;

ux= mean(X,2);
uy= mean(Y,2);
sigmx2= mean(sum((X*K).^2));

SXY= Y*K*(X')/n;
[U, D, V]= svd(SXY);
S= eye(3);
if det(SXY) < 0
    S(3,3)= -1;
end

R2= U*S*(V');
c2= trace(D*S)/sigmx2;
t2= uy-c2*R2*ux;

X= R2(:,1);
Y= R2(:,2);
Z= R2(:,3);
if norm(cross(X,Y)-Z) > 2e-2
    R2(:,3)= -Z;
end

return
