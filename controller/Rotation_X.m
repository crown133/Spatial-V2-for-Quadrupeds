function Rotation_Matrix_X= Rotation_X(theta)
%��X��ת��ת������
%2010��2��9��0:13:40

Rotation_Matrix_X=...
    [1 0 0;
     0 cos(theta) -sin(theta);
     0 sin(theta) cos(theta)];
