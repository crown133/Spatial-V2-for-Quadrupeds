function X_Matrix= X_Matrix( vector )
%生成三维向量的叉乘矩阵
%输入：向量vector 1X3
%输出：叉乘矩阵X_Matrix 3X3
%2009年11月19日15:12:29
X_Matrix=...
    [0 -vector(3) vector(2);
    vector(3) 0 -vector(1);
    -vector(2) vector(1) 0;];