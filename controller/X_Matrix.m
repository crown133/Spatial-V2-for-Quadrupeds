function X_Matrix= X_Matrix( vector )
%������ά�����Ĳ�˾���
%���룺����vector 1X3
%�������˾���X_Matrix 3X3
%2009��11��19��15:12:29
X_Matrix=...
    [0 -vector(3) vector(2);
    vector(3) 0 -vector(1);
    -vector(2) vector(1) 0;];