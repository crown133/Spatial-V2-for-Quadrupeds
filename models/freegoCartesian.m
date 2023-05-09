function torque = freegoCartesian(q, sideSign, f_f, pDes, vDes, Kp, Kd)
%FREEGO_JACOBIAN freego 腿部雅可比
global freego ;
torque = zeros(3,1); %关节转矩
footForce = zeros(3,1); %足端力
l1 = freego.abadLinkLength;
l2 = freego.hipLinkLength;
l3 = freego.kneeLinkLength;
l4 = 0;

s1 = sin(q(1));
s2 = sin(q(2));
s3 = sin(q(3));

c1 = cos(q(1));
c2 = cos(q(2));
c3 = cos(q(3));

c23 = c2 * c3 - s2 * s3;
s23 = s2 * c3 + c2 * s3;

J = [0                                                          l3 * c23 + l2 * c2              l3 * c23;
    l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1     -l3 * s1 * s23 - l2 * s1 * s2   -l3 * s1 * s23;
    l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1      l3 * c1 * s23 + l2 * c1 * s2    l3 * c1 * s23];

P = [l3 * s23 + l2 * s2; (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1; (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;];

V = J * q;

footForce(1) = f_f(1) + Kp(1)*(pDes(1) - P(1)) + Kd(1)*(vDes(1) - V(1));
footForce(2) = f_f(2) + Kp(2)*(pDes(2) - P(2)) + Kd(2)*(vDes(2) - V(2));
footForce(3) = f_f(3) + Kp(3)*(pDes(3) - P(3)) + Kd(3)*(vDes(3) - V(3));

torque = J' * footForce;

end

