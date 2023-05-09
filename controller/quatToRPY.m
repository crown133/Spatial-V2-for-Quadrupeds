function RPY = quatToRPY( q )

% RPY = zeros(3,1);
Epsilon = 0.0009765625;
Threshold = 0.5 - Epsilon;

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

test = q0*q2 - q1*q3;

% if(test < -Threshold || test > Threshold)
%     signn = sign(test);
%     Phi   = 0; 
%     Theta = signn * pi / 2.0;
%     Psi = -2*signn*atan2(q1, q0);
% else
    Phi   = atan2(2*(q0*q1+q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3); 
    Theta = asin( min( [2*(q0*q2-q3*q1), 0.99999]) );
    Psi   = atan2(2*(q0*q3+q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3);
% end


RPY = [Phi Theta Psi];

end
