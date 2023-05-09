function  Qout = fbanimm( X )

Q_old = evalin('base','Q_last'); 

if length(size(X)) == 3			% collapse 3D -> 2D array
  tmp(:,:) = X(:,1,:);
  X = tmp;
end

% apply kinematic transform using fbkin

% for i = 1 : size(X,2)
%   Q(:,i) = fbkin( X(1:7,i) );
% end

Q = fbkin( X(1:7,1) );
Q = [Q_old, Q];
% This code removes wrap-arounds and step-changes on passing through a
% singularity.  Whenever q6 or q4 wrap, they jump by 2*pi.  However, when
% q5 hits pi/2 (or -pi/2), q4 and q6 both jump by pi, and q5 turns around.
% To undo this, the code looks to see whether n is an even or odd number,
% and replaces q5 with pi-q5 whenever n is odd.  q4 is calculated via the
% sum or difference of q4 and q6 on the grounds that the sum well defined
% at the singularity at q5=pi/2 and the difference is well defined at
% q5=-pi/2.

  i = 2;
  if(Q(6,i) < -3)
  q6 = Q(6,i);
  end
  nn = round( (Q(6,i-1) - Q(6,i)) / pi );
  q6 = Q(6,i) + nn*pi;
  
  if Q(5,i) >= 0
    q46 = Q(4,i) + Q(6,i);
    q46 = q46 + 2*pi * round( (Q(4,i-1)+Q(6,i-1) - q46) / (2*pi) );
    Q(4,i) = q46 - q6;
  else
    q46 = Q(4,i) - Q(6,i);
    q46 = q46 + 2*pi * round( (Q(4,i-1)-Q(6,i-1) - q46) / (2*pi) );
    Q(4,i) = q46 + q6;
  end
  Q(6,i) = q6;
  
%   nnn = round( (Q(4,i-1) - Q(4,i)) / pi );
%   q4 = Q(4,i) + nnn*pi;
%   Q(4,i) = q4;
  
  if mod(nn,2) == 0
    q5 = Q(5,i);
  else
    q5 = pi - Q(5,i);
  end
  Q(5,i) = q5 + 2*pi * round( (Q(5,i-1) - q5) / (2*pi) );
% Q(5,i) = Q(5,i) + nnn*pi/2;

%   if abs( abs(Q(5,i)) - 1.57 ) < 0.003
%       Q(5,i) = Q(5,i) - 3*pi/2;
%   end
  
assignin('base','Q_last', Q(:,2)); % 为该变量指派新的值
Qout = Q(:,2);
