function  Qout = fbanimmm( X )


if length(size(X)) == 3			% collapse 3D -> 2D array
  tmp(:,:) = X(:,1,:);
  X = tmp;
end

% apply kinematic transform using fbkin

% for i = 1 : size(X,2)
%   Q(:,i) = fbkin( X(1:7,i) );
% end

Q = fbkin( X(1:7,1) );

Qout = Q;
