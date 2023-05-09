clc
clear

%%freego parameter define

% quadruped.bodyLength   bodyWidth   bodyHeight
% quadruped.abadGearRatio  hipGearRatio  kneeGearRatio
% quadruped.abadLinkLength hipLinkLength kneeLinkLength 
% quadruped.bodyMass abadMass hipMass kneeMass
% quadruped.bodyCOM  abadCOM  hipCOM  kneeCOM
% quadruped.bodyRotationalInertia 
% quadruped.abadRotationalInertia
% quadruped.hipRotationalInertia
% quadruped.kneeRotationalInertia
% quadruped.abadLocation  hipLocation  kneeLocation  
%%
global freego;

freego.bodyLength = 0.19 * 2;
freego.bodyWidth = 0.049 * 2;
freego.bodyHeight = 0.05 * 2;
freego.abadLinkLength = 0.062;
freego.hipLinkLength = 0.209;
freego.kneeLinkLength = 0.195;

freego.bodyMass = 3.3;
freego.abadMass = 0.54;
freego.hipMass = 0.634;
freego.kneeMass = 0.064;

freego.bodyCOM = [0 0 0]';
freego.abadCOM = [0, 0.036, 0]';
freego.hipCOM = [0, 0.016, -0.02]';
freego.kneeCOM = [0, 0, -0.061]';

freego.bodyRotationalInertia = [11253, 0, 0; 0, 36203, 0; 0, 0, 42673] * 1e-6;  %Ib 本体惯量
freego.abadRotationalInertia = [381, 58, 0.45; 58, 560, 0.95; 0.45, 0.95, 444] * 1e-6; % 胯惯量
freego.hipRotationalInertia = [1983, 245, 13; 245, 2103, 1.5; 13, 1.5, 408] * 1e-6; % 大腿惯量
freego.kneeRotationalInertia = [6, 0, 0; 0, 248, 0; 0, 0, 245] * 1e-6; % 小腿惯量
freego.kneeRotationalInertia = ry(pi/2) * freego.kneeRotationalInertia * ry(pi/2)';

freego.abadLocation = [freego.bodyLength, freego.bodyWidth, 0] * 0.5;
freego.hipLocation = [0, freego.abadLinkLength, 0];
freego.kneeLocation = [0, 0, -freego.hipLinkLength];

freego.flywheelXLocation = [0.06, 0, 0]';  % rfx
freego.flywheelYLocation = [0, 0.06, 0]';  % rfy
freego.flywheelZLocation = [0, 0, 0.06]';  % rfz

freego.flywheelCs = [freego.flywheelXLocation, freego.flywheelYLocation, freego.flywheelZLocation];  % xyz飞轮安装矩阵 

freego.flyWheelmass = 0.7; % the mass of fly wheel
freego.flyWheelR = 0.05;	% radius
freego.flyWheelT = 0.01;	% thickness

freego.Iaxial = freego.flyWheelmass * freego.flyWheelR^2 / 2;
freego.Iradial = freego.flyWheelmass * (3*freego.flyWheelR^2 + freego.flyWheelT^2) / 12;
freego.flyWheelIm = diag([freego.Iaxial; freego.Iaxial; freego.Iaxial]); %飞轮轴向转动惯量对角阵 

freego.COMLocation = [0, 0, 0.6];
freego.QuatBody = [1, 0, 0, 0];
freego.EulerBody = quatToRPY(freego.QuatBody);

       abad_InitAngle = [0 0 0 0];
       hip_InitAngle = -ones(1,4)*0.6;
       knee_InitAngle = ones(1,4)*1.2;
freego.q_init = [abad_InitAngle(1), hip_InitAngle(1), knee_InitAngle(1) ...
                 abad_InitAngle(2), hip_InitAngle(2), knee_InitAngle(2) ...
                 abad_InitAngle(3), hip_InitAngle(3), knee_InitAngle(3) ...
                 abad_InitAngle(4), hip_InitAngle(4), knee_InitAngle(4), 0, 0, 0];
%%
X_last = [freego.QuatBody, freego.COMLocation]';
Q_last = [freego.COMLocation, freego.EulerBody]';

% function  Qout = fbanimm( X )
% 
% X_old = evalin('base','X_last'); 
% 
% if length(size(X)) == 3			% collapse 3D -> 2D array
%   tmp(:,:) = X(:,1,:);
%   X = tmp;
% end
% assignin('base','X_last', X); % 为该变量指派新的值
% X = [X_old, X];
% 
% % apply kinematic transform using fbkin
% 
% % for i = 1 : size(X,2)
% %   Q(:,i) = fbkin( X(1:7,i) );
% % end
% 
% Q(:,2) = fbkin( X(1:7,2) );
% 
% % This code removes wrap-arounds and step-changes on passing through a
% % singularity.  Whenever q6 or q4 wrap, they jump by 2*pi.  However, when
% % q5 hits pi/2 (or -pi/2), q4 and q6 both jump by pi, and q5 turns around.
% % To undo this, the code looks to see whether n is an even or odd number,
% % and replaces q5 with pi-q5 whenever n is odd.  q4 is calculated via the
% % sum or difference of q4 and q6 on the grounds that the sum well defined
% % at the singularity at q5=pi/2 and the difference is well defined at
% % q5=-pi/2.
% 
% for i = 2 : size(X,2)
%   n = round( (Q(6,i-1) - Q(6,i)) / pi );
%   q6 = Q(6,i) + n*pi;
%   if Q(5,i) >= 0
%     q46 = Q(4,i) + Q(6,i);
%     q46 = q46 + 2*pi * round( (Q(4,i-1)+Q(6,i-1) - q46) / (2*pi) );
%     Q(4,i) = q46 - q6;
%   else
%     q46 = Q(4,i) - Q(6,i);
%     q46 = q46 + 2*pi * round( (Q(4,i-1)-Q(6,i-1) - q46) / (2*pi) );
%     Q(4,i) = q46 + q6;
%   end
%   Q(6,i) = q6;
%   if mod(n,2) == 0
%     q5 = Q(5,i);
%   else
%     q5 = pi - Q(5,i);
%   end
%   Q(5,i) = q5 + 2*pi * round( (Q(5,i-1) - q5) / (2*pi) );
% end
% 
% Q(:,1) = Q(:,2);
% Qout = Q(:,size(X,2));

