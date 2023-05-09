function robot = quadruped_robot( )
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
global freego ;
global X_Init1;

quadruped = freego;
persistent  last_robot last_n;

X_Init1 = [quadruped.QuatBody, quadruped.COMLocation, zeros(1,6), quadruped.q_init]';
pi =  3.14159265358979323846;
n = 16; % number of bodies

if length(last_robot) ~= 0 && last_n == n
  robot = last_robot;
  return
end

robot.gravity = [0 0 -1.63];  % -1.63 0 0 -9.80 get_gravity()
%% Xtree of quad
%    o  o  o  14(x) 15(y) 16(z)三轴动量轮 暂时忽略
%     \ | /
%       o 1   body
%    / / \  \                
%   FR FL HR HL             forward
%   o2 o5 o8 o 11         FL x -- x FR
%   |  |  |  |               |    |   see on top
%   o3 o6 o9 o 12         HL x -- x HR
%   |  |  |  |
%   o4 o7 o10o 13       
%%
robot.NB = n;
robot.parent = [0 1 2 3 1 5 6 1 8 9 1 11 12 1 1 1]; % the last three bodies are fly wheel

% joint type
robot.jtype{2} = 'Rx'; robot.jtype{5} = 'Rx';
robot.jtype{8} = 'Rx'; robot.jtype{11} = 'Rx';
robot.jtype{3} = 'Ry'; robot.jtype{4} = 'Ry';
robot.jtype{6} = 'Ry'; robot.jtype{7} = 'Ry';
robot.jtype{9} = 'Ry'; robot.jtype{10} = 'Ry';
robot.jtype{12} = 'Ry';robot.jtype{13} = 'Ry';

robot.jtype{14} = 'Rx';robot.jtype{15} = 'Ry';
robot.jtype{16} = 'Rz';
% Xtree
robot.Xtree{1} = eye(6); %xlt(freego.COMLocation);  %  base body
% abad
robot.Xtree{2} = xlt([ quadruped.bodyLength, -quadruped.bodyWidth, 0]/2); 
robot.Xtree{5} = xlt([ quadruped.bodyLength,  quadruped.bodyWidth, 0]/2);
robot.Xtree{8} = xlt([-quadruped.bodyLength, -quadruped.bodyWidth, 0]/2);
robot.Xtree{11}= xlt([-quadruped.bodyLength,  quadruped.bodyWidth, 0]/2);
% hip
robot.Xtree{3} = rotz(pi) * xlt([ quadruped.hipLocation(1), -quadruped.hipLocation(2), 0]); 
robot.Xtree{6} = rotz(pi) * xlt([ quadruped.hipLocation(1),  quadruped.hipLocation(2), 0]);
robot.Xtree{9} = rotz(pi) * xlt([-quadruped.hipLocation(1), -quadruped.hipLocation(2), 0]);
robot.Xtree{12}= rotz(pi) * xlt([-quadruped.hipLocation(1),  quadruped.hipLocation(2), 0]);
% knee
robot.Xtree{4} = xlt(quadruped.kneeLocation); 
robot.Xtree{7} = xlt(quadruped.kneeLocation);
robot.Xtree{10}= xlt(quadruped.kneeLocation);
robot.Xtree{13}= xlt(quadruped.kneeLocation);
% fly wheel
robot.Xtree{14} = xlt(quadruped.flywheelXLocation); % x axis
robot.Xtree{15} = xlt(quadruped.flywheelYLocation); % y axis
robot.Xtree{16} = xlt(quadruped.flywheelZLocation); % z axis

% Spatial Inertial
robot.I{1} = mcI(quadruped.bodyMass, quadruped.bodyCOM, quadruped.bodyRotationalInertia);

robot.I{2} = mcI(quadruped.abadMass,[quadruped.abadCOM(1) -quadruped.abadCOM(2) quadruped.abadCOM(3)], flipInertialY(quadruped.abadRotationalInertia));
robot.I{5} = mcI(quadruped.abadMass, quadruped.abadCOM, quadruped.abadRotationalInertia);
robot.I{8} = mcI(quadruped.abadMass,[quadruped.abadCOM(1) -quadruped.abadCOM(2) quadruped.abadCOM(3)], flipInertialY(quadruped.abadRotationalInertia));
robot.I{11}= mcI(quadruped.abadMass, quadruped.abadCOM, quadruped.abadRotationalInertia);

robot.I{3} = mcI(quadruped.hipMass,[quadruped.hipCOM(1) -quadruped.hipCOM(2) quadruped.hipCOM(3)], flipInertialY(quadruped.hipRotationalInertia));
robot.I{6} = mcI(quadruped.hipMass, quadruped.hipCOM, quadruped.hipRotationalInertia);
robot.I{9} = mcI(quadruped.hipMass,[quadruped.hipCOM(1) -quadruped.hipCOM(2) quadruped.hipCOM(3)], flipInertialY(quadruped.hipRotationalInertia));
robot.I{12}= mcI(quadruped.hipMass, quadruped.hipCOM, quadruped.hipRotationalInertia);

robot.I{4} = mcI(quadruped.kneeMass,[quadruped.kneeCOM(1) -quadruped.kneeCOM(2) quadruped.kneeCOM(3)], flipInertialY(quadruped.kneeRotationalInertia));
robot.I{7} = mcI(quadruped.kneeMass, quadruped.kneeCOM, quadruped.kneeRotationalInertia);
robot.I{10}= mcI(quadruped.kneeMass,[quadruped.kneeCOM(1) -quadruped.kneeCOM(2) quadruped.kneeCOM(3)], flipInertialY(quadruped.kneeRotationalInertia));
robot.I{13}= mcI(quadruped.kneeMass, quadruped.kneeCOM, quadruped.kneeRotationalInertia);

robot.I{14}= mcI(quadruped.flyWheelmass, [0, 0, 0], diag([quadruped.Iaxial, quadruped.Iradial, quadruped.Iradial]));
robot.I{15}= mcI(quadruped.flyWheelmass, [0, 0, 0], diag([quadruped.Iradial, quadruped.Iaxial, quadruped.Iradial]));
robot.I{16}= mcI(quadruped.flyWheelmass, [0, 0, 0], diag([quadruped.Iradial, quadruped.Iradial, quadruped.Iaxial]));

%% appearance 
robot.appearance.base = { 'tiles', [-50 50; -50 50; 0 0], 0.5 };
robot.appearance.body{1} = { 'box', [ quadruped.bodyLength/2  quadruped.bodyWidth/2 -quadruped.bodyHeight/2; -quadruped.bodyLength/2 -quadruped.bodyWidth/2  quadruped.bodyHeight/2], ...
                             'sphere', [ quadruped.bodyLength/2 + 0.05;-quadruped.bodyWidth/2 - 0.06; -quadruped.bodyHeight/2], 0.005, ...
                             'sphere', [ quadruped.bodyLength/2 + 0.05;-quadruped.bodyWidth/2 - 0.06;  quadruped.bodyHeight/2], 0.005, ...
                             'sphere', [ quadruped.bodyLength/2 + 0.05; quadruped.bodyWidth/2 + 0.06; -quadruped.bodyHeight/2], 0.005, ...
                             'sphere', [ quadruped.bodyLength/2 + 0.05; quadruped.bodyWidth/2 + 0.06;  quadruped.bodyHeight/2], 0.005, ...
                             'sphere', [-quadruped.bodyLength/2 - 0.05;-quadruped.bodyWidth/2 - 0.06; -quadruped.bodyHeight/2], 0.005, ...
                             'sphere', [-quadruped.bodyLength/2 - 0.05;-quadruped.bodyWidth/2 - 0.06;  quadruped.bodyHeight/2], 0.005, ...
                             'sphere', [-quadruped.bodyLength/2 - 0.05; quadruped.bodyWidth/2 + 0.06; -quadruped.bodyHeight/2], 0.005, ...
                             'sphere', [-quadruped.bodyLength/2 - 0.05; quadruped.bodyWidth/2 + 0.06;  quadruped.bodyHeight/2], 0.005 };

robot.appearance.body{2} = { 'cyl', [ 0 0 0; 0 -0.05 0], 0.03 };
robot.appearance.body{5} = { 'cyl', [ 0 0 0; 0  0.05 0], 0.03 };
robot.appearance.body{8} = { 'cyl', [ 0 0 0; 0 -0.05 0], 0.03 };
robot.appearance.body{11}= { 'cyl', [ 0 0 0; 0  0.05 0], 0.03 };

robot.appearance.body{3} = { 'cyl', [ 0 -0.016 0; 0 0.016 0], 0.025, 'box', [-0.015, -0.01, 0; 0.015, 0.01, - quadruped.hipLinkLength]};
robot.appearance.body{6} = { 'cyl', [ 0 -0.016 0; 0 0.016 0], 0.025, 'box', [-0.015, -0.01, 0; 0.015, 0.01, - quadruped.hipLinkLength] };
robot.appearance.body{9} = { 'cyl', [ 0 -0.016 0; 0 0.016 0], 0.025, 'box', [-0.015, -0.01, 0; 0.015, 0.01, - quadruped.hipLinkLength] };
robot.appearance.body{12}= { 'cyl', [ 0 -0.016 0; 0 0.016 0], 0.025, 'box', [-0.015, -0.01, 0; 0.015, 0.01, - quadruped.hipLinkLength] };

robot.appearance.body{4} = { 'cyl', [ 0 -0.016 0; 0 0.016 0], 0.025, 'box', [-0.015, -0.01, 0; 0.015, 0.01, - quadruped.kneeLinkLength], 'sphere',[0,0,- quadruped.kneeLinkLength], 0.02 };
robot.appearance.body{7} = { 'cyl', [ 0 -0.016 0; 0 0.016 0], 0.025, 'box', [-0.015, -0.01, 0; 0.015, 0.01, - quadruped.kneeLinkLength], 'sphere',[0,0,- quadruped.kneeLinkLength], 0.02  };
robot.appearance.body{10}= { 'cyl', [ 0 -0.016 0; 0 0.016 0], 0.025, 'box', [-0.015, -0.01, 0; 0.015, 0.01, - quadruped.kneeLinkLength], 'sphere',[0,0,- quadruped.kneeLinkLength], 0.02  };
robot.appearance.body{13}= { 'cyl', [ 0 -0.016 0; 0 0.016 0], 0.025, 'box', [-0.015, -0.01, 0; 0.015, 0.01, - quadruped.kneeLinkLength], 'sphere',[0,0,- quadruped.kneeLinkLength], 0.02  };

robot.appearance.body{14} = { 'cyl', [ -0.01 0 0; 0.01 0 0], 0.03 };
robot.appearance.body{15} = { 'cyl', [ 0 -0.01 0; 0 0.01 0], 0.03 };
robot.appearance.body{16} = { 'cyl', [ 0 0 -0.01; 0 0 0.01], 0.03 };

%% Camera settings
robot.camera.body = 1;
robot.camera.direction = [5 20 12];
robot.camera.locus = [0 0];

%%
%body
robot.gc.point(:,1) = [ quadruped.bodyLength/2 + 0.05;-quadruped.bodyWidth/2 - 0.06; -quadruped.bodyHeight/2];
robot.gc.point(:,2) = [ quadruped.bodyLength/2 + 0.05;-quadruped.bodyWidth/2 - 0.06;  quadruped.bodyHeight/2];
robot.gc.point(:,3) = [ quadruped.bodyLength/2 + 0.05; quadruped.bodyWidth/2 + 0.06; -quadruped.bodyHeight/2];
robot.gc.point(:,4) = [ quadruped.bodyLength/2 + 0.05; quadruped.bodyWidth/2 + 0.06;  quadruped.bodyHeight/2];
robot.gc.point(:,5) = [-quadruped.bodyLength/2 - 0.05;-quadruped.bodyWidth/2 - 0.06; -quadruped.bodyHeight/2];
robot.gc.point(:,6) = [-quadruped.bodyLength/2 - 0.05;-quadruped.bodyWidth/2 - 0.06;  quadruped.bodyHeight/2];
robot.gc.point(:,7) = [-quadruped.bodyLength/2 - 0.05; quadruped.bodyWidth/2 + 0.06; -quadruped.bodyHeight/2];
robot.gc.point(:,8) = [-quadruped.bodyLength/2 - 0.05; quadruped.bodyWidth/2 + 0.06;  quadruped.bodyHeight/2];
%knee
robot.gc.point(:,9)  = [0; 0; -quadruped.hipLinkLength];
robot.gc.point(:,10) = [0; 0; -quadruped.hipLinkLength];
robot.gc.point(:,11) = [0; 0; -quadruped.hipLinkLength];
robot.gc.point(:,12) = [0; 0; -quadruped.hipLinkLength];
%foot
robot.gc.point(:,13) = [0; 0; -quadruped.kneeLinkLength];
robot.gc.point(:,14) = [0; 0; -quadruped.kneeLinkLength];
robot.gc.point(:,15) = [0; 0; -quadruped.kneeLinkLength];
robot.gc.point(:,16) = [0; 0; -quadruped.kneeLinkLength];

robot.gc.body = [ones(1,8), 3,6,9,12, 4,7,10,13]; % body + knee + foot 

%%
robot = floatbase(robot);		% replace joint 1 with a chain of 6 joints emulating a floating base

last_robot = robot;
last_n = n;

%%
function matrix = flipInertialY(V)

matrix = [V(1,1) -V(1,2) V(1,3); -V(2,1) V(2,2) -V(2,3); V(3,1) -V(3,2) V(3,3)];

end
end
