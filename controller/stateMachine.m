function output_args = stateMachine(input_args)
% 进行跳跃、姿态调整、落地等状态的切换及力矩规划
% 基于时间序列
global freego ;
persistent height tau_ff_out rpyDes Kp Kd KpFoot KdFoot;

t = input_args(1);
tau_ff = input_args(2:13);

for i=1:3:12
  tau_ff(i) = -0.5*tau_ff(i);
end

fDes = zeros(1,12);
tau_ff_out = zeros(1,12);
rpyDes = zeros(1,3);
Kp = zeros(1,3);
Kd = [1, 1, 1];
KpFoot = [80, 80, 120];
KdFoot = [3, 3, 3];

if(t<0.01)
  height = -0.3;
elseif(t<3)
  height = -0.3 + t*0.05;
  KpFoot = [100, 100, 300];
  KdFoot = [5, 5, 8];

elseif(t<3.3)
  Kp = [5, 0, 0];
  Kd = [3, 1, 1];
  KpFoot = [5, 0, 0];
  KdFoot = [5, 1, 1];
%   fDes = [0, 0, -110, 0, 0, -110, 0, 0, -110, 0, 0, -110];
  fDes = [-500, 0, -200, -500, 0, -200, -500, 0, -200, -500, 0, -200];

% elseif(t<3.5)
%   KpFoot = [5, 5, 0];
%   KdFoot = [5, 5, 5];
% %   fDes = [-200, 0, -500, -200, 0, -500, -200, 0, -500, -200, 0, -500];
%   fDes = [-500, 0, 0, -500, 0, 0, -500, 0, 0, -500, 0, 0];
elseif(t<3.4)
  height = -0.4;
  Kd = [3, 1, 1];
%   rpyDes = [2*3.1415, 0, 0];
tau_ff_out = tau_ff';
elseif(t<4.8)
  Kp = [5,0,0];
%   Kd = [1, 2,2];
  KpFoot = [1, 1, 1];
  KdFoot = [3, 3, 3];
  tau_ff_out = tau_ff';
%   rpyDes = [2*3.1415, 0, 0];

% elseif(t<5.2)
% %   Kp = [4, 2, 2];
% KpFoot = [280, 280, 280];
% KdFoot = [1, 1, 1];
% rpyDes = [2*3.1415, 0, 0];
% 
% else 
% %   Kp = [4, 2, 2];
% KpFoot = [80, 80, 120];
% KdFoot = [2, 2, 2];
% rpyDes = [2*3.1415, 0, 0];
% height = -0.25;
% end
elseif(t<5.1)
% Kp = [2, 2, 2];
Kd = [4, 2, 2];
KpFoot = [380, 380, 180];
% KdFoot = [1, 1, 1];
height = -0.15;
else 
%   Kp = [4, 2, 2];
KpFoot = [280, 280, 80];
KdFoot = [2, 2, 2];
% rpyDes = [2*3.1415, 0, 0];
height = -0.3;
end

pDes = [0, -0.08, height, 0, 0.08, height, 0, -0.08, height, 0, 0.08, height];

output_args = [pDes, fDes, tau_ff_out, Kp, Kd, KpFoot, KdFoot, rpyDes];



