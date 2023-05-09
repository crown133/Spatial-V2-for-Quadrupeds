function  [sys, x0, str, ts] = ESO(t, x, u, flag, h, b0, beta, d)


switch flag,
    case 0,
        [sys, x0, str, ts] = mdlInitializeSizes;
    case 1,
        sys = mdlDerivatives(x, u, h, b0, beta, d);
    case 3,
        sys = mdlOutputs(x);
    case {2, 4, 9},
        sys = [];  %do nothing  
    otherwise 
        error('Simulink:blocks:unhandledFlag', num2str(flag));
end 

function [sys, x0, str, ts] = mdlInitializeSizes
    sizes = simsizes;
    sizes.NumContStates  = 3;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 3;
    sizes.NumInputs      = 2;
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes);
    x0  = [0; 0; 0]; 
    str = [];
    ts  = [0 0];

function  sys = mdlDerivatives(x, u, h, b0, beta, d)
   e = x(1) - u(1); %u(1)为系统输出反馈
   fe = fal(e, d(1), h);
   fe1 = fal(e, d(2), h);  
   sys(1,1) = h * (x(2) - beta(1)*e);
   sys(2,1) = h * (x(3) - beta(2)*fe + b0*u(2)); %u(2)为adrc的输出
   sys(3,1) = - h * beta(3)*fe1;
   
function  f = fal(x, a, b)
    if abs(x)<=b
        f = x / power(b,1-a);
    else 
        f = power(abs(x),a) * sign(x);
    end
    
    
function  sys = mdlOutputs(x)
%    sys(1, 1) = 0;
%    sys(2, 1) = 2;
     sys = x;