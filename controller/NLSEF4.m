function  [sys, x0, str, ts] = TD(t, x, u, flag, c, r, h1)


switch flag,
    case 0,
        [sys, x0, str, ts] = mdlInitializeSizes;
%     case 2
%         sys = mdlUpdate(u, r, c, h1);
    case 3,
        sys = mdlOutputs(u, r, c, h1);
    case {1, 2, 4, 9},
        sys = [];  %do nothing  
    otherwise 
        error('Simulink:blocks:unhandledFlag', num2str(flag));
end 
function [sys, x0, str, ts] = mdlInitializeSizes
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 1;
    sizes.NumInputs      = 2;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes);
    x0  = []; 
    str = [];
    ts  = [0 0];
     
% function sys = mdlUpdate(u, r, c, h1)
%      sys = -fhan(u(1), c*u(2), r, h1);   
function  sys = mdlOutputs(u, r, c, h1)
     sys = -fhan(u(1), c*u(2), r, h1);
%      sys = 0.5*fal(u(1), r, h1) + 0.25*fal(u(2), c, h1);
function  fh = fhan(x1, x2, r, h)
     d = r*h;
     d0 = h*d;
     y = x1 + h*x2;
     a0 = sqrt(d*d +8*r*abs(y));
     
     if abs(y)>d0
         a = x2 + (a0 - d)*sign(y)/2;
     else 
         a = x2 + y/h;
     end
     
     if abs(a)>d
         fh = -r*sign(a);
     else
         fh = -r*a/d;
     end
     
function f=fal(e,a,d)
    if abs(e)<d
        f=e*d^(a-1);
    else f=(abs(e))^a*sign(e);
    end
    

    
    