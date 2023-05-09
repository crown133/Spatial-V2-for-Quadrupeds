function  [sys, x0, str, ts] = TD(t, x, u, flag, r, h, n0)


switch flag
    case 0
        [sys, x0, str, ts] = mdlInitializeSizes;
    case 1
        %sys = mdlUpdate(t, x, u, r, h);
        sys = mdlDerivatives(x, u, r, h, n0);
    case 3
        sys = mdlOutputs(x);
    case {2, 4, 9}
        sys = [];  %do nothing  
    otherwise 
        error('Simulink:blocks:unhandledFlag', num2str(flag));
end 
function [sys, x0, str, ts] = mdlInitializeSizes
    sizes = simsizes;
    sizes.NumContStates  = 2;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 2;
    sizes.NumInputs      = 1;
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes);
    x0  = [0; 0]; 
    str = [];
    ts  = [0 0];

 function  sys = mdlDerivatives(x, u, r, h, n0)
     sys(1,1) = h* x(2);
     sys(2,1) = h * fhan(x(1)- u, x(2), r, h/n0);
    % sys(2,1) = -h0 * r*sign(x(1)-u + x(2)*abs(x(2))/2/r);
    % sys(2,1) = -h0 * r*fst2(x, u, r, h0);
     
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
     
function f = fst2(x,u,r,d)
v=x(1)-u+abs(x(2))*x(2)/(2*r);
if v>=d
    f=sign(v);
else f=v/d;
end

function  sys = mdlOutputs(x)
%    sys(1, 1) = 0;
%    sys(2, 1) = 2;
    sys = x;
     