function [sys,x0,str,ts]=han_td(t,x,u,flag,r,d)

switch flag,
  case 0                                                
    [sys,x0,str,ts] = mdlInitializeSizes;
  case 1                                                
    sys = mdlDerivatives(x,u,r,d);
   case 3                                                
    sys = mdlOutputs(x);  
  case {2,4,9}                                         
    sys = []; % do nothing    
  otherwise 
     error(['Unhandled flag=',num2str(flag)]); 
end

function [sys,x0,str,ts] = mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0  = [0;0]; 
str = [];
ts  = [-1 0];

function sys = mdlDerivatives(x,u,r,d)
sys(1,1) = x(2);
sys(2,1) = -r*fst2(x,u,r,d);

function sys = mdlOutputs(x)
sys = x;

function f = fst2(x,u,r,d)
v=x(1)-u+abs(x(2))*x(2)/(2*r);
if v>=d
    f=sign(v);
else f=v/d;
end
