function [sys,x0,str,ts]=han_ctr(t,x,u,flag,aa,bet1,b,d)

switch flag
    case 0
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 3
        sys=mdlOutputs(t,x,u,aa,bet1,b,d);
    case {1,2,4,9}
        sys=[];
    otherwise 
        error(['Unhandled flag=',num2str(flag)]);
end
function [sys,x0,str,ts]=mdlInitializeSizes
    sizes=simsizes;
    sizes.NumContStates=0;
    sizes.NumDiscStates=0;
    sizes.NumOutputs=1;
    sizes.NumInputs=5;
    sizes.DirFeedthrough=1;
    sizes.NumSampleTimes=1;
    sys=simsizes(sizes);
    x0=[];
    str=[];
    ts=[-1 0];
function sys=mdlOutputs(t,x,u,aa,bet1,b,d)
    e1=u(1)-u(3); e2=u(2)-u(4);
    u0=bet1(1)*fal(e1,aa(1),d)+bet1(2)*fal(e2,aa(2),d);
    sys=u0-u(5)/b;
function f=fal(e,a,d)
    if abs(e)<d
        f=e*d^(a-1);
    else f=(abs(e))^a*sign(e);
    end
    
        
    
    