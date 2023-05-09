function tau_out = steering(Inputs)
% 操纵律，对控制器力矩进行分配
% 输入为freego 位置、姿态角、各关节角度以及飞轮
global freego ;
quad = freego;

%% input args
position = Inputs(1:3);
euler_angle = Inputs(4:6);
ql = Inputs(7:18);
theta_11 = ql(1); theta_12 = ql(2); theta_13 = ql(3);
theta_21 = ql(4); theta_22 = ql(5); theta_23 = ql(6);
theta_31 = ql(7); theta_32 = ql(8); theta_33 = ql(9);
theta_41 = ql(10); theta_42 = ql(11); theta_43 = ql(12);

qf = Inputs(19:21);

tau_b = Inputs(22:24);
%% euler
phi = euler_angle(1); theta = euler_angle(2); psi = euler_angle(3);

%%
m11 = freego.abadMass; m12 = freego.hipMass; m13 = freego.kneeMass;
m21 = freego.abadMass; m22 = freego.hipMass; m23 = freego.kneeMass;
m31 = freego.abadMass; m32 = freego.hipMass; m33 = freego.kneeMass;
m41 = freego.abadMass; m42 = freego.hipMass; m43 = freego.kneeMass;
mfw = freego.flyWheelmass;

r11 = freego.abadCOM; r12 = freego.hipCOM; r13 = freego.kneeCOM;
r21 = freego.abadCOM; r22 = freego.hipCOM; r23 = freego.kneeCOM;
r31 = freego.abadCOM; r32 = freego.hipCOM; r33 = freego.kneeCOM;
r41 = freego.abadCOM; r42 = freego.hipCOM; r43 = freego.kneeCOM;

rf1 = freego.flywheelXLocation;  rf2 = freego.flywheelYLocation;  rf3 = freego.flywheelZLocation; 
Cs = freego.flywheelCs;

%%
Ab_11 =Rotation_X(theta_11);         A11_b=Ab_11';
A11_12=Rotation_Y(theta_12);        A12_11=A11_12';
A12_13=Rotation_Y(theta_13);        A13_12=A12_13';
Ab_12 = Ab_11*A11_12;                A12_b=Ab_12'; 
Ab_13 = Ab_12*A12_13;                A13_b=Ab_13';
A11_13 = A11_12*A12_13;             A13_11=A11_13'; 

Ab_21 =Rotation_X(theta_21);         A21_b=Ab_21';
A21_22=Rotation_Y(theta_22);        A22_21=A21_22';
A22_23=Rotation_Y(theta_23);        A23_22=A22_23';
Ab_22 = Ab_21*A21_22;                A22_b=Ab_22'; 
Ab_23 = Ab_22*A22_23;                A23_b=Ab_23';
A21_23 = A21_22*A22_23;             A23_21=A21_23'; 

Ab_31 =Rotation_X(theta_31);         A31_b=Ab_31';
A31_32=Rotation_Y(theta_32);        A32_31=A31_32';
A32_33=Rotation_Y(theta_33);        A33_32=A32_33';
Ab_32 = Ab_31*A31_32;                A32_b=Ab_32'; 
Ab_33 = Ab_32*A32_33;                A33_b=Ab_33';
A31_33 = A31_32*A32_33;             A33_31=A31_33'; 

Ab_41 =Rotation_X(theta_41);         A41_b=Ab_41';
A41_42=Rotation_Y(theta_42);        A42_41=A41_42';
A42_43=Rotation_Y(theta_43);        A43_42=A42_43';
Ab_42 = Ab_41*A41_42;                A42_b=Ab_42'; 
Ab_43 = Ab_42*A42_43;                A43_b=Ab_43';
A41_43 = A41_42*A42_43;             A43_41=A41_43'; 

%%铰接点相对内接体体坐标系的位置矢量
rb = freego.bodyCOM';

rb_11 = [ freego.bodyLength, -freego.bodyWidth, 0]'/2;    
r11_12 = [ freego.hipLocation(1), -freego.hipLocation(2), 0]'; 
r12_13 = freego.kneeLocation';   
r11_13 = r11_12 + A11_12*r12_13; 
rb_12 = rb_11 + Ab_11*r11_12;    
rb_13 = rb_12 + Ab_12*r12_13;  

Xrb_11 = X_Matrix(rb_11);
Xr11_12 = X_Matrix(r11_12);
Xr12_13 = X_Matrix(r12_13);
Xr11_13 = X_Matrix(r11_13);
Xrb_12  = X_Matrix(rb_12);
Xrb_13  = X_Matrix(rb_13);

rb_21 = [ freego.bodyLength, freego.bodyWidth, 0]'/2;   
r21_22 = [ freego.hipLocation(1), freego.hipLocation(2), 0]';  
r22_23 = freego.kneeLocation';   
r21_23 = r21_22 + A21_22*r22_23; 
rb_22 = rb_21 + Ab_21*r21_22;
rb_23 = rb_22 + Ab_22*r22_23;

Xrb_21 = X_Matrix(rb_21);
Xr21_22 = X_Matrix(r21_22);
Xr22_23 = X_Matrix(r22_23);
Xr21_23 = X_Matrix(r21_23);
Xrb_22  = X_Matrix(rb_22);
Xrb_23  = X_Matrix(rb_23);

rb_31 = [ freego.bodyLength, -freego.bodyWidth, 0]'/2;
r31_32 = [ freego.hipLocation(1), -freego.hipLocation(2), 0]';
r32_33 = freego.kneeLocation';
r31_33 = r31_32 + A31_32*r32_33;
rb_32 = rb_31 + Ab_31*r31_32;
rb_33 = rb_32 + Ab_32*r32_33;

Xrb_31 = X_Matrix(rb_31);
Xr31_32 = X_Matrix(r31_32);
Xr32_33 = X_Matrix(r32_33);
Xr31_33 = X_Matrix(r31_33);
Xrb_32  = X_Matrix(rb_32);
Xrb_33  = X_Matrix(rb_33);

rb_41 = [ freego.bodyLength, freego.bodyWidth, 0]'/2;
r41_42 = [ freego.hipLocation(1), freego.hipLocation(2), 0]';
r42_43 = freego.kneeLocation';
r41_43 = r41_42 + A41_42*r42_43;
rb_42 = rb_41 + Ab_41*r41_42;
rb_43 = rb_42 + Ab_42*r42_43;

Xrb_41 = X_Matrix(rb_41);
Xr41_42 = X_Matrix(r41_42);
Xr42_43 = X_Matrix(r42_43);
Xr41_43 = X_Matrix(r41_43);
Xrb_42  = X_Matrix(rb_42);
Xrb_43  = X_Matrix(rb_43);

%% 静矩
S11 = m11 * r11;   X_S11 = X_Matrix(S11);
S12 = m12 * r12;   X_S12 = X_Matrix(S12);
S13 = m13 * r13;   X_S13 = X_Matrix(S13);

S21 = m21 * r21;   X_S21 = X_Matrix(S21);
S22 = m22 * r22;   X_S22 = X_Matrix(S22);
S23 = m23 * r23;   X_S23 = X_Matrix(S23);

S31 = m31 * r31;   X_S31 = X_Matrix(S31);
S32 = m32 * r32;   X_S32 = X_Matrix(S32);
S33 = m33 * r33;   X_S33 = X_Matrix(S33);

S41 = m41 * r41;   X_S41 = X_Matrix(S41);
S42 = m42 * r42;   X_S42 = X_Matrix(S42);
S43 = m43 * r43;   X_S43 = X_Matrix(S43);

S112 = m12*r11_12 + A11_12*S12; %S1j
S113 = m13*r11_13 + A11_13*S13;
S213 = m13*r12_13 + A12_13*S13; %S2j

S122 = m22*r21_22 + A21_22*S22; %S1j
S123 = m23*r21_23 + A21_23*S23;
S223 = m23*r22_23 + A22_23*S23; %S2j

S132 = m32*r31_32 + A31_32*S32; %S1j
S133 = m33*r31_33 + A31_33*S33;
S233 = m33*r32_33 + A32_33*S33; %S2j

S142 = m42*r41_42 + A41_42*S42; %S1j
S143 = m43*r41_43 + A41_43*S43;
S243 = m43*r42_43 + A42_43*S43; %S2j

S11t = S11 + S112 + S113;  X_S11t = X_Matrix(S11t);
S12t = S12 + S213;         X_S12t = X_Matrix(S12t);
S13t = S13;                X_S13t = X_Matrix(S13t);

S21t = S21 + S122 + S123;  X_S21t = X_Matrix(S21t);
S22t = S22 + S223;         X_S22t = X_Matrix(S22t);
S23t = S23;                X_S23t = X_Matrix(S23t);

S31t = S31 + S132 + S133;  X_S31t = X_Matrix(S31t);
S32t = S32 + S233;         X_S32t = X_Matrix(S32t);
S33t = S33;                X_S33t = X_Matrix(S33t);

S41t = S41 + S142 + S143;  X_S41t = X_Matrix(S41t);
S42t = S42 + S243;         X_S42t = X_Matrix(S42t);
S43t = S43;                X_S43t = X_Matrix(S43t);

%%
Ib = freego.bodyRotationalInertia;

I11 = flipInertialY(freego.abadRotationalInertia);
I12 = flipInertialY(freego.hipRotationalInertia);
I13 = flipInertialY(freego.kneeRotationalInertia);

I21 = freego.abadRotationalInertia;
I22 = freego.hipRotationalInertia;
I23 = freego.kneeRotationalInertia;

I31 = flipInertialY(freego.abadRotationalInertia);
I32 = flipInertialY(freego.hipRotationalInertia);
I33 = flipInertialY(freego.kneeRotationalInertia);

I41 = freego.abadRotationalInertia;
I42 = freego.hipRotationalInertia;
I43 = freego.kneeRotationalInertia;

Im = freego.flyWheelIm;
deltaIt = mfw*(rf1'*rf1*eye(3) - rf1*rf1') + mfw*(rf2'*rf2*eye(3) - rf2*rf2') + mfw*(rf3'*rf3*eye(3) - rf3*rf3') + Cs*Im*Cs';

Io11 = m11*(Xrb_11')*Xrb_11 + (Xrb_11')*Ab_11*X_S11*A11_b + Ab_11*(X_S11')*A11_b*Xrb_11 + Ab_11*I11*A11_b;
Io12 = m12*(Xrb_12')*Xrb_12 + (Xrb_12')*Ab_12*X_S12*A12_b + Ab_12*(X_S12')*A12_b*Xrb_12 + Ab_12*I12*A12_b;
Io13 = m13*(Xrb_13')*Xrb_13 + (Xrb_13')*Ab_13*X_S13*A13_b + Ab_13*(X_S13')*A13_b*Xrb_13 + Ab_13*I13*A13_b;

Io21 = m21*(Xrb_21')*Xrb_21 + (Xrb_21')*Ab_21*X_S21*A21_b + Ab_21*(X_S21')*A21_b*Xrb_21 + Ab_21*I21*A21_b;
Io22 = m22*(Xrb_22')*Xrb_22 + (Xrb_22')*Ab_22*X_S22*A22_b + Ab_22*(X_S22')*A22_b*Xrb_22 + Ab_22*I22*A22_b;
Io23 = m23*(Xrb_23')*Xrb_23 + (Xrb_23')*Ab_23*X_S23*A23_b + Ab_23*(X_S23')*A23_b*Xrb_23 + Ab_23*I23*A23_b;

Io31 = m31*(Xrb_31')*Xrb_31 + (Xrb_31')*Ab_31*X_S31*A31_b + Ab_31*(X_S31')*A31_b*Xrb_31 + Ab_31*I31*A31_b;
Io32 = m32*(Xrb_32')*Xrb_32 + (Xrb_32')*Ab_32*X_S32*A32_b + Ab_32*(X_S32')*A32_b*Xrb_32 + Ab_32*I32*A32_b;
Io33 = m33*(Xrb_33')*Xrb_33 + (Xrb_33')*Ab_33*X_S33*A33_b + Ab_33*(X_S33')*A33_b*Xrb_33 + Ab_33*I33*A33_b;

Io41 = m41*(Xrb_41')*Xrb_41 + (Xrb_41')*Ab_41*X_S41*A41_b + Ab_41*(X_S41')*A41_b*Xrb_41 + Ab_41*I41*A41_b;
Io42 = m42*(Xrb_42')*Xrb_42 + (Xrb_42')*Ab_42*X_S42*A42_b + Ab_42*(X_S42')*A42_b*Xrb_42 + Ab_42*I42*A42_b;
Io43 = m43*(Xrb_43')*Xrb_43 + (Xrb_43')*Ab_43*X_S43*A43_b + Ab_43*(X_S43')*A43_b*Xrb_43 + Ab_43*I43*A43_b;

It = Ib + Io11 + Io12 + Io13 + Io21 + Io22 + Io23 + Io31 + Io32 + Io33 + Io41 + Io42 + Io43 + deltaIt;  % It

I112 = m12*(Xr11_12')*Xr11_12 + (Xr11_12')*A11_12*X_S12*A12_11 + A11_12*(X_S12')*A12_11*Xr11_12 + A11_12*I12*A12_11;  % I1j
I113 = m13*(Xr11_13')*Xr11_13 + (Xr11_13')*A11_13*X_S13*A13_11 + A11_13*(X_S13')*A13_11*Xr11_13 + A11_13*I13*A13_11;
I213 = m13*(Xr12_13')*Xr12_13 + (Xr12_13')*A12_13*X_S13*A13_12 + A12_13*(X_S13')*A13_12*Xr12_13 + A12_13*I13*A13_12; % I2j

I122 = m22*(Xr21_22')*Xr21_22 + (Xr21_22')*A21_22*X_S22*A22_21 + A21_22*(X_S22')*A22_21*Xr21_22 + A21_22*I22*A22_21;  % I1j
I123 = m23*(Xr21_23')*Xr21_23 + (Xr21_23')*A21_23*X_S23*A23_21 + A21_23*(X_S23')*A23_21*Xr21_23 + A21_23*I23*A23_21;
I223 = m23*(Xr22_23')*Xr22_23 + (Xr22_23')*A22_23*X_S23*A23_22 + A22_23*(X_S23')*A23_22*Xr22_23 + A22_23*I23*A23_22; % I2j

I132 = m32*(Xr31_32')*Xr31_32 + (Xr31_32')*A31_32*X_S32*A32_31 + A31_32*(X_S32')*A32_31*Xr31_32 + A31_32*I32*A32_31;  % I1j
I133 = m33*(Xr31_33')*Xr31_33 + (Xr31_33')*A31_33*X_S33*A33_31 + A31_33*(X_S33')*A33_31*Xr31_33 + A31_33*I33*A33_31;
I233 = m33*(Xr32_33')*Xr32_33 + (Xr32_33')*A32_33*X_S33*A33_32 + A32_33*(X_S33')*A33_32*Xr32_33 + A32_33*I33*A33_32; % I2j

I142 = m42*(Xr41_42')*Xr41_42 + (Xr41_42')*A41_42*X_S42*A42_41 + A41_42*(X_S42')*A42_41*Xr41_42 + A41_42*I42*A42_41;  % I1j
I143 = m43*(Xr41_43')*Xr41_43 + (Xr41_43')*A41_43*X_S43*A43_41 + A41_43*(X_S43')*A43_41*Xr41_43 + A41_43*I43*A43_41;
I243 = m43*(Xr42_43')*Xr42_43 + (Xr42_43')*A42_43*X_S43*A43_42 + A42_43*(X_S43')*A43_42*Xr42_43 + A42_43*I43*A43_42; % I2j

I11t = I11 + I112 + I113; %Ijt
I12t = I12 + I213;
I13t = I13;

I21t = I21 + I122 + I123;
I22t = I22 + I223;
I23t = I23;

I31t = I31 + I132 + I133;
I32t = I32 + I233;
I33t = I33;

I41t = I41 + I142 + I143;
I42t = I42 + I243;
I43t = I43;

I11_o = I11t + A11_b*(Xrb_11')*Ab_11*X_S11t; %Ij_o
I12_o = I12t + A12_b*(Xrb_12')*Ab_12*X_S12t; 
I13_o = I13t + A13_b*(Xrb_13')*Ab_13*X_S13t; 

I21_o = I21t + A21_b*(Xrb_21')*Ab_21*X_S21t; %Ij_o
I22_o = I22t + A22_b*(Xrb_22')*Ab_22*X_S22t; 
I23_o = I23t + A23_b*(Xrb_23')*Ab_23*X_S23t; 

I31_o = I31t + A31_b*(Xrb_31')*Ab_31*X_S31t; %Ij_o
I32_o = I32t + A32_b*(Xrb_32')*Ab_32*X_S32t; 
I33_o = I33t + A33_b*(Xrb_33')*Ab_33*X_S33t; 

I41_o = I41t + A41_b*(Xrb_41')*Ab_41*X_S41t; %Ij_o
I42_o = I42t + A42_b*(Xrb_42')*Ab_42*X_S42t; 
I43_o = I43t + A43_b*(Xrb_43')*Ab_43*X_S43t; 

%%
M11 = It;
M12 = Cs*Im;
% M13 = Ab_11*I11_o*[1 0 0]';  M16 = Ab_21*I21_o*[1 0 0]';  M19  = Ab_31*I31_o*[1 0 0]';  M112 = Ab_41*I41_o*[1 0 0]';  %胯
% M14 = Ab_12*I12_o*[0 1 0]';  M17 = Ab_22*I22_o*[0 1 0]';  M110 = Ab_32*I32_o*[0 1 0]';  M113 = Ab_42*I42_o*[0 1 0]';  %大腿
% M15 = Ab_13*I13_o*[0 1 0]';  M18 = Ab_23*I23_o*[0 1 0]';  M111 = Ab_33*I33_o*[0 1 0]';  M114 = Ab_43*I43_o*[0 1 0]';  %小腿

M13 = Ab_11*I11_o;  M16 = Ab_21*I21_o;  M19  = Ab_31*I31_o;  M112 = Ab_41*I41_o;  %胯
M14 = Ab_12*I12_o;  M17 = Ab_22*I22_o;  M110 = Ab_32*I32_o;  M113 = Ab_42*I42_o;  %大腿
M15 = Ab_13*I13_o;  M18 = Ab_23*I23_o;  M111 = Ab_33*I33_o;  M114 = Ab_43*I43_o;  %小腿

M22 = Cs*Im;

M33 = I11t;
M44 = I12t;
M55 = I13t;

M66 = I21t;
M77 = I22t;
M88 = I23t;

M99 = I31t;
M1010 = I32t;
M1111 = I33t;

M1212 = I41t;
M1313 = I42t;
M1414 = I43t;

%% tau
distr_m1 = [0.1 0 0; 0 0.1 0; 0 0 0.5]; % 主要给飞轮分配，z轴多一些
distr_m2 = [1 0 0; 0 0.5 0; 0 0 0.5]; %给胯分配，x轴多一些
% distr_m3 = [0.5 0 0; 0 2 0; 0 0 0.5]; %给大腿分配，y轴多一些
distr_m3 = [0.1 0 0; 0 0.5 0; 0 0 0.1]; %给大腿分配，y轴多一些

% tau_fw = (M12/M22)\(distr_m1*tau_b);
tau_fw = tau_b;

% tau_leg(1) = [1 0 0]*((M13/M33)\(distr_m2*tau_b)); %FR
% tau_leg(2) = [0 1 0]*((M14/M44)\(distr_m3*tau_b));
% tau_leg(3) = 0; %shank
% 
% tau_leg(4) = [1 0 0]*((M16/M66)\distr_m2*tau_b); %FL
% tau_leg(5) = [0 1 0]*((M17/M77)\distr_m3*tau_b);
% tau_leg(6) = 0; %shank
% 
% tau_leg(7) = [1 0 0]*((M19/M99)\distr_m2*tau_b); %HR
% tau_leg(8) = [0 1 0]*((M110/M1010)\distr_m3*tau_b);
% tau_leg(9) = 0; %shank
% 
% tau_leg(10) = [1 0 0]*((M112/M1212)\distr_m2*tau_b); %HL
% tau_leg(11) = [0 1 0]*((M113/M1313)\distr_m3*tau_b);
% tau_leg(12) = 0; %shank

tau_leg(1) = [1 0 0]*(distr_m2*tau_b); %FR
tau_leg(2) = [0 1 0]*(distr_m3*tau_b);
tau_leg(3) = 0; %shank

tau_leg(4) = [1 0 0]*(distr_m2*tau_b); %FL
tau_leg(5) = [0 1 0]*(distr_m3*tau_b);
tau_leg(6) = 0; %shank

tau_leg(7) = [1 0 0]*(distr_m2*tau_b); %HR
tau_leg(8) = [0 1 0]*(distr_m3*tau_b);
tau_leg(9) = 0; %shank

tau_leg(10) = [1 0 0]*(distr_m2*tau_b); %HL
tau_leg(11) = [0 1 0]*(distr_m3*tau_b);
tau_leg(12) = 0; %shank

tau_out = [tau_fw', tau_leg]';

%%
function matrix = flipInertialY(V)

matrix = [V(1,1) -V(1,2) V(1,3); -V(2,1) V(2,2) -V(2,3); V(3,1) -V(3,2) V(3,3)];

end
end

