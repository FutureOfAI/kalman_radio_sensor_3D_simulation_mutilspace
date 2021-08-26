close all;
clear all;
clc;

xr1 = 300/20;% in meter
yr1 = 100/20;% in meter
zr1 = 150/20;% in meter
xv1 = 30/20;% in meter
yv1 = -20/20;% in meter
zv1 = 20/20;% in meter

xr2 = 400/20;
yr2 = 150/20;
zr2 = 200/20;
xv2 = -30/20;
yv2 = 10/20;
zv2 = 20/20;

xr3 = 300/20;% in meter
yr3 = 500/20;% in meter
zr3 = 200/20;
xv3 = 10/20;% in meter
yv3 = -20/20;% in meter
zv3 = 10/20;

xr4 = 350/20;
yr4 = 200/20;
zr4 = 100/20;% in meter
xv4 = 10/20;
yv4 = 20/20;
zv4 = 30/20;% in meter


x_p_N = 600/20;
x_v_N = -20/20;
y_p_N = 650/20;
y_v_N = 15/20;
z_p_N = 550/20;
z_v_N =40/20;

R1 = sqrt((xr1-x_p_N)^2+(yr1-y_p_N)^2+(zr1-z_p_N)^2);
R2 = sqrt((xr2-x_p_N)^2+(yr2-y_p_N)^2+(zr2-z_p_N)^2);
R3 = sqrt((xr3-x_p_N)^2+(yr3-y_p_N)^2+(zr3-z_p_N)^2);
R4 = sqrt((xr4-x_p_N)^2+(yr4-y_p_N)^2+(zr4-z_p_N)^2);

Rv1 = sqrt((xv1-x_v_N)^2+(yv1-y_v_N)^2+(zv1-z_v_N)^2);
Rv2 = sqrt((xv2-x_v_N)^2+(yv2-y_v_N)^2+(zv2-z_v_N)^2);
Rv3 = sqrt((xv3-x_v_N)^2+(yv3-y_v_N)^2+(zv3-z_v_N)^2);
Rv4 = sqrt((xv4-x_v_N)^2+(yv4-y_v_N)^2+(zv4-z_v_N)^2);

% HO
k=1;
a1 = [(x_p_N(k)-xr1)/R1(k),(y_p_N(k)-yr1)/R1(k),(z_p_N(k)-zr1)/R1(k)]';
a2 = [(x_p_N(k)-xr2)/R2(k),(y_p_N(k)-yr2)/R2(k),(z_p_N(k)-zr2)/R2(k)]';
a3 = [(x_p_N(k)-xr3)/R3(k),(y_p_N(k)-yr3)/R3(k),(z_p_N(k)-zr3)/R3(k)]';
a4 = [(x_p_N(k)-xr4)/R4(k),(y_p_N(k)-yr4)/R4(k),(z_p_N(k)-zr4)/R4(k)]';
b1 = [(x_v_N(k)-xv1)/R1(k),(y_v_N(k)-yv1)/R1(k),(z_v_N(k)-zv1)/R1(k)]'-Rv1(k)/R1(k)^2*[(x_p_N(k)-xr1),(y_p_N(k)-yr1),(z_p_N(k)-zr1)]';
b2 = [(x_v_N(k)-xv2)/R2(k),(y_v_N(k)-yv2)/R2(k),(z_v_N(k)-zv2)/R2(k)]'-Rv2(k)/R2(k)^2*[(x_p_N(k)-xr2),(y_p_N(k)-yr2),(z_p_N(k)-zr1)]';
b3 = [(x_v_N(k)-xv3)/R3(k),(y_v_N(k)-yv3)/R3(k),(z_v_N(k)-zv3)/R3(k)]'-Rv3(k)/R3(k)^2*[(x_p_N(k)-xr3),(y_p_N(k)-yr3),(z_p_N(k)-zr1)]';
b4 = [(x_v_N(k)-xv4)/R4(k),(y_v_N(k)-yv4)/R4(k),(z_v_N(k)-zv4)/R4(k)]'-Rv4(k)/R4(k)^2*[(x_p_N(k)-xr4),(y_p_N(k)-yr4),(z_p_N(k)-zr1)]';
C1_1 = [-a1',a2',zeros(1,6)];
C1_2 = [-a1',zeros(1,3),a3',zeros(1,3)];
C1_3 = [-a1',zeros(1,6),a4'];
C2_1 = [-b1',b2',zeros(1,6)];
C2_2 = [-b1',zeros(1,3),b3',zeros(1,3)];
C2_3 = [-b1',zeros(1,6),b4'];
dp = [C1_1;C1_2;C1_3;C2_1;C2_2;C2_3]*eye(12,3); % u
dv = [zeros(3,3);[C1_1;C1_2;C1_3]*eye(12,3)]; % u_h

dq = -[C1_1;C1_2;C1_3;C2_1;C2_2;C2_3]; % s
dw = [zeros(3,12);-[C1_1;C1_2;C1_3]]; % s_h

sugma2 = 10^4;

R_l = sugma2*eye(3,3);
R_l(1,2) = 0.5*sugma2;
R_l(1,3) = 0.5*sugma2;
R_l(2,1) = 0.5*sugma2;
R_l(2,3) = 0.5*sugma2;
R_l(3,1) = 0.5*sugma2;
R_l(3,2) = 0.5*sugma2;

R_f = 0.1*R_l;

Q_a = [R_l, zeros(3,3)
       zeros(3,3), R_f];

R_s =sugma2*eye(12,12);
R_s_h = 0.5*R_s;

Q_b = [R_s, zeros(12,12)
       zeros(12,12), R_s_h];

X = [dp,dv]'*inv(Q_a)*[dp,dv];
Y = [dp,dv]'*inv(Q_a)*[dq,dw];
Z = [dq,dw]'*inv(Q_a)*[dq,dw] + inv(Q_b);

% cov_theta(:,:,k) = inv(X)+inv(X)*Y*inv(Z-Y'*inv(X)*Y)*Y'*inv(X);

FIM = [X, Y
       Y', Z];

CRLB = trace(FIM)



