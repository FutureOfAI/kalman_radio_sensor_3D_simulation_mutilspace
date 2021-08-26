%% 
% program name uwb_distribution.m
% evaluate the impact of uwb errors in 3D positioning
% 2020-09-24
close all;
clear all;
clc;

r2d = (180/pi);
d2r = (pi/180);
dt = 0.1;
fz = 1*0.005;
T = 4*(1/fz);
t0 = 0:dt:T;
t0_1 = t0';
n = length(t0);
m = size(t0_1);
radius = 1*8;
gama_0 = 1*45*d2r;
apha_0 = 1*45*d2r;
ft =1*0.01;
wt = 2*pi*ft;
% length = 250;
% width = 250;
% n = length*width;
% m = size(n);
% ======================================================
% Define four UWB sensors locations
% =====================================================
xr1 = 0;% in meter
yr1 = 0;% in meter
zr1 = 20;% in meter
xr2 = 0;
yr2 = 20;
zr2 = 20;

psi = 90*d2r;
r3 = 20;
xr3 = r3*sin(psi);% in meter
yr3 = r3*cos(psi);% in meter
zr3 = 20;

phi = 0*d2r;
beta = 45*d2r;
alpha = 45*d2r;
r4 = 20;
xr4 = r4*sin(phi)*cos(beta);
yr4 = r4*sin(phi)*cos(alpha);
zr4 = 20-r4*cos(phi);% in meter

% xr4 = 20;
% yr4 = 20;
% zr4 = 19.9;% in meter

H = [xr2-xr1, yr2-yr1, zr2-zr1
    xr3-xr1, yr3-yr1, zr3-zr1
    xr4-xr1, yr4-yr1, zr4-zr1];

q(1) = xr1^2+yr1^2+zr1^2;
q(2) = xr2^2+yr2^2+zr2^2;
q(3) = xr3^2+yr3^2+zr3^2;
q(4) = xr4^2+yr4^2+zr4^2;

% X = linspace(-50,100,20);
% Y = linspace(-50,100,20);
% Z = linspace(-50,100,20);
% [X2,Y2] = meshgrid(X,Y);
% Z2 = meshgrid(Z);
% for k = 1:20
%     for j = 1:20
%         for i = 1:20
%             x_p_N((k-1)*20*20+(j-1)*20+i) = X2(j,i);
%             y_p_N((k-1)*20*20+(j-1)*20+i) = Y2(j,i);
%             z_p_N((k-1)*20*20+(j-1)*20+i) = Z(k);
%         end
%     end
% end


% [X,Y,Z] = sphere(19);
% r = 50;
% X2 = X * r;
% Y2 = Y * r;
% Z2 = Z * r;
% for k = 1:20
%     for j = 1:20
%         for i = 1:20
%             x_p_N((k-1)*20*20+(j-1)*20+i) = X2(j,i);
%             y_p_N((k-1)*20*20+(j-1)*20+i) = Y2(j,i);
%             z_p_N((k-1)*20*20+(j-1)*20+i) = Z2(j,i);
%         end
%     end
% end


[x_p_N,x_v_N,y_p_N,y_v_N,z_p_N,z_v_N] = trajectory3d_line_vel(radius,wt,t0);


% =====================================================================================
% Define "4 radio sensor" parameters (noise)
% =====================================================================================
radiosensor_err_factor = 1.0;
sigma = 0.1;
sig_x_r=radiosensor_err_factor*sigma;              % radio sensor measurement noise in meters x-direction
sig_y_r=radiosensor_err_factor*sigma;              % radio sensor measurement noise in meters y-direction
sig_p_r=radiosensor_err_factor*sigma;
sig_q_r=radiosensor_err_factor*sigma;
%======================================================================================
[R1m,R2m,R3m,R4m,nvx_r,nvy_r,nvp_r,nvq_r] = radio_sensor3d_m_4_v2(xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,x_p_N,y_p_N,z_p_N,sig_x_r,sig_y_r,sig_p_r,sig_q_r,n-1,m,0);%4 four radio sensors
[R1,R2,R3,R4] = radio_sensor3d_4_v2(xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,x_p_N,y_p_N,z_p_N,n-1,m);

% LS
for k=1:n
    b = [R1m(k)^2-R2m(k)^2-q(1)+q(2)
         R1m(k)^2-R3m(k)^2-q(1)+q(3)
         R1m(k)^2-R4m(k)^2-q(1)+q(4)];
    r(:,k) = 0.5*inv(H'*H)*H'*b;
%     r(:,k) = 0.5*inv(H)*b;
end

LS_delta_P = [x_p_N-r(1,:); y_p_N-r(2,:); z_p_N-r(3,:)];
err1 = LS_delta_P(1,:).^2+LS_delta_P(2,:).^2+LS_delta_P(3,:).^2;
% C = sort(err1);
% N = length(C);
% number = fix(0.9973*N);
% C1 = sqrt(C(number))
LS = mean(sqrt(err1))
std_LS = std(err1);
% [LS,std_LS]

% f_distr = (1/(sin(psi)^2*cos(phi)^2))*((1/(yr2^2))+(1/(r3^2))+(1/(r4^2)))

for k=1:n-1
    delta_R = [-2*R1m(k)*nvx_r(k)+2*R2m(k)*nvy_r(k)
                -2*R1m(k)*nvx_r(k)+2*R3m(k)*nvy_r(k)
                -2*R1m(k)*nvx_r(k)+2*R4m(k)*nvy_r(k)];
    delta_P(:,k) = 0.5*inv(H)*delta_R;
    conv_dR(:,:,k) = [R1m(k)^2*nvx_r(k)^2+R2m(k)^2*nvy_r(k)^2, R1m(k)^2*nvx_r(k)^2, R1m(k)^2*nvx_r(k)^2;
                    R1m(k)^2*nvx_r(k)^2, R1m(k)^2*nvx_r(k)^2+R3m(k)^2*nvy_r(k)^2, R1m(k)^2*nvx_r(k)^2;
                    R1m(k)^2*nvx_r(k)^2, R1m(k)^2*nvx_r(k)^2, R1m(k)^2*nvx_r(k)^2+R4m(k)^2*nvy_r(k)^2];
    E_dp(:,:,k) = inv(H)*conv_dR(:,:,k)*inv(H)';
end

for k=1:n-1
   plot_E_dp(1,k) =  E_dp(1,1,k);
   plot_E_dp(2,k) =  E_dp(2,2,k);
   plot_E_dp(3,k) =  E_dp(3,3,k);
end

err = abs(plot_E_dp(1,:)')+abs(plot_E_dp(2,:)')+abs(plot_E_dp(3,:)');

for k = 1:20
    for j = 1:20
       for i = 1:20
            zz(i,j) = sqrt(err((k-1)*20*20+(j-1)*20+i));
       end
    end
end

% B = sort(err);
% N = length(B);
% number = fix(0.9973*N);
% B1 = sqrt(B(number))
% CRLB = sqrt(mean(err))

% % Chan method for 4 anchors
% Ga_p=[1,0,0;0,1,0;0,0,1;1,1,1];
% for k = 1:n-1
%     Q = [nvx_r(k),0,0;0,nvy_r(k),0;0,0,nvq_r(k)];
%     B_2p = [x_p_N(k)-xr1,0,0;0,y_p_N(k)-yr1,0;0,0,z_p_N(k)-zr1];
%     B_p = [x_p_N(k)-xr1,0,0,0;0,y_p_N(k)-yr1,0,0;0,0,z_p_N(k)-zr1,0;0,0,0,R1(k)];
%     G_a = -[xr2-xr1,yr2-yr1,zr2-zr1,R2(k)-R1(k);xr3-xr1,yr3-yr1,zr3-zr1,R3(k)-R1(k);xr4-xr1,yr4-yr1,zr4-zr1,R4(k)-R1(k)];
%     B = [R2(k),0,0;0,R3(k),0;0,0,R4(k)];
%     Phi(:,:,k) = inv(B_2p*Ga_p'*inv(B_p)*G_a'*inv(B)*inv(Q)*inv(B)*G_a*inv(B_p)*Ga_p*B_2p);
% end
% for k=1:n-1
%    plot_Phi(1,k) =  Phi(1,1,k);
%    plot_Phi(2,k) =  Phi(2,2,k);
%    plot_Phi(3,k) =  Phi(3,3,k);
% end
% 
% err1 = abs(plot_Phi(1,:)')+abs(plot_Phi(2,:)')+abs(plot_Phi(3,:)');
% Chan = sqrt(mean(err1));
% std_Chan = std(err1);

% % CRLB for 4 anchors
% for k = 1:n-1
%     Q = [nvx_r(k),0,0;0,nvy_r(k),0;0,0,nvp_r(k)];
%     Gt11 = (xr1-x_p_N(k))/R1(k)-(xr2-x_p_N(k))/R2(k);
%     Gt12 = (yr1-y_p_N(k))/R1(k)-(yr2-y_p_N(k))/R2(k);
% %     Gt13 = (zr1-z_p_N(k))/R1(k)-(zr2-z_p_N(k))/R2(k);
%     Gt21 = (xr1-x_p_N(k))/R1(k)-(xr3-x_p_N(k))/R3(k);
%     Gt22 = (yr1-y_p_N(k))/R1(k)-(yr3-y_p_N(k))/R3(k);
% %     Gt23 = (zr1-z_p_N(k))/R1(k)-(zr3-z_p_N(k))/R3(k);
%     Gt31 = (xr1-x_p_N(k))/R1(k)-(xr4-x_p_N(k))/R4(k);
%     Gt32 = (yr1-y_p_N(k))/R1(k)-(yr4-y_p_N(k))/R4(k);
% %     Gt33 = (zr1-z_p_N(k))/R1(k)-(zr4-z_p_N(k))/R4(k);
%     Gt = [Gt11,Gt12;Gt21,Gt22;Gt31,Gt32];
%     Gl = [xr2-xr1,R2(k)-R1(k);xr3-xr1,R3(k)-R1(k);xr4-xr1,R4(k)-R1(k)];
%     Phi0(:,:,k)=inv(Gt'*inv(Q)*Gl);
% end
% 
% for k=1:n-1
%    plot_Phi0(1,k) =  Phi0(1,1,k);
%    plot_Phi0(2,k) =  Phi0(2,2,k);
% %    plot_Phi0(3,k) =  Phi0(3,3,k);
%    if abs(plot_Phi0(1,k))>2
%        plot_Phi0(1,k) = 0;
%    end
%    if abs(plot_Phi0(2,k))>2
%        plot_Phi0(2,k) = 0;
%    end    
% %    if abs(plot_Phi0(3,k))>2
% %        plot_Phi0(3,k) = 0;
% %    end     
% end
% 
% err2 = abs(plot_Phi0(1,:)')+abs(plot_Phi0(2,:)');
% CRLB = sqrt(mean(err2));
% std_CRLB = std(err2);

% % Monica
% zeta2 = 20^2; % assume the hight is between anchor1 and anchor4
% omega2 = (yr2-yr1)^2;
% eta2 = 4*(omega2/4+zeta2);
% lemda2 = omega2/eta2;
% b = 19*lemda2/70+1/3;
% d = -lemda2/24;
% beta=solve('x^3+0.3876*x^2-0.0083=0','x');
% betaS = double(max([beta(1,1),beta(2,1),beta(3,1)]));
% detaS = sqrt(betaS*(omega2+4*zeta2));
% C(1) = 2.5*detaS^2*eta2^2*omega2+26*detaS^4*eta2*omega2+146*detaS^6*omega2+1.25*detaS^4*eta2^2+13.5*detaS^6*eta2+58.5*detaS^8;
% C(2) = -(468*detaS^5*omega2+60*detaS^3*eta2*omega2+3*detaS*eta2^2*omega2+15*detaS^5*eta2+108*detaS^7);
% C(3) = eta2^2*omega2+56*detaS^2*eta2*omega2+606*detaS^4*omega2+5*detaS^4*eta2+81*detaS^6;
% C(4) = -(24*detaS*eta2*omega2+408*detaS^3*omega2+30*detaS^5);
% C(5) = 5*detaS^4+158*detaS^2*omega2+4*eta2*omega2;
% C(6) = -36*detaS*omega2;
% C(7) = 4*omega2;
% x_avg = mean(x_p_N)/9;

% for k = 1:n-1
%     err3(k) = nvx_r(k)^2/(4*detaS^4*eta2*omega2)*sum(C)*x_avg;
% end
% Monica = sqrt(mean(err3));
% std_Monica = std(err3);
% [Monica,std_Monica]

% Simplify upper bound
for k = 1:n-1
    err4(k) = nvx_r(k)^2/(sin(psi)^2*cos(phi)^2)*((3*R1(k)^2+R2(k)^2)/yr2^2+(3*R1(k)^2+R3(k)^2)/r3^2+(3*R1(k)^2+R4(k)^2)/r4^2);
end
Simplify = sqrt(mean(err4));
std_Simplify = std(err4);
% [Simplify,std_Simplify]
% GDOP
for k=1:n-1
    p1 = sqrt((xr1-xr2)^2+(yr1-yr2)^2+(zr1-zr2)^2);
    p2 = sqrt((xr1-xr3)^2+(yr1-yr3)^2+(zr1-zr3)^2);
    p3 = sqrt((xr1-xr4)^2+(yr1-yr4)^2+(zr1-zr4)^2);
    p4 = sqrt((xr2-xr3)^2+(yr2-yr3)^2+(zr2-zr3)^2);
    p5 = sqrt((xr2-xr4)^2+(yr2-yr4)^2+(zr2-zr4)^2);
    p6 = sqrt((xr3-xr4)^2+(yr3-yr4)^2+(zr3-zr4)^2);
    pm_1 = R1(k)^2+R2(k)^2-p1^2;
    pm_2 = 2*R1(k)*R2(k);
    G1(k) = 2/(1-(pm_1/pm_2)^2)*nvx_r(k)^2;
    pm_1 = R1(k)^2+R3(k)^2-p2^2;
    pm_2 = 2*R1(k)*R3(k);
    G2(k) = 2/(1-(pm_1/pm_2)^2)*nvy_r(k)^2;
    pm_1 = R1(k)^2+R4(k)^2-p3^2;
    pm_2 = 2*R1(k)*R4(k);
    G3(k) = 2/(1-(pm_1/pm_2)^2)*nvp_r(k)^2;    
    pm_1 = R2(k)^2+R3(k)^2-p4^2;
    pm_2 = 2*R2(k)*R3(k);    
    G4(k) = 2/(1-(pm_1/pm_2)^2)*nvx_r(k)^2;
    pm_1 = R2(k)^2+R4(k)^2-p5^2;
    pm_2 = 2*R2(k)*R4(k);    
    G5(k) = 2/(1-(pm_1/pm_2)^2)*nvy_r(k)^2;    
    pm_1 = R3(k)^2+R4(k)^2-p6^2;
    pm_2 = 2*R3(k)*R4(k);    
    G6(k) = 2/(1-(pm_1/pm_2)^2)*nvp_r(k)^2;       
    if k>1
        if G1(k)>2
            G1(k) = G1(k-1);
        end
        if G2(k)>2
            G2(k) = G2(k-1);
        end
        if G3(k)>2
            G3(k) = G3(k-1);
        end          
        if G4(k)>2
            G4(k) = G4(k-1);
        end
        if G5(k)>2
            G5(k) = G5(k-1);
        end
        if G6(k)>2
            G6(k) = G6(k-1);
        end        
    else
        G1(k) = 0;
        G2(k) = 0;
        G3(k) = 0;        
        G4(k) = 0;
        G5(k) = 0;
        G6(k) = 0;
    end
    err6(k) = G1(k)+G2(k)+G3(k)+G4(k)+G5(k)+G6(k);
end
GDOP = sqrt(mean(err6));
std_GDOP = std(err6);
% [GDOP,std_GDOP]

% HO
xv1 = 1.5;% in meter
yv1 = -1;% in meter
zv1 = 1;% in meter
xv2 = -1.5;
yv2 = 0.5;
zv2 = 1;
xv3 = 0.5;% in meter
yv3 = -1;% in meter
zv3 = 0.5;
xv4 = 0.5;
yv4 = 1;
zv4 = 1.5;% in meter
[Rv1,Rv2,Rv3,Rv4] = radio_sensor3d_4_v2(xv1,yv1,zv1,xv2,yv2,zv2,xv3,yv3,zv3,xv4,yv4,zv4,x_p_N,y_p_N,z_p_N,n-1,m);

for k=1:n-1
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
    
    FIM = [X, Y
           Y', Z];

    err7(k) = trace(FIM);
end

HO = sqrt(mean(err7));
std_HO = std(err7);

% avg = [LS,Simplify,GDOP,CRLB]
% mstd = [std_LS,std_Simplify,std_GDOP,std_CRLB]


% figure (1)
% plot3(x_p_N,y_p_N,z_p_N,xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*','linewidth',2)
% xlabel('x(m)','FontSize',14)
% ylabel('y(m)','FontSize',14)
% zlabel('z(m)','FontSize',14)
% title('3D Simulation','FontSize',18);
% set(gca, 'XTick', 0:5:20, 'FontSize', 14)
% set(gca, 'YTick', 0:5:20, 'FontSize', 14)
% set(gca, 'ZTick', 0:5:20, 'FontSize', 14)
% % an = {' q_1',' q_2',' q_3',' q_4'};
% % xt=[xr1,xr2,xr3,xr4];
% % yt=[yr1,yr2,yr3,yr4];
% % zt=[zr1,zr2,zr3,zr4];
% % text(xt,yt,zt,an,'fontsize',18,'linewidth',2);
% % view(30,30)
% % axis equal
% grid

% figure (6)
% plot3(r(1,:),r(2,:),r(3,:),xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*','linewidth',2)
% 
% PSI = [10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160];
% error2_5_LS = [2.593	1.403	1.058	0.923	0.872	0.847	0.838	0.835	0.837	0.903	0.957	1.043	1.177	1.400	1.805	2.622];
% error2_20_LS = [0.797	0.427	0.318	0.270	0.252	0.240	0.238	0.239	0.239	0.259	0.281	0.313	0.373	0.467	0.627	0.961];
% error2_5n_LS = [4.936	2.725	2.076	1.794	1.676	1.639	1.621	1.614	1.591	1.719	1.828	2.039	2.368	2.649	3.535	5.021];
% error2_20n_LS = [1.539	0.813	0.599	0.510	0.460	0.454	0.452	0.454	0.450	0.510	0.531	0.603	0.687	0.830	1.050	1.533];
% 
% figure (2)
% plot(PSI, error2_5_LS, 'r--O', PSI, error2_5n_LS, 'b-^', 'linewidth',2);
% % plot(PSI, error2_LS, 'b--O','linewidth',1);
% % xlabel('psi in deg','FontSize',14)
% ylabel('positioning error (m)','FontSize',14)
% % title('position error along psi','FontSize',18);
% legend('LOS','NLOS')
% set(gca, 'XTick', 0:40:160, 'FontSize', 14)
% % set(gca, 'YTick', 0:0.4:1.6, 'FontSize', 14)
% set(gca, 'YTick', 0:1:5.5, 'FontSize', 14)

% figure (2)
% % plot(PSI, sqrt(error2_LS), 'r--o', PSI, sqrt(error2_CR), 'b-^', 'linewidth',1);
% plot(PSI, sqrt(error2_LS), 'b--o', 'linewidth',1);
% % xlabel('psi in deg','FontSize',14)
% ylabel('RMSE','FontSize',14)
% % title('position error along psi','FontSize',18);
% % legend('LS','CRLB')

% PHI = [-80,-70,-60,-50,-40,-30,-20,-10,0,10,20,30,40,50,60,70];
% error3_5_LS = [2.679	1.437	1.082	0.925	0.848	0.830	0.822	0.834	0.828	0.910	0.994	1.113	1.274	1.559	2.002	2.967];
% error3_20_LS = [0.830	0.446	0.330	0.275	0.252	0.237	0.231	0.231	0.232	0.265	0.299	0.345	0.431	0.529	0.729	1.120];
% error3_5n_LS = [5.139	2.750	2.174	1.817	1.667	1.650	1.615	1.575	1.614	1.655	1.962	2.156	2.502	2.985	3.911	5.593];
% error3_20n_LS = [1.415	0.742	0.547	0.474	0.446	0.441	0.435	0.441	0.438	0.516	0.556	0.618	0.720	0.906	1.169	1.780];
% 
% figure (3)
% plot(PHI, error3_5_LS, 'r--O', PHI, error3_5n_LS, 'b-^', 'linewidth',2);
% % plot(PSI, error2_LS, 'b--O','linewidth',1);
% % xlabel('psi in deg','FontSize',14)
% ylabel('positioning error (m)','FontSize',14)
% % title('position error along psi','FontSize',18);
% legend('LOS','NLOS')
% set(gca, 'XTick', -80:40:80, 'FontSize', 14)
% % set(gca, 'YTick', 0:0.4:1.8, 'FontSize', 14)
% set(gca, 'YTick', 0:1:6, 'FontSize', 14)

% PHI = [-80,-70,-60,-50,-40,-30,-20,-10,0,10,20,30,40,50,60,70,80];
% error3_LS = [3.2406,0.7634,0.3357,0.1941,0.1214,0.0875,0.0656,0.0489,0.0407,0.0402,0.0471,0.0619,0.0877,0.1278,0.1954,0.3391,0.9039];
% error3_CR = [3.4157,0.8255,0.3724,0.2172,0.1464,0.1066,0.0821,0.067,0.0589,0.0551,0.0516,0.0495,0.0522,0.0601,0.0818,0.1505,0.5572];
% 
% figure (3)
% % plot(PHI, sqrt(error3_LS), 'r--o', PHI, sqrt(error3_CR), 'b-^', 'linewidth',1);
% plot(PHI, sqrt(error3_LS), 'b--o', 'linewidth',1);
% % xlabel('psi in deg','FontSize',14)
% ylabel('RMSE','FontSize',14)
% % title('position error along psi','FontSize',18);
% % legend('LS','CRLB')
% 
% % error3a = [5.5353,3.0284,1.9243,1.3704,1.0843,0.8911,0.8455,0.747,0.6464,0.6155,0.6068,0.6172,0.6228,0.6636,0.842,1.1488];
% % error3b = [0.8369,0.2157,0.1066,0.0637,0.046,0.0425,0.0423,0.0431,0.0463,0.0593,0.0807,0.1178,0.1742,0.2389,0.4652,0.944];
% % figure (8)
% % plot(10:10:160, sqrt(error3b),'r--o',10:10:160, sqrt(error2),'g--*',10:10:160, sqrt(error3a),'b--','linewidth',1);
% % xlabel('angle in deg','FontSize',14)
% % ylabel('RMSE','FontSize',14)
% % title('position error along angle','FontSize',18);
% % legend('2d theta','3d psi','3d phi + 90deg')
% 
% 
% error4_LS = [0.0004,0.0037,0.0103,0.0202,0.0342,0.0499,0.0695,0.0918,0.118,0.1521,0.1764,0.2096,0.2492,0.2906,0.3372,0.4016,0.4383,0.5036,0.543,0.6435,0.7028,0.7507,0.8269,0.9064,0.9973,1.1189];
% error4_CR = [0.0006,0.0054,0.0147,0.0291,0.0489,0.0727,0.1002,0.1335,0.1717,0.2207,0.2558,0.3085,0.37,0.4215,0.4932,0.5674,0.639,0.7258,0.8062,0.9195,1.0039,1.1,1.1887,1.3089,1.432,1.5939];
% 
% figure (4)
% % plot(0.01:0.02:0.51, sqrt(error4_LS),'r--o',0.01:0.02:0.51, sqrt(error4_CR), 'b-^','linewidth',1);
% plot(0.01:0.02:0.51, sqrt(error4_CR), 'b--o','linewidth',1);
% % xlabel('ranging noise standard deviation in m','FontSize',14)
% ylabel('RMSE','FontSize',14)
% % title('position error along standard deviation','FontSize',18);
% % legend('LS','CRLB')
% %
% error5_LS = [48.2,5.5428,2.0376,1.0477,0.6215,0.4533,0.3291,0.2544,0.1984,0.1682,0.1446,0.1232,0.1094,0.0994,0.0868,0.0801,0.0707,0.0632,0.0577,0.0545,0.0533,0.0469,0.0459,0.0425,0.0413,0.0403];
% error5_CR = [48.917,5.4828,1.9404,1.0294,0.5965,0.4256,0.3004,0.235,0.1859,0.1575,0.139,0.1177,0.1091,0.1011,0.092,0.0874,0.0802,0.0749,0.0713,0.0686,0.0686,0.0648,0.0639,0.0615,0.0598,0.0578];
% 
% figure (5)
% % plot(1:2:51, sqrt(error5_LS), 'r--o',1:2:51, sqrt(error5_CR), 'b-^','linewidth',1);
% plot(1:2:51, sqrt(error5_LS), 'b--o','linewidth',1);
% % title('RMSE changes with  ','FontSize',18);
% ylabel('RMSE','FontSize',14)
% % xlabel('  in m','FontSize',14)
% % legend('LS','CRLB')
% %
% error6_LS = [47.716,5.4682,1.9584,1.0136,0.6348,0.4339,0.3257,0.2511,0.2079,0.1707,0.1422,0.1258,0.1117,0.099,0.085,0.0784,0.0695,0.065,0.0577,0.0551,0.0496,0.0493,0.0459,0.0445,0.041,0.0399];
% error6_CR = [49.005,5.3244,1.9409,1.0231,0.6272,0.4119,0.3,0.2342,0.188,0.1619,0.1375,0.1227,0.1101,0.0992,0.0913,0.0835,0.0798,0.0781,0.0745,0.0684,0.0669,0.0639,0.0628,0.0622,0.0585,0.0585];
% 
% figure (13)
% % plot(1:2:51, sqrt(error6_LS), 'r--o',1:2:51, sqrt(error6_CR), 'b-^','linewidth',1);
% plot(1:2:51, sqrt(error6_LS), 'b--o','linewidth',1);
% % title('RMSE changes with  ','FontSize',18);
% ylabel('RMSE','FontSize',14)
% % xlabel('  in m','FontSize',14)
% % legend('LS','CRLB')
% 
% %
% error7_LS = [48.858,5.1425,1.9671,1.1016,0.6389,0.4543,0.3161,0.2608,0.2106,0.1723,0.1452,0.121,0.1082,0.0952,0.0896,0.078,0.0687,0.0647,0.0609,0.0567,0.051,0.0471,0.0442,0.0435,0.042,0.0396];
% error7_CR = [48.774,5.2253,1.9247,0.988,0.611,0.4177,0.2948,0.242,0.1926,0.1572,0.1368,0.1166,0.1095,0.0973,0.0924,0.0862,0.0799,0.0767,0.0747,0.0709,0.0665,0.0643,0.0625,0.0621,0.0605,0.059];
% 
% figure (15)
% % plot(1:2:51, sqrt(error7_LS), 'r--o',1:2:51, sqrt(error7_CR), 'b-^','linewidth',1);
% plot(1:2:51, sqrt(error7_LS), 'b--o','linewidth',1);
% % title('RMSE changes with  ','FontSize',18);
% ylabel('RMSE','FontSize',14)
% % xlabel('  in m','FontSize',14)
% % legend('LS','CRLB')
% %
% error5a3 = [7.2894,4.9889,2.9127,2.614,2.0181,1.7546,1.5356,1.3796,1.2689,1.1938,1.1145,1.0429,1.0066,0.9088,0.805,0.8107,0.8124,0.8159,0.7771,0.7679,0.7591,0.7265,0.719,0.6868];
% error5a2 = [7.6451,4.4306,3.2746,2.4761,2.1368,1.7361,1.5318,1.4405,1.265,1.1626,1.102,1.0045,0.978,0.9132,0.8931,0.8663,0.8424,0.8165,0.8152,0.7888,0.7802,0.7398,0.7387,0.724];
% error5a1 = [7.6653,4.3534,3.3458,2.4442,2.0498,1.7493,1.5891,1.4478,1.28,1.1843,1.1003,1.0378,0.9931,0.9218,0.8953,0.9016,0.8767,0.821,0.788,0.7748,0.773,0.7661,0.7542,0.7269];
% error5b2 = [9.3186,3.2345,1.731,0.9901,0.6443,0.4504,0.3652,0.2917,0.2418,0.2027,0.162,0.1353,0.1254,0.1214,0.1011,0.0944,0.0841,0.0815,0.0758,0.067,0.0642,0.0616,0.0582,0.0566];
% error5b1 = [8.6929,2.9793,1.5437,0.9098,0.6232,0.4339,0.3302,0.2719,0.1968,0.1984,0.1319,0.1212,0.1204,0.1039,0.0828,0.0923,0.0802,0.0818,0.0631,0.0623,0.0592,0.0602,0.0567,0.0536];
% figure (9)
% plot(3:2:49, sqrt(error5b1),'r--o',3:2:49, sqrt(error5b2),'r--*',3:2:49, sqrt(error5a1),'b--o',3:2:49, sqrt(error5a2),'b--*',3:2:49, sqrt(error5a3),'b--','linewidth',1);
% xlabel('anchor distance in m','FontSize',14)
% ylabel('RMSE','FontSize',14)
% title('position error along anchor distance','FontSize',18);
% legend('2d-k','2d-l','3d-k','3d-l','3d-m')
% 
% % figure(10)
% % X22 = X2;
% % Y22 = Y2;
% % Z22 = 100*ones(20,20);
% % for k = 20
% %     for j = 1:20
% %        for i = 1:20
% %             zz(i,j) = sqrt(err((k-1)*20*20+(j-1)*20+i));
% %        end
% %     end
% % end
% % surf(X22,Y22,Z22,zz);
% % hold on
% % X22 = 0*ones(20,20);
% % Y22 = Y2;
% % Z22 = Z2;
% % for k = 1:20
% %     for j = 1:20
% %        for i = 6
% %             zz(i,j) = sqrt(err((k-1)*20*20+(j-1)*20+i));
% %        end
% %     end
% % end
% % surf(X22,Y22,Z22,zz);
% % hold on
% % X22 = X2';
% % Y22 = 0*ones(20,20);
% % Z22 = Z2;
% % for k = 1:20
% %     for j = 6
% %        for i = 1:20
% %             zz(i,j) = sqrt(err((k-1)*20*20+(j-1)*20+i));
% %        end
% %     end
% % end
% % surf(X22,Y22,Z22,zz);
% % hold on
% % plot3(xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*','linewidth',2)
% % xlabel('x-axis in m','FontSize',14)
% % ylabel('y-axis in m','FontSize',14)
% % zlabel('Z-axis in m','FontSize',14)
% % title('RMSE surf','FontSize',18);
% % cb = colorbar;
% % 
% % 
% % figure(6)
% % X22 = X2;
% % Y22 = Y2;
% % Z22 = 25*ones(20,20);
% % for k = 1
% %     for j = 1:20
% %        for i = 1:20
% %             zz(i,j) = sqrt(err((k-1)*20*20+(j-1)*20+i));
% %        end
% %     end
% % end
% % % mesh(X2,Y2,Z2,zz);
% % contour(X2,Y2,zz);
% % hold on
% % plot3(xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*','linewidth',2)
% % xlabel('x-axis(m)','FontSize',14)
% % ylabel('y-axis(m)','FontSize',14)
% % % zlabel('Z-axis(m)','FontSize',14)
% % title('RMSE contour projected on the Y-X plane','FontSize',18);
% % 
% % figure(11)
% % X22 = 25*ones(20,20);
% % Y22 = Y2;
% % Z22 = Z2;
% % for k = 1:20
% %     for j = 1:20
% %        for i = 1
% %             zz(i,j) = sqrt(err((k-1)*20*20+(j-1)*20+i));
% %        end
% %     end
% % end
% % contour(X2,Y2,zz);
% % hold on
% % plot(yr1,zr1,'r*',yr2,zr2,'r*',yr3,zr3,'r*',yr4,zr4,'r*','linewidth',2)
% % xlabel('y-axis(m)','FontSize',14)
% % ylabel('z-axis(m)','FontSize',14)
% % % zlabel('Z-axis in m','FontSize',14)
% % title('RMSE contour projected on the Z-Y plane','FontSize',18);
% % 
% % figure(12)
% % X22 = X2;
% % Y22 = 25*ones(20,20);
% % Z22 = Z2;
% % for k = 1:20
% %     for j = 1
% %        for i = 1:20
% %             zz(i,j) = sqrt(err((k-1)*20*20+(j-1)*20+i));
% %        end
% %     end
% % end
% % contour(X2,Y2,zz);
% % hold on
% % plot(zr1,xr1,'r*',zr2,xr2,'r*',zr3,xr3,'r*',zr4,xr4,'r*','linewidth',2)
% % xlabel('z-axis(m)','FontSize',14)
% % ylabel('x-axis(m)','FontSize',14)
% % % zlabel('Z-axis in m','FontSize',14)
% % title('RMSE contour projected on the X-Z plane','FontSize',18);
% 
% % anchp = [xr1 yr1 zr1
% %         xr3 yr3 zr3
% %         xr2 yr2 zr2
% %         xr4 yr4 zr4
% %         xr1 yr1 zr1];
% 
% % figure (5)
% % plotcube([50 50 25],[-25 -25 0],.1,[0 0 1]);
% % hold on
% % % plot3(xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*','linewidth',2)
% % plot3(anchp(:,1),anchp(:,2),anchp(:,3),'r--*','linewidth',2);
% % xlabel('X position in m','FontSize',18)
% % ylabel('Y position in m','FontSize',18)
% % zlabel('Z position in m','FontSize',18)
% % text(-40,0,40,'\fontsize{20}\color{black}Anchors distribution')
% 
% error2d = [0.0005,0.005,0.0138,0.0257,0.0435,0.0723,0.0967,0.1269,0.1607,0.199,0.2676,0.2944,0.3677,0.4134,0.5173,0.5642,0.6093,0.6875,0.7827,0.8945,0.9092,0.973,1.1189,1.27,1.3571];
% error3d = [0.0694,0.22,0.357,0.5389,0.6836,0.8139,0.9335,1.2734,1.2247,1.3696,1.4206,1.5916,1.6634,1.9866,1.9706,2.1153,2.2472,2.3462,2.681,2.9447,2.9603,3.194,3.228,3.2634,3.4205];
% figure (7)
% plot(0.01:0.02:0.49, sqrt(error2d),'r--o',0.01:0.02:0.49, sqrt(error3d),'b--*','linewidth',1);
% xlabel('ranging noise standard deviation in m','FontSize',14)
% ylabel('RMSE','FontSize',14)
% title('position error between 2d and 3d','FontSize',18);
% legend('2d var','3d var')

% figure (9)
% dd = [2 4 6 8 10 12 14 16 18 20];
% LS = [1.1115	0.5786	0.4089	0.3319	0.2865	0.2642	0.2429	0.2332	0.2215	0.2162];
% CH = [5.4822	2.4881	1.5312	1.0901	0.8406	0.7057	0.6193	0.5703	0.5396	0.5116];
% CR = [0.7409	0.4289	0.3131	0.2631	0.2361	0.2205	0.2094	0.2054	0.2026	0.1964];
% MO = [1.4053	0.8420	0.6565	0.5744	0.5211	0.5050	0.4802	0.4742	0.4558	0.4524];
% SI = [1.9827	1.0141	0.6984	0.5568	0.4696	0.4327	0.3931	0.3762	0.3536	0.3436];
% GD = [1.7224	0.8929	0.6434	0.5314	0.4766	0.4493	0.4293	0.4242	0.4105	0.4059];
% plot(dd, CR, 'k', dd, GD,'g:', dd, SI,'r-.', 'linewidth',2);
% xlabel('distance of any two anchors (m)','FontSize',14)
% ylabel('3D positioning error (m)','FontSize',14)
% legend('CRLB','GDOP','Our apporach')
% 
% avg = [mean(LS),mean(GD),mean(SI),mean(CR)]
% mstd = [std(LS),std(MO),std(SI),std(CR)]

% figure (10)
% PSI = [10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160];
% LS = [0.6379	0.3553	0.2721	0.2359	0.2166	0.2089	0.2085	0.2075	0.2136	0.2287	0.2449	0.2681	0.3020	0.3662	0.4569	0.6612];
% CR = [0.6232	0.4522	0.4219	0.2992	0.2213	0.2054	0.2015	0.1989	0.1979	0.2012	0.2021	0.2037	0.2112	0.2250	0.2490	0.2869];
% SI = [1.9759	1.0069	0.6687	0.5346	0.4421	0.3917	0.3641	0.3476	0.3381	0.3548	0.3763	0.4120	0.4676	0.5676	0.7320	1.0653];
% GD = [0.8267	0.6819	0.5846	0.5751	0.5606	0.4803	0.4396	0.4168	0.4055	0.4042	0.4060	0.4076	0.4130	0.4193	0.4272	0.4402];
% plot(PSI, CR, 'k', PSI, GD,'g:', PSI, SI,'r-.', 'linewidth',2);
% xlabel('angle (deg)','FontSize',14)
% ylabel('positioning error (m)','FontSize',14)
% legend('CRLB','GDOP','Our apporach')
% % axis([0 160 0 1.3])
% grid

% avg = [mean(LS),mean(GD),mean(SI),mean(CR)]
% mstd = [std(LS),std(GD),std(SI),std(CR)]

% figure (11)
% PSI = [-70	-60	-50	-40	-30	-20	-10	0	10	20	30	40	50	60	70	80];
% CR = [0.165321474	0.155229212	0.150887317	0.151015019	0.307780654	0.210008421	0.200804326	0.19670293	0.205266388	0.227785044	0.277115169	0.233317948	0.208777034	0.368831034	0.543656151	0.325371259];
% LS = [0.63794353	0.355269291	0.272112356	0.235870763	0.216567674	0.208945642	0.208520152	0.207526193	0.213642002	0.228727297	0.244873996	0.268112324	0.301956341	0.366159336	0.456917173	0.661194667];
% SI = [1.094210719	0.737889934	0.571829894	0.469977708	0.408939743	0.372605052	0.361341634	0.317846068	0.336020039	0.358762787	0.38823609	0.437621141	0.526513038	0.671708529	0.977086541	1.956179419];
% GD = [0.53242404	0.477154432	0.443166896	0.425668912	0.409634398	0.401567125	0.402495789	0.406052587	0.476590678	0.433110871	0.429599332	0.418214154	0.420195373	0.41387149	0.406830641	0.501265771];
% plot(PSI, LS, 'k', PSI, GD,'g:', PSI, SI,'r-.', 'linewidth',2);
% xlabel('angle (deg)','FontSize',14)
% ylabel('positioning error (m)','FontSize',14)
% legend('CRLB','GDOP','Our apporach')
% % axis([0 160 0 1.3])
% grid
% 
% avg = [mean(LS),mean(GD),mean(SI),mean(CR)]


% plot noise
% figure (12)
% plot(R2m-R2,'linewidth',1)
% xlabel('samples','FontSize',14)
% ylabel('noise (m)','FontSize',14)

