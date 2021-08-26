%% 
% program name uwb_distribution_2D.m
% evaluate the impact of uwb errors in 2D positioning
% 2021-01-23
close all;
clear all;
clc;

r2d = (180/pi);
d2r = (pi/180);
dt = 0.5;
fz = 1*0.005;
T = 4*(1/fz);
t0 = 0:dt:T;
t0_1 = t0';
n = length(t0);
m = size(t0_1);
radius = 1*8;
apha_0 = 0*45*d2r;
ft =1*0.01;
wt = 2*pi*ft;
% ======================================================
% Define four UWB sensors locations
% =====================================================
xr1 = 0;% in meter
yr1 = 0;% in meter
xr2 = 0;
yr2 = 20;

theta =90*d2r;
r3 = 20;
xr3 = r3*sin(theta);% in meter
yr3 = r3*cos(theta);% in meter

H = [xr2-xr1, yr2-yr1
    xr3-xr1, yr3-yr1];

q(1) = xr1^2+yr1^2;
q(2) = xr2^2+yr2^2;
q(3) = xr3^2+yr3^2;

% position of mobile
[x_p_N,x_v_N,y_p_N,y_v_N] = trajectory2d_line_vel(radius,apha_0,wt,t0,n);
% x = linspace(-50,100,40);
% y = linspace(-50,100,40);
% [xx,yy] = meshgrid(x,y);
% for j = 1:40
%    for i = 1:40
%        x_p_N((j-1)*40+i) = x(i);
%        y_p_N((j-1)*40+i) = y(j);
%    end
% end

% =====================================================================================
% Define "4 radio sensor" parameters (noise)
% =====================================================================================
radiosensor_err_factor = 1.0;
sigma = 0.1;
sig_x_r=radiosensor_err_factor*sigma;              % radio sensor measurement noise in meters x-direction
sig_y_r=radiosensor_err_factor*sigma;              % radio sensor measurement noise in meters y-direction
sig_z_r=radiosensor_err_factor*sigma;
%======================================================================================
[R1m,R2m,R3m,nvx_r,nvy_r,nvz_r] = radio_sensor2d_m(xr1,yr1,xr2,yr2,xr3,yr3,x_p_N,y_p_N,sig_x_r,sig_y_r,sig_z_r,n-1,m);%3 four radio sensors
[R1,R2,R3] = radio_sensor2d(xr1,yr1,xr2,yr2,xr3,yr3,x_p_N,y_p_N,n-1,m);%3 four radio sensors

% LS
for k=1:n
    b_1 = [R1m(k)^2-R2m(k)^2-q(1)+q(2)
         R1m(k)^2-R3m(k)^2-q(1)+q(3)];
    r(:,k) = 0.5*inv(H'*H)*H'*b_1;
end

LS_delta_P = [x_p_N(1:n-1)-r(1,1:n-1); y_p_N(1:n-1)-r(2,1:n-1)];
err1 = LS_delta_P(1,:).^2+LS_delta_P(2,:).^2;
LS = sqrt(mean(err1));
std_LS = std(err1);

% f_distr = (1/(sin(theta)^2))*((1/(yr2^2))+(1/(r3^2)))

% %  LS lterative method
% p_t(1,1,1) = 25;
% p_t(2,1,1) = 25;
% % p_t(3,1,1) = 0;
% for k = 1:n 
% r_t(1,1,k) = sqrt((p_t(1,1,k)-xr1)^2+(p_t(2,1,k)-yr1)^2);
% r_t(2,1,k) = sqrt((p_t(1,1,k)-xr2)^2+(p_t(2,1,k)-yr2)^2);
% % r_t(3,1,k) = sqrt((p_t(1,1,k)-x_anchor_4)^2+(p_t(2,1,k)-y_anchor_4)^2+(p_t(3,1,k)-z_anchor_4)^2) ;
% r_e(1,1,k) = R1m(k);
% r_e(2,1,k) = R2m(k);
% % r_e(3,1,k) = pos(4,k);
% r_d(1,1,k) = r_e(1,1,k)-r_t(1,1,k);
% r_d(2,1,k) = r_e(2,1,k)-r_t(2,1,k);
% % r_d(3,1,k) = r_e(3,1,k)-r_t(3,1,k);
% 
% H_t = [(p_t(1,1,k)-xr1)/r_e(1,1,k) (p_t(2,1,k)-yr1)/r_e(1,1,k)
%     (p_t(1,1,k)-xr2)/r_e(2,1,k) (p_t(2,1,k)-yr2)/r_e(2,1,k)];
% p_t(:,1,k+1) = p_t(:,1,k)+inv(H_t)*r_d(:,1,k);
% p_x(k) =p_t(2,1,k);
% p_y(k) =p_t(1,1,k);
% end
% 
% LS_delta_P = [x_p_N(1:n-1)-p_x(1:n-1); y_p_N(1:n-1)-p_y(1:n-1)];
% err1 = LS_delta_P(1,:).^2+LS_delta_P(2,:).^2;
% LS = sqrt(mean(err1))

for k=1:n-1
    delta_R = [-2*R1m(k)*nvx_r(k)+2*R2m(k)*nvy_r(k)
                -2*R1m(k)*nvx_r(k)+2*R3m(k)*nvz_r(k)];
    delta_P(:,k) = 0.5*inv(H)*delta_R;
    conv_dR(:,:,k) = [R1m(k)^2*nvx_r(k)^2+R2m(k)^2*nvy_r(k)^2, R1m(k)^2*nvx_r(k)^2;
                    R1m(k)^2*nvx_r(k)^2, R1m(k)^2*nvx_r(k)^2+R3m(k)^2*nvz_r(k)^2];
    E_dp(:,:,k) = inv(H)*conv_dR(:,:,k)*inv(H)';
end

for k=1:n-1
   plot_E_dp(1,k) =  E_dp(1,1,k);
   plot_E_dp(2,k) =  E_dp(2,2,k);
end

err = abs(plot_E_dp(1,:)')+abs(plot_E_dp(2,:)');

for j = 1:40
   for i = 1:40
        zz(i,j) = sqrt(err((j-1)*40+i));
   end
end
% CRLB = sqrt(mean(err))
% B = sort(err);
% N = length(B);
% number = fix(0.9973*N);
% B1 = B(number)

% % Chan method
% Ga_p=[1,0;0,1;1,1];
% for k = 1:n-1
%     Q = [nvx_r(k),0;0,nvy_r(k)];
%     B_2p = [x_p_N(k)-xr1,0;0,y_p_N(k)-yr1];
%     B_p = [x_p_N(k)-xr1,0,0;0,y_p_N(k)-yr1,0;0,0,R1(k)];
%     G_a = -[xr2-xr1,yr2-yr1,R2(k)-R1(k);xr3-xr1,yr3-yr1,R3(k)-R1(k)];
%     B1 = [R2(k),0;0,R3(k)];
%     Phi(:,:,k) = inv(B_2p*Ga_p'*inv(B_p)*G_a'*inv(B1)*inv(Q)*inv(B1)*G_a*inv(B_p)*Ga_p*B_2p);
% end
% for k=1:n-1
%    plot_Phi(1,k) =  Phi(1,1,k);
%    plot_Phi(2,k) =  Phi(2,2,k);
% end
% 
% err1 = abs(plot_Phi(1,:)')+abs(plot_Phi(2,:)');
% Chan = sqrt(mean(err1));
% std_Chan = std(err1);

% CRLB
for k = 1:n-1
    Q = [nvx_r(k),0;0,nvy_r(k)];
    Gt11 = (xr1-x_p_N(k))/R1(k)-(xr2-x_p_N(k))/R2(k);
    Gt12 = (yr1-y_p_N(k))/R1(k)-(yr2-y_p_N(k))/R2(k);
    Gt21 = (xr1-x_p_N(k))/R1(k)-(xr3-x_p_N(k))/R3(k);
    Gt22 = (yr1-y_p_N(k))/R1(k)-(yr3-y_p_N(k))/R3(k);
    Gt = [Gt11,Gt12;Gt21,Gt22];
    Gl = [xr2-xr1,R2(k)-R1(k);xr3-xr1,R3(k)-R1(k)];
    Phi0(:,:,k)=inv(Gt'*inv(Q)*Gl);
end

for k=1:n-1
   plot_Phi0(1,k) =  Phi0(1,1,k);
   plot_Phi0(2,k) =  Phi0(2,2,k);
   if abs(plot_Phi0(1,k))>2
       plot_Phi0(1,k) = 0;
   end
   if abs(plot_Phi0(2,k))>2
       plot_Phi0(2,k) = 0;
   end   
end

err2 = abs(plot_Phi0(1,:)')+abs(plot_Phi0(2,:)');
CRLB = sqrt(mean(err2));
std_CRLB = std(err2);

% Monica
zeta2 = 20^2;
omega2 = (yr2-yr1)^2;
eta2 = 4*(omega2/4+zeta2);
lemda2 = omega2/eta2;
b = 19*lemda2/70+1/3;
d = -lemda2/24;
syms x
beta=solve(x^3+0.3876*x^2-0.0083==0);
betaS = double(max([beta(1,1),beta(2,1),beta(3,1)]));
detaS = sqrt(betaS*(omega2+4*zeta2));
C(1) = 2.5*detaS^2*eta2^2*omega2+26*detaS^4*eta2*omega2+146*detaS^6*omega2+1.25*detaS^4*eta2^2+13.5*detaS^6*eta2+58.5*detaS^8;
C(2) = -(468*detaS^5*omega2+60*detaS^3*eta2*omega2+3*detaS*eta2^2*omega2+15*detaS^5*eta2+108*detaS^7);
C(3) = eta2^2*omega2+56*detaS^2*eta2*omega2+606*detaS^4*omega2+5*detaS^4*eta2+81*detaS^6;
C(4) = -(24*detaS*eta2*omega2+408*detaS^3*omega2+30*detaS^5);
C(5) = 5*detaS^4+158*detaS^2*omega2+4*eta2*omega2;
C(6) = -36*detaS*omega2;
C(7) = 4*omega2;
x_avg = mean(x_p_N)/9;

for k = 1:n-1
    err3(k) = nvx_r(k)^2/(4*detaS^4*eta2*omega2)*sum(C)*x_avg;
end
Monica = sqrt(mean(err3));
std_Monica = std(err3);

% Simplify upper bound
for k = 1:n-1
    err4(k) = nvx_r(k)^2/sin(theta)^2*((2*R1(k)^2+R2(k)^2)/yr2^2+(2*R1(k)^2+R3(k)^2)/r3^2);
end
Simplify = sqrt(mean(err4));
std_Simplify = std(err4);

% Error minimizing Localization CRLB
for k=1:n-1
    A(k,1) = (x_p_N(k)-xr1)/R1(k);
    A(k,2) = (x_p_N(k)-xr2)/R2(k);
    A(k,3) = (x_p_N(k)-xr3)/R3(k);
    B(k,1) = (y_p_N(k)-yr1)/R1(k);
    B(k,2) = (y_p_N(k)-yr2)/R2(k);
    B(k,3) = (y_p_N(k)-yr3)/R3(k);
    I11 = 0;
    I22 = 0;
    I12 = 0;
    I21 = 0;
    for m=1:2
        for l=(m+1):3
            I11 = I11 + (A(k,m)-A(k,l))^2/nvx_r(k)^2;
            inv_I11 = 1/I11;
            I22 = I22 + (B(k,m)-B(k,l))^2/nvx_r(k)^2;
            I12 = I12 + (A(k,m)-A(k,l))*(B(k,m)-B(k,l))/nvx_r(k)^2;
            I21 = I12;
        end
    end
%     err5(k) = I22/(I11*I22-I12^2)+I11/(I11*I22-I12^2);
    err5(k) = inv_I11+inv_I11*I12*I12*inv_I11/(I22-I12*inv_I11*I12);
end
MinError = sqrt(mean(err5));
std_MinError = std(err5);

% GDOP
for k=1:n-1
    p1 = sqrt((xr1-xr2)^2+(yr1-yr2)^2);
    p2 = sqrt((xr1-xr3)^2+(yr1-yr3)^2);
    p3 = sqrt((xr2-xr3)^2+(yr2-yr3)^2);
    pm_1 = R1(k)^2+R2(k)^2-p1^2;
    pm_2 = 2*R1(k)*R2(k);
    G1(k) = 2/(1-(pm_1/pm_2)^2)*nvx_r(k)^2;
    pm_3 = R1(k)^2+R3(k)^2-p2^2;
    pm_4 = 2*R1(k)*R3(k);    
    G2(k) = 2/(1-(pm_3/pm_4)^2)*nvy_r(k)^2;
    pm_5 = R2(k)^2+R3(k)^2-p3^2;
    pm_6 = 2*R2(k)*R3(k);    
    G3(k) = 2/(1-(pm_5/pm_6)^2)*nvz_r(k)^2;
    if k>1
        if G3(k)>2
            G3(k) = G3(k-1);
        end
    else
        G3(k) = 0;
    end
    err6(k) = G1(k)+G2(k)+G3(k);
end
GDOP = sqrt(mean(err6));
std_GDOP = std(err6);

% HO
% for k=1:n-1
%     k=1;
%     a1 = [(x_p_N(k)-xr1)/R1(k),(y_p_N(k)-yr1)/R1(k),0]';
%     a2 = [(x_p_N(k)-xr2)/R2(k),(y_p_N(k)-yr2)/R2(k),0]';
%     a3 = [(x_p_N(k)-xr3)/R3(k),(y_p_N(k)-yr3)/R3(k),0]';
%     b1 = [x_v_N(k)/R1(k),y_v_N(k)/R1(k),0]'-nvx_r(k)/dt*[(x_p_N(k)-xr1)/R1(k)^2,(y_p_N(k)-yr1)/R1(k)^2,0]';
%     b2 = [x_v_N(k)/R2(k),y_v_N(k)/R2(k),0]'-nvx_r(k)/dt*[(x_p_N(k)-xr2)/R2(k)^2,(y_p_N(k)-yr2)/R2(k)^2,0]';
%     b3 = [x_v_N(k)/R3(k),y_v_N(k)/R3(k),0]'-nvx_r(k)/dt*[(x_p_N(k)-xr3)/R3(k)^2,(y_p_N(k)-yr3)/R3(k)^2,0]';
%     C1_1 = [-a1',a2',0,0,0];
%     C1_2 = [-a1',0,0,0,a3'];
%     C2_1 = [-b1',b2',0,0,0];
%     C2_2 = [-b1',0,0,0,b3'];
%     dp = [C1_1;C1_2;C2_1;C2_2]*eye(9,3); % u
%     dv = [zeros(2,3);[C1_1;C1_2]*eye(9,3)]; % u_h
% %     G_theta = [dp,dv];
%     
%     dq = -[C1_1;C1_2;C2_1;C2_2]; % s
%     dw = [zeros(2,9);-[C1_1;C1_2]]; % s_h
%     
%     Q_a = zeros(4);
%     Q_a(1,1) = nvx_r(k);
%     Q_a(1,2) = 0.5*nvx_r(k);
%     Q_a(2,1) = 0.5*nvy_r(k);    
%     Q_a(2,2) = nvy_r(k);
%     Q_a(3,3) = nvx_r(k);
%     Q_a(3,4) = 0.5*nvx_r(k);
%     Q_a(4,3) = 0.5*nvy_r(k);       
%     Q_a(4,4) = nvy_r(k);
% 
%     X = [dp,dv]'*inv(Q_a)*[dp,dv];
%     Y = [dp,dv]'*inv(Q_a)*[dq,dw];
%     Z = [dq,dw]'*inv(Q_a)*[dq,dw];
%     
%     cov_theta(:,:,k) = inv(X)+inv(X)*Y*inv(Z-Y'*inv(X)*Y)*Y'*inv(X);
%     
% end

% HO v2
% for k=1:n-1
%     B = 2*[R2(k),0;0,R3(k)];
%     B_1 = [B,zeros(2,2);B/dt,B];
%     G_1 = -2*[xr2-xr1,yr2-yr1,0,R2(k)-R1(k),zeros(1,4)
%               xr3-xr1,yr3-yr1,0,R3(k)-R1(k),zeros(1,4)
%               (xr2-xr1)/dt,(yr2-yr1)/dt,0,(R2(k)-R1(k))/dt,xr2-xr1,yr2-yr1,0,R2(k)-R1(k)
%               (xr3-xr1)/dt,(yr3-yr1)/dt,0,(R3(k)-R1(k))/dt,xr3-xr1,yr3-yr1,0,R3(k)-R1(k)];
%     B_2_11 = 2*[x_p_N(k)-xr1,0,0;0,y_p_N(k)-yr1,0;0,0,0];
%     B_2_31 = B_2_11/2/dt;
%     B_2_33 = B_2_11/2;
%     B_2 = [B_2_11,zeros(3,1),zeros(3,3),zeros(3,1)
%            zeros(1,3),2*R1(k),zeros(1,3),0
%            B_2_31,zeros(3,1),B_2_33,zeros(3,1)
%            zeros(1,3),R1(k)/dt,zeros(1,3),R1(k)];
%     G_2 = [eye(3),zeros(3)
%            ones(1,3),zeros(1,3)
%            zeros(3),eye(3)
%            zeros(1,3),ones(1,3)];
%     B_3 = [B_2_11,zeros(3);B_2_31,B_2_33];
%     
%     Q_a = zeros(4);
%     Q_a(1,1) = nvx_r(k);
%     Q_a(1,2) = 0.5*nvx_r(k);
%     Q_a(2,1) = 0.5*nvy_r(k);    
%     Q_a(2,2) = nvy_r(k);
%     Q_a(3,3) = nvx_r(k);
%     Q_a(3,4) = 0.5*nvx_r(k);
%     Q_a(4,3) = 0.5*nvy_r(k);       
%     Q_a(4,4) = nvy_r(k);
%     
%     G_3 = inv(B_1)*G_1*inv(B_2)*G_2*B_3;
%     cov_phi(:,:,k) = inv(G_3'*inv(Q_a)*G_3);
% end

% avg = [LS,Monica,Simplify,GDOP,CRLB]
% avg = [LS,CRLB,Monica,Simplify]
% mstd = [std_LS,std_Monica,std_Simplify,std_GDOP,std_CRLB]

% figure (1)
% plot(x_p_N,y_p_N,'b',xr1,yr1,'r*',xr2,yr2,'r*',xr3,yr3,'r*','linewidth',2)
% xlabel('x (m)','FontSize',14)
% ylabel('y (m)','FontSize',14)
% set(gca, 'XTick', -15:10:35, 'FontSize', 14)
% % title('2D Simulation','FontSize',18);
% % viscircles([0,0],5,'LineStyle','-.','linewidth',1);
% hold on
% t = (-90:0.5:90)*pi/180;
% x = 20*cos(t);
% y = 20*sin(t);
% plot(x,y,'r--')
% % an = {' q_1',' q_2',' q_3'};
% % xt=[xr1,xr2,xr3];
% % yt=[yr1,yr2,yr3];
% % text(xt,yt,an,'fontsize',18,'linewidth',2);
% % axis([-5 20.5 -20.5 20.5])
% % axis([-0.5 5.5 -3.5 6.5])
% axis equal
% % grid

% figure (2)
% tt = 1:n-1;
% plot( tt, sqrt(err3(tt)),'b--', tt, sqrt(err4(tt)),'r-.', tt, sqrt(err1(tt)),'k', 'linewidth',2);
% xlabel('sampling data','FontSize',14)
% ylabel('NLOS positioning error (m)','FontSize',14)
% legend('Approch in [38]','Our apporach','CRLB')
% set(gca, 'XTick', 0:400:1600, 'FontSize', 14)
% % set(gca, 'YTick', 0:0.5:1.5, 'FontSize', 14)
% set(gca, 'YTick', 0:1:5, 'FontSize', 14)

% figure (2)
% tt = 1:n-1;
% plot( tt, sqrt(err3(tt)),'b--', tt, sqrt(err6(tt)),'g:', tt, sqrt(err4(tt)),'r-.', tt, sqrt(err2(tt)),'k', 'linewidth',2);
% xlabel('sampling data','FontSize',14)
% ylabel('positioning error (m)','FontSize',14)
% legend('Approch in [36]','GDOP','Our apporach','CRLB')
% axis([100 160 0 1.2])

% PSI = [10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160];
% error2_5_LS = [2.044	1.022	0.783	0.620	0.556	0.539	0.540	0.536	0.526	0.606	0.677	0.723	0.891	1.148	1.458	2.104];
% error2_20_LS = [0.654	0.325	0.213	0.176	0.155	0.150	0.150	0.149	0.149	0.184	0.195	0.227	0.270	0.333	0.448	0.684];
% error2_5n_LS = [4.192	2.091	1.513	1.323	1.155	1.117	1.112	1.113	1.114	1.261	1.402	1.477	1.745	2.210	2.891	4.219];
% error2_20n_LS = [1.323	0.675	0.530	0.448	0.402	0.375	0.368	0.373	0.371	0.412	0.453	0.538	0.644	0.813	1.085	1.699];
% 
% figure (2)
% plot(PSI, error2_5_LS, 'r--O', PSI, error2_5n_LS, 'b-^', 'linewidth',2);
% % plot(PSI, error2_LS, 'b--O','linewidth',1);
% % xlabel('psi in deg','FontSize',14)
% ylabel('positioning error (m)','FontSize',14)
% % title('position error along psi','FontSize',18);
% legend('LOS','NLOS')
% set(gca, 'XTick', 0:40:160, 'FontSize', 14)
% % set(gca, 'YTick', 0:0.4:2, 'FontSize', 14)
% set(gca, 'YTick', 0:1:5, 'FontSize', 14)

% 
% error3_LS = [0.0167,0.0481,0.0807,0.1177,0.1433,0.1807,0.2093,0.2457,0.2709,0.316,0.349,0.3837,0.3965,0.4385,0.4692,0.5041,0.5184,0.5834,0.608,0.643,0.6686,0.6809,0.7309,0.7639,0.8127,0.8316];
% error3_CR = [0.0083,0.0251,0.0404,0.0575,0.0743,0.0905,0.1069,0.1219,0.1398,0.1552,0.1727,0.187,0.2004,0.2228,0.2398,0.2522,0.2651,0.2858,0.3028,0.317,0.338,0.3434,0.3697,0.3747,0.3969,0.4199];
% 
% figure (3)
% % plot(0.01:0.02:0.51, error3_LS, 'r--o',0.01:0.02:0.51, error3_CR*2, 'b-^','linewidth',1);
% plot(0.01:0.02:0.51, error3_CR*2, 'b--o','linewidth',1);
% ylabel('RMSE','FontSize',14)
% % legend('LS','CRLB')
% 
% error4_LS = [0.6164,0.5878,0.5625,0.5374,0.5124,0.4874,0.4624,0.4374,0.4124,0.3874,0.3624,0.3374,0.3124,0.2874,0.2624,0.2374,0.2124,0.1875,0.1625,0.1375,0.1125,0.0875,0.0625,0.0375,0.0126,0.0126];
% error4_CR = [2.836,0.9312,0.5501,0.3954,0.3054,0.2554,0.2108,0.1856,0.1634,0.1489,0.1347,0.124,0.1203,0.1114,0.106,0.1018,0.0983,0.0939,0.0921,0.0894,0.0865,0.0861,0.0837,0.0836,0.0815,0.0796];
% 
% figure (4)
% % plot(3:2:51, error4_LS(2:26),'r--O',3:2:51, error4_CR(2:26),'b-^','linewidth',1);
% plot(3:2:51, error4_CR(2:26),'b--o','linewidth',1);
% % xlabel('  in m','FontSize',14)
% ylabel('RMSE','FontSize',14)
% % title('RMSE changes with  ','FontSize',18);
% % legend('LS','CRLB')
% 
% error5_LS = [0.6123,0.5873,0.5623,0.5373,0.5123,0.4874,0.4624,0.4374,0.4124,0.3874,0.3624,0.3374,0.3124,0.2874,0.2624,0.2374,0.2124,0.1875,0.1625,0.1375,0.1125,0.0875,0.0625,0.0375,0.0126,0.0126];
% error5_CR = [2.8703,0.9317,0.5584,0.3886,0.3013,0.2473,0.217,0.1846,0.1671,0.1511,0.1396,0.125,0.1167,0.1115,0.1046,0.1005,0.0977,0.0968,0.0925,0.0912,0.0872,0.0876,0.0839,0.0822,0.082,0.08];
% 
% figure (7)
% % plot(3:2:51, error5_LS(2:26),'r--o',3:2:51, error5_CR(2:26),'b-^','linewidth',1);
% plot(3:2:51, error5_CR(2:26),'b--o','linewidth',1);
% % xlabel('  in m','FontSize',14)
% ylabel('RMSE','FontSize',14)
% % title('RMSE changes with  ','FontSize',18);
% % legend('LS','CRLB')


% figure(5)
% % mesh(xx,yy,zz);
% contour(xx,yy,zz);
% % surf(xx,yy,zz);
% hold on
% plot(xr1,yr1,'r*',xr2,yr2,'r*',xr3,yr3,'r*','linewidth',2)
% xlabel('x-axis(m)','FontSize',14)
% ylabel('y-axis(m)','FontSize',14)
% title('RMSE contour','FontSize',18);
% % 
% figure(6)
% % mesh(xx,yy,zz);
% % contour(xx,yy,zz);
% surf(xx,yy,zz);
% hold on
% plot3(xr1,yr1,0.25,'r*',xr2,yr2,0.25,'r*',xr3,yr3,0.25,'r*','linewidth',2)
% xlabel('x-axis(m)','FontSize',14)
% ylabel('y-axis(m)','FontSize',14)
% zlabel('RMSE(m)','FontSize',14)
% title('RMSE surf','FontSize',18);

% figure (8)
% plotcube([2.5 2.5 2.5],[0 0 0],.1,[0 0 1]);
% hold on
% plot3(xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*','linewidth',3)
% xlabel('X position in m','FontSize',18)
% ylabel('Y position in m','FontSize',18)
% zlabel('Z position in m','FontSize',18)
% view([20 25 10])

% figure (9)
% dd = [4 8 12 16 20];
% CR_k_L = [0.505	0.259	0.194	0.170	0.149];
% CR_l_L = [0.501	0.266	0.192	0.167	0.149];
% CR_k_N = [1.039	0.578	0.463	0.414	0.371];
% CR_l_N = [1.040	0.568	0.463	0.394	0.371];
% 
% plot(dd, CR_k_L, 'r--O', dd, CR_k_N,'b-^', 'linewidth',2);
% xlabel('distance of two anchors (m)','FontSize',14)
% ylabel('positioning error (m)','FontSize',14)
% legend('LOS','NLOS')
% set(gca, 'XTick', 0:4:20, 'FontSize', 14)
% set(gca, 'YTick', 0.1:0.2:1.1, 'FontSize', 14)

% figure (9)
% dd = [2 4 6 8 10 12 14 16 18 20];
% LS = [1.0569 0.5053 0.3433 0.271 0.2242 0.199 0.175 0.1657 0.1632 0.1568];
% CH = [5.036 2.2084 1.3 0.8967 0.6561 0.5278 0.4385 0.3962 0.3601 0.3435];
% CR = [0.9395 0.5285 0.3253 0.259 0.2256 0.1963 0.1978 0.179 0.1829 0.1722];
% MO = [1.3321 0.8255 0.6084 0.557 0.5111 0.4744 0.4428 0.4458 0.4296 0.4434];
% SI = [1.3 0.6556 0.4286 0.3431 0.2861 0.2485 0.2142 0.2091 0.1926 0.1944];
% ME = [1.5178 0.66 0.4096 0.2748 0.2212 0.1723 0.1472 0.1287 0.113 0.1018];
% HO = [0.3984 0.1862 0.1374 0.1128 0.0979 0.0918 0.0848 0.0772 0.0742 0.0746];
% GD = [1.4562 0.7159 0.5032 0.4451 0.421 0.3768 0.3849 0.3785 0.373 0.3708];
% NL_CR = [2.0749	1.0612	0.7300	0.5712	0.4967	0.4501	0.4166	0.4053	0.3898	0.3820];
% NL_MO = [5.1103	3.0678	2.3957	2.0764	1.8959	1.8167	1.7231	1.7054	1.6804	1.6452];
% NL_SI = [4.8440	2.4627	1.6680	1.2642	1.0538	0.9356	0.8326	0.7986	0.7560	0.7268];
% plot(dd, NL_CR, 'k', dd, NL_MO,'b--', dd, NL_SI,'r-.', 'linewidth',2);
% xlabel('distance of any two anchors (m)','FontSize',14)
% ylabel('NLOS positioning error (m)','FontSize',14)
% legend('CRLB','Approch in [38]','Our apporach')
% set(gca, 'XTick', 0:4:20, 'FontSize', 14)
% % set(gca, 'YTick', 0.1:0.2:1.4, 'FontSize', 14)
% set(gca, 'YTick', 0:1:6, 'FontSize', 14)

% 
% avg = [mean(LS),mean(MO),mean(SI),mean(CR),mean(GD)]
% mstd = [std(LS),std(MO),std(SI),std(CR)]
% 
% figure (10)
% PSI = [10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160];
% LS = [0.6212 0.3113 0.2232 0.1701 0.1544 0.145 0.1447 0.1441 0.1532 0.1703 0.1876 0.2215 0.261 0.3104 0.4289 0.6597];
% CH = [11.49 62.525 26.762 20.156 0.5293 0.362 0.3418 0.3359 0.3387 0.3505 0.3685 0.3853 0.4109 0.4547 0.5045 0.5847];
% CR = [0.6419 0.5214 0.4503 0.2859 0.1824 0.1644 0.1618 0.1462 0.1462 0.149 0.1617 0.1722 0.1734 0.1814 0.1942 0.2242];
% MO = [0.4305 0.4394 0.447 0.4257 0.4478 0.4328 0.4324 0.4292 0.4218 0.4273 0.4308 0.4187 0.4315 0.4279 0.4403 0.434];
% SI = [1.0681 0.5405 0.3757 0.2773 0.2434 0.2082 0.1927 0.1889 0.1866 0.1948 0.2124 0.2291 0.2737 0.3289 0.4577 0.6581];
% ME = [2.8281 115.6161 5.7531 3.2665 0.174 0.107 0.1061 0.1042 0.1018 0.1119 0.1176 0.1224 0.1346 0.1504 0.1671 0.1919];
% HO = [2.655 6.5586 6.1837 4.1543 0.1421 0.0756 0.0726 0.0711 0.0699 0.0791 0.0914 0.0969 0.1139 0.12 0.1427 0.1715];
% GD = [0.7354 0.5826 0.5759 0.9412 0.9334 0.4439 0.3886 0.3833 0.3708 0.3729 0.4108 0.4127 0.4157 0.4607 0.4645 0.5251];
% NL_CR = [1.3004	0.6804	0.5097	0.4501	0.4029	0.3766	0.3639	0.3662	0.3653	0.3894	0.4727	0.5490	0.6517	0.8143	1.0956	1.6665];
% NL_MO = [1.6490	1.6463	1.6350	1.6336	1.6381	1.6463	1.6383	1.6456	1.6316	1.6477	1.6521	1.6450	1.6426	1.6286	1.6376	1.6483];
% NL_SI = [4.1042	2.0429	1.3599	1.0615	0.8843	0.7974	0.7437	0.7185	0.7133	0.7535	0.8139	0.9030	1.0454	1.2670	1.6755	2.5158];
% plot(PSI, NL_CR, 'k', PSI, NL_MO,'b--', PSI, NL_SI,'r-.', 'linewidth',2);
% xlabel('angle (deg)','FontSize',14)
% ylabel('NLOS positioning error (m)','FontSize',14)
% legend('CRLB','Approch in [38]','Our apporach')
% % axis([0 160 0 1.3])
% grid
% set(gca, 'XTick', 0:40:160, 'FontSize', 14)
% % set(gca, 'YTick', 0:0.2:1.2, 'FontSize', 14)
% set(gca, 'YTick', 0:1:4.5, 'FontSize', 14)
% 
% avg = [mean(CR),mean(CH(4:16)),mean(GD),mean(SI),mean(MO)]


% % plot noise
% figure (11)
% plot(1:8000,R1m(1:8000)-R1(1:8000),'b',1:8000,R2m(1:8000)-R2(1:8000),'r','linewidth',1)
% xlabel('samples','FontSize',14)
% ylabel('noise (m)','FontSize',14)
% legend('ST','Gaussion')

% figure (12)
% [p_G,x_G] = hist(R2m(1:8000)-R2(1:8000));
% [p_ST,x_ST] = hist(R1m(1:8000)-R1(1:8000));
% plot(x_G,p_G/sum(p_G),'r',x_ST,p_ST/sum(p_ST),'b','linewidth',2); %PDF
% xlabel('positioning error (m)','FontSize',14)
% ylabel('probability','FontSize',14)
% legend('Gaussion','ST')

