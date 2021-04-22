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
radius = 1*20;
apha_0 = 0*45*d2r;
ft =1*0.01;
wt = 2*pi*ft;
% ======================================================
% Define four UWB sensors locations
% =====================================================
xr1 = 0;% in meter
yr1 = 0;% in meter
xr2 = 0;
yr2 = 50;

theta =5*d2r;
r3 = 50;
xr3 = r3*sin(theta);% in meter
yr3 = r3*cos(theta);% in meter

H = [xr2-xr1, yr2-yr1
    xr3-xr1, yr3-yr1];

q(1) = xr1^2+yr1^2;
q(2) = xr2^2+yr2^2;
q(3) = xr3^2+yr3^2;

% position of mobile
[x_p_N,y_p_N] = trajectory2d_line(radius,apha_0,wt,t0,n);
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

% LS
for k=1:n
    b = [R1m(k)^2-R2m(k)^2-q(1)+q(2)
         R1m(k)^2-R3m(k)^2-q(1)+q(3)];
    r(:,k) = 0.5*inv(H'*H)*H'*b;
end

LS_delta_P = [x_p_N(1:n-1)-r(1,1:n-1); y_p_N(1:n-1)-r(2,1:n-1)];
err1 = LS_delta_P(1,:).^2+LS_delta_P(2,:).^2;
LS = sqrt(mean(err1))

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
CRLB = sqrt(mean(err))
% B = sort(err);
% N = length(B);
% number = fix(0.9973*N);
% B1 = B(number)

figure (1)
plot(x_p_N,y_p_N,xr1,yr1,'r*',xr2,yr2,'r*',xr3,yr3,'r*','linewidth',2)
xlabel('X position in m','FontSize',14)
ylabel('Y position in m','FontSize',14)
title('2D Simulation','FontSize',18);
viscircles([0,0],50,'LineStyle','-.','linewidth',1);
an = {' q_1',' q_2',' q_3'};
xt=[xr1,xr2,xr3];
yt=[yr1,yr2,yr3];
text(xt,yt,an,'fontsize',18,'linewidth',2);
axis([-5 55 -5 55])
grid

PSI = [5,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170];
error2_LS = [1.2986,0.6219,0.308,0.2098,0.1767,0.1533,0.1453,0.1445,0.1542,0.1612,0.1739,0.1983,0.2332,0.2699,0.3306,0.4537,0.6685,1.3388];
error2_CR = [0.656,0.316,0.1558,0.1042,0.0856,0.0749,0.0728,0.0752,0.0751,0.0805,0.0853,0.0987,0.1172,0.1354,0.1701,0.2264,0.342,0.6924];

figure (2)
% plot(PSI, error2_LS, 'r--O', PSI, error2_CR*2, 'b-^', 'linewidth',1);
plot(PSI, error2_LS, 'b--O','linewidth',1);
% xlabel('psi in deg','FontSize',14)
ylabel('RMSE','FontSize',14)
% title('position error along psi','FontSize',18);
% legend('LS','CRLB')

error3_LS = [0.0167,0.0481,0.0807,0.1177,0.1433,0.1807,0.2093,0.2457,0.2709,0.316,0.349,0.3837,0.3965,0.4385,0.4692,0.5041,0.5184,0.5834,0.608,0.643,0.6686,0.6809,0.7309,0.7639,0.8127,0.8316];
error3_CR = [0.0083,0.0251,0.0404,0.0575,0.0743,0.0905,0.1069,0.1219,0.1398,0.1552,0.1727,0.187,0.2004,0.2228,0.2398,0.2522,0.2651,0.2858,0.3028,0.317,0.338,0.3434,0.3697,0.3747,0.3969,0.4199];

figure (3)
% plot(0.01:0.02:0.51, error3_LS, 'r--o',0.01:0.02:0.51, error3_CR*2, 'b-^','linewidth',1);
plot(0.01:0.02:0.51, error3_CR*2, 'b--o','linewidth',1);
ylabel('RMSE','FontSize',14)
% legend('LS','CRLB')

error4_LS = [0.6164,0.5878,0.5625,0.5374,0.5124,0.4874,0.4624,0.4374,0.4124,0.3874,0.3624,0.3374,0.3124,0.2874,0.2624,0.2374,0.2124,0.1875,0.1625,0.1375,0.1125,0.0875,0.0625,0.0375,0.0126,0.0126];
error4_CR = [2.836,0.9312,0.5501,0.3954,0.3054,0.2554,0.2108,0.1856,0.1634,0.1489,0.1347,0.124,0.1203,0.1114,0.106,0.1018,0.0983,0.0939,0.0921,0.0894,0.0865,0.0861,0.0837,0.0836,0.0815,0.0796];

figure (4)
% plot(3:2:51, error4_LS(2:26),'r--O',3:2:51, error4_CR(2:26),'b-^','linewidth',1);
plot(3:2:51, error4_CR(2:26),'b--o','linewidth',1);
% xlabel('  in m','FontSize',14)
ylabel('RMSE','FontSize',14)
% title('RMSE changes with  ','FontSize',18);
% legend('LS','CRLB')

error5_LS = [0.6123,0.5873,0.5623,0.5373,0.5123,0.4874,0.4624,0.4374,0.4124,0.3874,0.3624,0.3374,0.3124,0.2874,0.2624,0.2374,0.2124,0.1875,0.1625,0.1375,0.1125,0.0875,0.0625,0.0375,0.0126,0.0126];
error5_CR = [2.8703,0.9317,0.5584,0.3886,0.3013,0.2473,0.217,0.1846,0.1671,0.1511,0.1396,0.125,0.1167,0.1115,0.1046,0.1005,0.0977,0.0968,0.0925,0.0912,0.0872,0.0876,0.0839,0.0822,0.082,0.08];

figure (7)
% plot(3:2:51, error5_LS(2:26),'r--o',3:2:51, error5_CR(2:26),'b-^','linewidth',1);
plot(3:2:51, error5_CR(2:26),'b--o','linewidth',1);
% xlabel('  in m','FontSize',14)
ylabel('RMSE','FontSize',14)
% title('RMSE changes with  ','FontSize',18);
% legend('LS','CRLB')
% 
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

% sig = 0.6827;
% 
% B = median(error2_LS)
% N = length(B);
% number = fix(sig*N);
% B1 = B(number)
% 
% C = median(error2_CR)
% N = length(C);
% number = fix(sig*N);
% C1 = C(number)
% 
% D = median(error3_LS)
% N = length(D);
% number = fix(sig*N);
% D1 = D(number)
% 
% E = median(error3_CR)
% N = length(E);
% number = fix(sig*N);
% E1 = E(number)
% 
% L = median(error4_LS)
% N = length(L);
% number = fix(sig*N);
% L1 = L(number)
% 
% J = median(error4_CR)
% N = length(J);
% number = fix(sig*N);
% J1 = J(number)
% 
% K = median(error5_LS)
% N = length(K);
% number = fix(sig*N);
% K1 = K(number)
% 
% M = median(error5_CR)
% N = length(M);
% number = fix(sig*N);
% M1 = M(number)