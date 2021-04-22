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
radius = 1*20;
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
zr1 = 0;% in meter
xr2 = 0;
yr2 = 50;
zr2 = 0;

psi = 0*d2r;
r3 = 50;
xr3 = r3*cos(psi);% in meter
yr3 = r3*sin(psi);% in meter
zr3 = 0;

phi = 0*d2r;
beta = 45*d2r;
alpha = 45*d2r;
r4 = 50;
xr4 = r4*sin(phi)*cos(beta);
yr4 = r4*sin(phi)*cos(alpha);
zr4 = r4*cos(phi);% in meter

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


[x_p_N,y_p_N,z_p_N] = trajectory3d_line_v2(radius,wt,t0);


% =====================================================================================
% Define "4 radio sensor" parameters (noise)
% =====================================================================================
radiosensor_err_factor = 1.0;
sigma = 0.1;
sig_x_r=radiosensor_err_factor*sigma;              % radio sensor measurement noise in meters x-direction
sig_y_r=radiosensor_err_factor*sigma;              % radio sensor measurement noise in meters y-direction
%======================================================================================
[R1m,R2m,R3m,R4m,nvx_r,nvy_r] = radio_sensor3d_m_4(xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,x_p_N,y_p_N,z_p_N,sig_x_r,sig_y_r,n-1,m,0);%4 four radio sensors

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
LS = mean(err1)

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
CRLB = mean(err)

figure (1)
plot3(x_p_N,y_p_N,z_p_N,xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*','linewidth',2)
xlabel('X position in m','FontSize',14)
ylabel('Y position in m','FontSize',14)
zlabel('Z position in m','FontSize',14)
title('3D Simulation','FontSize',18);
an = {' q_1',' q_2',' q_3',' q_4'};
xt=[xr1,xr2,xr3,xr4];
yt=[yr1,yr2,yr3,yr4];
zt=[zr1,zr2,zr3,zr4];
text(xt,yt,zt,an,'fontsize',18,'linewidth',2);
% view(30,30)
% axis equal
grid

figure (6)
plot3(r(1,:),r(2,:),r(3,:),xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*','linewidth',2)

PSI = [5,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170];
error2_LS = [2.5326,0.6939,0.215,0.1287,0.0916,0.0742,0.06,0.0486,0.0433,0.0407,0.0446,0.0586,0.0879,0.1543,0.3001,0.6653,1.8466,8.9622];
error2_CR = [2.5286,0.6272,0.1755,0.0968,0.0693,0.0587,0.0546,0.0536,0.0545,0.0587,0.0675,0.0769,0.0954,0.1233,0.1858,0.3055,0.6544,2.6209];

figure (2)
% plot(PSI, sqrt(error2_LS), 'r--o', PSI, sqrt(error2_CR), 'b-^', 'linewidth',1);
plot(PSI, sqrt(error2_LS), 'b--o', 'linewidth',1);
% xlabel('psi in deg','FontSize',14)
ylabel('RMSE','FontSize',14)
% title('position error along psi','FontSize',18);
% legend('LS','CRLB')

PHI = [-80,-70,-60,-50,-40,-30,-20,-10,0,10,20,30,40,50,60,70,80];
error3_LS = [3.2406,0.7634,0.3357,0.1941,0.1214,0.0875,0.0656,0.0489,0.0407,0.0402,0.0471,0.0619,0.0877,0.1278,0.1954,0.3391,0.9039];
error3_CR = [3.4157,0.8255,0.3724,0.2172,0.1464,0.1066,0.0821,0.067,0.0589,0.0551,0.0516,0.0495,0.0522,0.0601,0.0818,0.1505,0.5572];

figure (3)
% plot(PHI, sqrt(error3_LS), 'r--o', PHI, sqrt(error3_CR), 'b-^', 'linewidth',1);
plot(PHI, sqrt(error3_LS), 'b--o', 'linewidth',1);
% xlabel('psi in deg','FontSize',14)
ylabel('RMSE','FontSize',14)
% title('position error along psi','FontSize',18);
% legend('LS','CRLB')

% error3a = [5.5353,3.0284,1.9243,1.3704,1.0843,0.8911,0.8455,0.747,0.6464,0.6155,0.6068,0.6172,0.6228,0.6636,0.842,1.1488];
% error3b = [0.8369,0.2157,0.1066,0.0637,0.046,0.0425,0.0423,0.0431,0.0463,0.0593,0.0807,0.1178,0.1742,0.2389,0.4652,0.944];
% figure (8)
% plot(10:10:160, sqrt(error3b),'r--o',10:10:160, sqrt(error2),'g--*',10:10:160, sqrt(error3a),'b--','linewidth',1);
% xlabel('angle in deg','FontSize',14)
% ylabel('RMSE','FontSize',14)
% title('position error along angle','FontSize',18);
% legend('2d theta','3d psi','3d phi + 90deg')


error4_LS = [0.0004,0.0037,0.0103,0.0202,0.0342,0.0499,0.0695,0.0918,0.118,0.1521,0.1764,0.2096,0.2492,0.2906,0.3372,0.4016,0.4383,0.5036,0.543,0.6435,0.7028,0.7507,0.8269,0.9064,0.9973,1.1189];
error4_CR = [0.0006,0.0054,0.0147,0.0291,0.0489,0.0727,0.1002,0.1335,0.1717,0.2207,0.2558,0.3085,0.37,0.4215,0.4932,0.5674,0.639,0.7258,0.8062,0.9195,1.0039,1.1,1.1887,1.3089,1.432,1.5939];

figure (4)
% plot(0.01:0.02:0.51, sqrt(error4_LS),'r--o',0.01:0.02:0.51, sqrt(error4_CR), 'b-^','linewidth',1);
plot(0.01:0.02:0.51, sqrt(error4_CR), 'b--o','linewidth',1);
% xlabel('ranging noise standard deviation in m','FontSize',14)
ylabel('RMSE','FontSize',14)
% title('position error along standard deviation','FontSize',18);
% legend('LS','CRLB')
%
error5_LS = [48.2,5.5428,2.0376,1.0477,0.6215,0.4533,0.3291,0.2544,0.1984,0.1682,0.1446,0.1232,0.1094,0.0994,0.0868,0.0801,0.0707,0.0632,0.0577,0.0545,0.0533,0.0469,0.0459,0.0425,0.0413,0.0403];
error5_CR = [48.917,5.4828,1.9404,1.0294,0.5965,0.4256,0.3004,0.235,0.1859,0.1575,0.139,0.1177,0.1091,0.1011,0.092,0.0874,0.0802,0.0749,0.0713,0.0686,0.0686,0.0648,0.0639,0.0615,0.0598,0.0578];

figure (5)
% plot(1:2:51, sqrt(error5_LS), 'r--o',1:2:51, sqrt(error5_CR), 'b-^','linewidth',1);
plot(1:2:51, sqrt(error5_LS), 'b--o','linewidth',1);
% title('RMSE changes with  ','FontSize',18);
ylabel('RMSE','FontSize',14)
% xlabel('  in m','FontSize',14)
% legend('LS','CRLB')
%
error6_LS = [47.716,5.4682,1.9584,1.0136,0.6348,0.4339,0.3257,0.2511,0.2079,0.1707,0.1422,0.1258,0.1117,0.099,0.085,0.0784,0.0695,0.065,0.0577,0.0551,0.0496,0.0493,0.0459,0.0445,0.041,0.0399];
error6_CR = [49.005,5.3244,1.9409,1.0231,0.6272,0.4119,0.3,0.2342,0.188,0.1619,0.1375,0.1227,0.1101,0.0992,0.0913,0.0835,0.0798,0.0781,0.0745,0.0684,0.0669,0.0639,0.0628,0.0622,0.0585,0.0585];

figure (13)
% plot(1:2:51, sqrt(error6_LS), 'r--o',1:2:51, sqrt(error6_CR), 'b-^','linewidth',1);
plot(1:2:51, sqrt(error6_LS), 'b--o','linewidth',1);
% title('RMSE changes with  ','FontSize',18);
ylabel('RMSE','FontSize',14)
% xlabel('  in m','FontSize',14)
% legend('LS','CRLB')

%
error7_LS = [48.858,5.1425,1.9671,1.1016,0.6389,0.4543,0.3161,0.2608,0.2106,0.1723,0.1452,0.121,0.1082,0.0952,0.0896,0.078,0.0687,0.0647,0.0609,0.0567,0.051,0.0471,0.0442,0.0435,0.042,0.0396];
error7_CR = [48.774,5.2253,1.9247,0.988,0.611,0.4177,0.2948,0.242,0.1926,0.1572,0.1368,0.1166,0.1095,0.0973,0.0924,0.0862,0.0799,0.0767,0.0747,0.0709,0.0665,0.0643,0.0625,0.0621,0.0605,0.059];

figure (15)
% plot(1:2:51, sqrt(error7_LS), 'r--o',1:2:51, sqrt(error7_CR), 'b-^','linewidth',1);
plot(1:2:51, sqrt(error7_LS), 'b--o','linewidth',1);
% title('RMSE changes with  ','FontSize',18);
ylabel('RMSE','FontSize',14)
% xlabel('  in m','FontSize',14)
% legend('LS','CRLB')
%
error5a3 = [7.2894,4.9889,2.9127,2.614,2.0181,1.7546,1.5356,1.3796,1.2689,1.1938,1.1145,1.0429,1.0066,0.9088,0.805,0.8107,0.8124,0.8159,0.7771,0.7679,0.7591,0.7265,0.719,0.6868];
error5a2 = [7.6451,4.4306,3.2746,2.4761,2.1368,1.7361,1.5318,1.4405,1.265,1.1626,1.102,1.0045,0.978,0.9132,0.8931,0.8663,0.8424,0.8165,0.8152,0.7888,0.7802,0.7398,0.7387,0.724];
error5a1 = [7.6653,4.3534,3.3458,2.4442,2.0498,1.7493,1.5891,1.4478,1.28,1.1843,1.1003,1.0378,0.9931,0.9218,0.8953,0.9016,0.8767,0.821,0.788,0.7748,0.773,0.7661,0.7542,0.7269];
error5b2 = [9.3186,3.2345,1.731,0.9901,0.6443,0.4504,0.3652,0.2917,0.2418,0.2027,0.162,0.1353,0.1254,0.1214,0.1011,0.0944,0.0841,0.0815,0.0758,0.067,0.0642,0.0616,0.0582,0.0566];
error5b1 = [8.6929,2.9793,1.5437,0.9098,0.6232,0.4339,0.3302,0.2719,0.1968,0.1984,0.1319,0.1212,0.1204,0.1039,0.0828,0.0923,0.0802,0.0818,0.0631,0.0623,0.0592,0.0602,0.0567,0.0536];
figure (9)
plot(3:2:49, sqrt(error5b1),'r--o',3:2:49, sqrt(error5b2),'r--*',3:2:49, sqrt(error5a1),'b--o',3:2:49, sqrt(error5a2),'b--*',3:2:49, sqrt(error5a3),'b--','linewidth',1);
xlabel('anchor distance in m','FontSize',14)
ylabel('RMSE','FontSize',14)
title('position error along anchor distance','FontSize',18);
legend('2d-k','2d-l','3d-k','3d-l','3d-m')

% figure(10)
% X22 = X2;
% Y22 = Y2;
% Z22 = 100*ones(20,20);
% for k = 20
%     for j = 1:20
%        for i = 1:20
%             zz(i,j) = sqrt(err((k-1)*20*20+(j-1)*20+i));
%        end
%     end
% end
% surf(X22,Y22,Z22,zz);
% hold on
% X22 = 0*ones(20,20);
% Y22 = Y2;
% Z22 = Z2;
% for k = 1:20
%     for j = 1:20
%        for i = 6
%             zz(i,j) = sqrt(err((k-1)*20*20+(j-1)*20+i));
%        end
%     end
% end
% surf(X22,Y22,Z22,zz);
% hold on
% X22 = X2';
% Y22 = 0*ones(20,20);
% Z22 = Z2;
% for k = 1:20
%     for j = 6
%        for i = 1:20
%             zz(i,j) = sqrt(err((k-1)*20*20+(j-1)*20+i));
%        end
%     end
% end
% surf(X22,Y22,Z22,zz);
% hold on
% plot3(xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*','linewidth',2)
% xlabel('x-axis in m','FontSize',14)
% ylabel('y-axis in m','FontSize',14)
% zlabel('Z-axis in m','FontSize',14)
% title('RMSE surf','FontSize',18);
% cb = colorbar;
% 
% 
% figure(6)
% X22 = X2;
% Y22 = Y2;
% Z22 = 25*ones(20,20);
% for k = 1
%     for j = 1:20
%        for i = 1:20
%             zz(i,j) = sqrt(err((k-1)*20*20+(j-1)*20+i));
%        end
%     end
% end
% % mesh(X2,Y2,Z2,zz);
% contour(X2,Y2,zz);
% hold on
% plot3(xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*','linewidth',2)
% xlabel('x-axis(m)','FontSize',14)
% ylabel('y-axis(m)','FontSize',14)
% % zlabel('Z-axis(m)','FontSize',14)
% title('RMSE contour projected on the Y-X plane','FontSize',18);
% 
% figure(11)
% X22 = 25*ones(20,20);
% Y22 = Y2;
% Z22 = Z2;
% for k = 1:20
%     for j = 1:20
%        for i = 1
%             zz(i,j) = sqrt(err((k-1)*20*20+(j-1)*20+i));
%        end
%     end
% end
% contour(X2,Y2,zz);
% hold on
% plot(yr1,zr1,'r*',yr2,zr2,'r*',yr3,zr3,'r*',yr4,zr4,'r*','linewidth',2)
% xlabel('y-axis(m)','FontSize',14)
% ylabel('z-axis(m)','FontSize',14)
% % zlabel('Z-axis in m','FontSize',14)
% title('RMSE contour projected on the Z-Y plane','FontSize',18);
% 
% figure(12)
% X22 = X2;
% Y22 = 25*ones(20,20);
% Z22 = Z2;
% for k = 1:20
%     for j = 1
%        for i = 1:20
%             zz(i,j) = sqrt(err((k-1)*20*20+(j-1)*20+i));
%        end
%     end
% end
% contour(X2,Y2,zz);
% hold on
% plot(zr1,xr1,'r*',zr2,xr2,'r*',zr3,xr3,'r*',zr4,xr4,'r*','linewidth',2)
% xlabel('z-axis(m)','FontSize',14)
% ylabel('x-axis(m)','FontSize',14)
% % zlabel('Z-axis in m','FontSize',14)
% title('RMSE contour projected on the X-Z plane','FontSize',18);

% anchp = [xr1 yr1 zr1
%         xr3 yr3 zr3
%         xr2 yr2 zr2
%         xr4 yr4 zr4
%         xr1 yr1 zr1];

% figure (5)
% plotcube([50 50 25],[-25 -25 0],.1,[0 0 1]);
% hold on
% % plot3(xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*','linewidth',2)
% plot3(anchp(:,1),anchp(:,2),anchp(:,3),'r--*','linewidth',2);
% xlabel('X position in m','FontSize',18)
% ylabel('Y position in m','FontSize',18)
% zlabel('Z position in m','FontSize',18)
% text(-40,0,40,'\fontsize{20}\color{black}Anchors distribution')

error2d = [0.0005,0.005,0.0138,0.0257,0.0435,0.0723,0.0967,0.1269,0.1607,0.199,0.2676,0.2944,0.3677,0.4134,0.5173,0.5642,0.6093,0.6875,0.7827,0.8945,0.9092,0.973,1.1189,1.27,1.3571];
error3d = [0.0694,0.22,0.357,0.5389,0.6836,0.8139,0.9335,1.2734,1.2247,1.3696,1.4206,1.5916,1.6634,1.9866,1.9706,2.1153,2.2472,2.3462,2.681,2.9447,2.9603,3.194,3.228,3.2634,3.4205];
figure (7)
plot(0.01:0.02:0.49, sqrt(error2d),'r--o',0.01:0.02:0.49, sqrt(error3d),'b--*','linewidth',1);
xlabel('ranging noise standard deviation in m','FontSize',14)
ylabel('RMSE','FontSize',14)
title('position error between 2d and 3d','FontSize',18);
legend('2d var','3d var')

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
% 
% P = median(error6_LS)
% N = length(P);
% number = fix(sig*N);
% P1 = P(number)
% 
% Q = median(error6_CR)
% N = length(Q);
% number = fix(sig*N);
% Q1 = Q(number)
% 
% V = median(error7_LS)
% N = length(V);
% number = fix(sig*N);
% V1 = V(number)
% 
% W = median(error7_CR)
% N = length(W);
% number = fix(sig*N);
% W1 = W(number)