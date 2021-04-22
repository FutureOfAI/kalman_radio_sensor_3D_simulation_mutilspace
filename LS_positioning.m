%
close all;
clear all;
clc;

% group1_psi_70_deg.csv
% group2_psi_40_deg.csv
% group3_psi_10_deg.csv
% group4_psi_100_deg.csv
% group5_psi_130_deg.csv
% group6_psi_55_deg.csv
% group7_psi_25_deg.csv
% group8_psi_90_deg.csv
% group10_psi_35_deg01.csv
% group10_psi_35_deg02.csv
% group11_psi_50_deg.csv
% group12_psi_65_deg.csv
% group13_psi70_phi20_l4.79.csv
% group14_psi70_phi20_l3.79.csv
% group15_psi70_phi20_l2.79.csv
% group16_psi70_phi20_l1.79.csv
% group17_psi70_phi20_m1.69.csv
% group18_psi70_phi20_m1.19.csv
uwb = load('group1_psi_70_deg.csv'); % in the exp_data floder
[count,row] = size(uwb);
post = uwb(:,7:10)'; % in m
d2r = (pi/180);

%% clear bad distance data
for j_1= 2:800
    
    if(post(1,j_1)>15)
        post(1,j_1) = post(1,j_1-1);
    end
    
    if(post(2,j_1)>15)
        post(2,j_1) = post(2,j_1-1);
    end
    
    if(post(3,j_1)>15)
        post(3,j_1) = post(3,j_1-1);
    end
    
    if(post(4,j_1)>15)
        post(4,j_1) = post(4,j_1-1);
    end
end

% anchor bias
post(1,:) = post(1,:)+0.057;
post(2,:) = post(2,:)+0.766;
post(3,:) = post(3,:)+0.138;
post(4,:) = post(4,:)+0.415;

% pos(:,1) = post(:,1);
% for i=1:10:8000
%    for j=1:10
%       pos(:,i+j) = post(:,fix(i/10)+1); 
%    end
% end

% ======================================================
% Define four UWB sensors locations
% =====================================================
xr1 = 0;% in meter
yr1 = 0;% in meter
zr1 = 2.52;% in meter
xr2 = 5.79;
yr2 = 0;
zr2 = 2.52;

psi = 70*d2r;
r3 = 5.79;
xr3 = r3*cos(psi);% in meter
yr3 = r3*sin(psi);% in meter
zr3 = 2.52;

% phi = 0*d2r;
% beta = 45*d2r;
% alpha = 45*d2r;
% r4 = 50;
% xr4 = r4*sin(phi)*cos(beta);
% yr4 = r4*sin(phi)*cos(alpha);
% zr4 = r4*cos(phi);% in meter
xr4 = 0.41;
yr4 = 0.64;
zr4 = 0.464;% in meter

H = 2*[xr2-xr1, yr2-yr1, zr2-zr1
    xr3-xr1, yr3-yr1, zr3-zr1
    xr4-xr1, yr4-yr1, zr4-zr1];

k(1) = xr1^2+yr1^2+zr1^2;
k(2) = xr2^2+yr2^2+zr2^2;
k(3) = xr3^2+yr3^2+zr3^2;
k(4) = xr4^2+yr4^2+zr4^2;

for i=1:800
    b = [post(1,i)^2-post(2,i)^2-k(1)+k(2)
         post(1,i)^2-post(3,i)^2-k(1)+k(3)
         post(1,i)^2-post(4,i)^2-k(1)+k(4)];
    r(:,i) = inv(H'*H)*H'*b;
%     conv_dR(:,:,k) = [R1m(k)^2*nvx_r(k)^2+R2m(k)^2*nvy_r(k)^2, R1m(k)^2*nvx_r(k)^2, R1m(k)^2*nvx_r(k)^2;
%                     R1m(k)^2*nvx_r(k)^2, R1m(k)^2*nvx_r(k)^2+R3m(k)^2*nvy_r(k)^2, R1m(k)^2*nvx_r(k)^2;
%                     R1m(k)^2*nvx_r(k)^2, R1m(k)^2*nvx_r(k)^2, R1m(k)^2*nvx_r(k)^2+R4m(k)^2*nvy_r(k)^2];
%     E_dp(:,:,k) = inv(H)*conv_dR(:,:,k)*inv(H)';
end

% %
% figure (1)
% subplot(411)
% plot(1:8000,pos(1,1:8000))
% ylabel('Anchor1 distance (m)')
% subplot(412)
% plot(1:8000,pos(2,1:8000))
% ylabel('Anchor2 distance (m)')
% subplot(413)
% plot(1:8000,pos(3,1:8000))
% ylabel('Anchor3 distance (m)')
% subplot(414)
% plot(1:8000,pos(4,1:8000))
% ylabel('Anchor4 distance (m)')
% %




