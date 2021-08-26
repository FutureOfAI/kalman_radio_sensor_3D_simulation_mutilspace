%% 
% program name uwb_distribution.m
% evaluate the impact of uwb errors in 3D positioning
% 2021-05-22
close all;
clear all;
clc;

r2d = (180/pi);
d2r = (pi/180);

% ======================================================
% Define four UWB sensors locations
% =====================================================
xr1 = 0;% in meter
yr1 = 0;% in meter
zr1 = 5;% in meter
xr2 = 0;
yr2 = 5;
zr2 = 5;

for k=1:17
    psi = 9*10*d2r;
    r3 = 5;
    xr3 = r3*sin(psi);% in meter
    yr3 = r3*cos(psi);% in meter
    zr3 = 5;

    phi = (k-9)*10*d2r;
%     phi = 0*d2r;
    beta = 45*d2r;
    alpha = 45*d2r;
    r4 = 5;
    xr4 = r4*sin(phi)*cos(beta);
    yr4 = r4*sin(phi)*cos(alpha);
    zr4 = r4-r4*cos(phi);% in meter

    a3(:,k) = [xr3,yr3,zr3];
    a4(:,k) = [xr4,yr4,zr4];
    
    H = [xr2-xr1, yr2-yr1, zr2-zr1
        xr3-xr1, yr3-yr1, zr3-zr1
        xr4-xr1, yr4-yr1, zr4-zr1];

    q(1) = xr1^2+yr1^2+zr1^2;
    q(2) = xr2^2+yr2^2+zr2^2;
    q(3) = xr3^2+yr3^2+zr3^2;
    q(4) = xr4^2+yr4^2+zr4^2;

%     UWB mobile node position
%     x = 2; 
%     y = 2; 
%     z = 2;
%     R1 = sqrt((x-xr1)^2+(y-yr1)^2+(z-zr1)^2);
%     R2 = sqrt((x-xr2)^2+(y-yr2)^2+(z-zr1)^2);
%     R3 = sqrt((x-xr3)^2+(y-yr3)^2+(z-zr1)^2);
%     R4 = sqrt((x-xr4)^2+(y-yr4)^2+(z-zr4)^2);
    R1 = 4.1231;
    R2 = 4.6904;
    R3 = 4.6904;
    R4 = 3.4641;

    b = [R1^2-R2^2-q(1)+q(2)
         R1^2-R3^2-q(1)+q(3)
         R1^2-R4^2-q(1)+q(4)];
    r(:,k) = 0.5*inv(H'*H)*H'*b;

end

figure (1)
% plot3(r(1,1:14),r(2,1:14),r(3,1:14),'bo',xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',a3(1,1:14),a3(2,1:14),a3(3,1:14),'m*',xr4,yr4,zr4,'r*','linewidth',2)
plot3(r(1,5:17),r(2,5:17),r(3,5:17),'bo',xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',a4(1,5:17),a4(2,5:17),a4(3,5:17),'m*','linewidth',2)
xlabel('x(m)','FontSize',14)
ylabel('y(m)','FontSize',14)
zlabel('z(m)','FontSize',14)
title('3D Simulation','FontSize',18);
hold on
% t = (-90:0.5:90)*pi/180;
% x = 5*cos(t);
% y = 5*sin(t);
% z = 5*ones(361);
% plot3(x,y,z,'r--')
view(15,30)
% axis equal
grid




