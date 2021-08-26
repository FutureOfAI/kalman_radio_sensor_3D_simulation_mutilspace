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
% Define three UWB sensors locations
% =====================================================
xr1 = 0;% in meter
yr1 = 0;% in meter
xr2 = 0;
yr2 = 5;

for k=1:17
    theta =k*10*d2r;
    r3 = 5;
    xr3 = r3*sin(theta);% in meter
    yr3 = r3*cos(theta);% in meter

    a3(:,k) = [xr3, yr3];
    
    H = [xr2-xr1, yr2-yr1
        xr3-xr1, yr3-yr1];

    q(1) = xr1^2+yr1^2;
    q(2) = xr2^2+yr2^2;
    q(3) = xr3^2+yr3^2;

    % UWB mobile node position
%     x = 2; 
%     y = 2; 
%     r1 = sqrt((x-xr1)^2+(y-yr1)^2);
%     r2 = sqrt((x-xr2)^2+(y-yr2)^2);
%     r3 = sqrt((x-xr3)^2+(y-yr3)^2);
    R1 = 2.8284;
    R2 = 3.6056;
    R3 = 3.6056;

%     b = [R1^2-R2^2-q(1)+q(2)
%          R1^2-R3^2-q(1)+q(3)];
%     r(:,k) = 0.5*inv(H'*H)*H'*b;
    [locx,locy] = triposition(xr1,yr1,R1,xr2,yr2,R2,xr3,yr3,R3); 
    r(:,k) = [locx,locy];

    dR1c(k) = sqrt((xr1-r(1,k))^2+(yr1-r(2,k))^2)-R1;
    dR2c(k) = sqrt((xr2-r(1,k))^2+(yr2-r(2,k))^2)-R2;
    dR3c(k) = sqrt((xr3-r(1,k))^2+(yr3-r(2,k))^2)-R3;
    
    % add noise to distance
    n = 1000;    
    % =====================================================================================
    % Define "3 radio sensor" parameters (noise)
    % =====================================================================================
    radiosensor_err_factor = 1.0;
    sigma = 0.1;
    sig_x_r=radiosensor_err_factor*sigma;              % radio sensor measurement noise in meters x-direction
    sig_y_r=radiosensor_err_factor*sigma;              % radio sensor measurement noise in meters y-direction
    sig_z_r=radiosensor_err_factor*sigma;
    nvx_r=normrnd(0,sig_x_r,1,n);
    nvy_r=normrnd(0,sig_y_r,1,n);
    nvz_r=normrnd(0,sig_z_r,1,n);
    for i=1:n
       R1m(i) = R1 + nvx_r(i);
       R2m(i) = R2 + nvx_r(i);
       R3m(i) = R3 + nvx_r(i);
    end
     % LS
    for i=1:n
        b = [R1m(i)^2-R2m(i)^2-q(1)+q(2)
             R1m(i)^2-R3m(i)^2-q(1)+q(3)];
        p(:,i) = 0.5*inv(H'*H)*H'*b;
    end

    LS_delta_P = [r(1,k)-p(1,:); r(2,k)-p(2,:)];
    err1 = LS_delta_P(1,:).^2+LS_delta_P(2,:).^2;
    psi(k,:) = sqrt(err1);   
    
end

mean_psi = mean(psi,2);

n = length(psi(1,:));
for i=1:11
    a(i,1)=length(find(psi(i,:)<=0.01))/n; % <0.1
    a(i,2)=length(find(psi(i,:)<=0.02))/n; % <0.2
    a(i,3)=length(find(psi(i,:)<=0.03))/n; % <0.3
    a(i,4)=length(find(psi(i,:)<=0.04))/n; % <0.4
    a(i,5)=length(find(psi(i,:)<=0.05))/n; % <0.5
    a(i,6)=length(find(psi(i,:)<=0.06))/n; % <0.6
    a(i,7)=length(find(psi(i,:)<=0.07))/n; % <0.7
    a(i,8)=length(find(psi(i,:)<=0.08))/n; % <0.8
    a(i,9)=length(find(psi(i,:)<=0.09))/n; % <0.9
    a(i,10)=length(find(psi(i,:)<=0.1))/n; % <1.0
end
t = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1];
figure (1)
plot(t,a(1,:),'-o',t,a(2,:),'-+',t,a(3,:),'-.',t,a(4,:),t,a(5,:),'m',t,a(6,:),'k',t,a(7,:),'b',t,a(8,:),'g',t,a(9,:),'r','linewidth',2);
xlabel('positioning error (m)','FontSize',14)
ylabel('Probability','FontSize',14)
% figure (2)
% plot(t,a(17,:),'-o',t,a(16,:),'-+',t,a(15,:),'-.',t,a(14,:),'m',t,a(13,:),'k',t,a(12,:),'b',t,a(11,:),'g',t,a(10,:),'r','linewidth',2);
% xlabel('positioning error (m)','FontSize',14)
% ylabel('Probability','FontSize',14)



% figure (1)
% plot(r(1,1:14),r(2,1:14),'bo',xr1,yr1,'r*',xr2,yr2,'r*',a3(1,1:14),a3(2,1:14),'m*','linewidth',2)
% xlabel('x(m)','FontSize',14)
% ylabel('y(m)','FontSize',14)
% title('2D Simulation','FontSize',18);
% hold on
% t = (-90:0.5:90)*pi/180;
% x = 5*cos(t);
% y = 5*sin(t);
% plot(x,y,'r--')
% axis([-5 5.5 -5.5 5.5])
% axis equal




