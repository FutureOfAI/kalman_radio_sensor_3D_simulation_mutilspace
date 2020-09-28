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
radius = 1*25;
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
yr1 = 25;% in meter
zr1 = 31;% in meter
xr2 = 0;
yr2 = -25;
zr2 = 31;
xr3 = 25;% in meter
yr3 = 0;% in meter
zr3 = 29;
xr4 = -25;
yr4 = 1;
zr4 = 29;% in meter

H = [xr2-xr1, yr2-yr1, zr2-zr1
    xr3-xr1, yr3-yr1, zr3-zr1
    xr4-xr1, yr4-yr1, zr4-zr1];

[x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N,z_p_N,z_v_N,z_a_N] = trajectory3d_line(radius,gama_0,apha_0,wt,t0);
% x_p_N = zeros(1,n);
% y_p_N = zeros(1,n);
% z_p_N = zeros(1,n);
% for i=1:length
%     for j=1:width
%         x_p_N((i-1)*width+j) = j*0.1;
%         y_p_N((i-1)*width+j) = i*0.1;
%     end
% end

% =====================================================================================
% Define "4 radio sensor" parameters (noise)
% =====================================================================================
radiosensor_err_factor = 1.0;
sig_x_r=radiosensor_err_factor*0.1;              % radio sensor measurement noise in meters x-direction
sig_y_r=radiosensor_err_factor*0.1;              % radio sensor measurement noise in meters y-direction
%======================================================================================
[R1m,R2m,R3m,R4m,nvx_r,nvy_r] = radio_sensor3d_m_4(xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,x_p_N,y_p_N,z_p_N,sig_x_r,sig_y_r,n,m,0);%4 four radio sensors

for k=1:n
    delta_R = [-2*R1m(k)*nvx_r(k)+2*R2m(k)*nvy_r(k)
                -2*R1m(k)*nvx_r(k)+2*R3m(k)*nvy_r(k)
                -2*R1m(k)*nvx_r(k)+2*R4m(k)*nvy_r(k)];
    delta_P(:,k) = 0.5*inv(H)*delta_R;
    conv_dR = [R1m(k)^2*nvx_r(k)^2+R2m(k)^2*nvy_r(k)^2, R1m(k)^2*nvx_r(k)^2, R1m(k)^2*nvx_r(k)^2;
                    R1m(k)^2*nvx_r(k)^2, R1m(k)^2*nvx_r(k)^2+R3m(k)^2*nvy_r(k)^2, R1m(k)^2*nvx_r(k)^2;
                    R1m(k)^2*nvx_r(k)^2, R1m(k)^2*nvx_r(k)^2, R1m(k)^2*nvx_r(k)^2+R4m(k)^2*nvy_r(k)^2];
    E_dp(:,:,k) = inv(H)*conv_dR*inv(H)';
end

for k=1:n
   plot_E_dp(1,k) =  E_dp(1,1,k);
   plot_E_dp(2,k) =  E_dp(2,2,k);
   plot_E_dp(3,k) =  E_dp(3,3,k);
end

figure(1)
plot(1:n, delta_P(3,1:n));
ylabel('z-axis positioning error in m')

figure(2)
subplot(311)
plot(1:n, plot_E_dp(1,1:n));
subplot(312)
plot(1:n, plot_E_dp(2,1:n));
subplot(313)
plot(1:n, plot_E_dp(3,1:n));


