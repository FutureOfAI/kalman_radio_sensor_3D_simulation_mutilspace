function [R1m,R2m,R3m,R4m,nvx_r,nvy_r] = radio_sensor3d_m_4(xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,x_p_N,y_p_N,z_p_N,sig_x_r,sig_y_r,n,m,simul_nlos)
%radiosensor_err_factor = 1.0; R1m(i) = sqrt((xr1-x_p_N(i))^2+(yr1-y_p_N(i))^2+h^2)+nvx_r(i);
%sig_x_r=radiosensor_err_factor*0.1;              % radio sensor measurement noise in meter/sec
%sig_y_r=radiosensor_err_factor*0.2;              % radio sensor measurement noise in meter/sec in y-direction

nvx_r=normrnd(0,sig_x_r,1,n);
nvy_r=normrnd(0,sig_y_r,1,n);

% nvx_r = screwt(0.1,0.3,3,sig_x_r,sig_y_r,n);
% nvy_r = screwt(0.1,0.3,3,sig_x_r,sig_y_r,n);

R1m=zeros(m);
R2m=zeros(m);
R3m=zeros(m);
R4m=zeros(m);
for i=1:n
    
    if simul_nlos == 1
        nvx_tmp(i) = nvx_r(i);
        if i>1000 && i<1500
            nvx_tmp(i) = nvx_r(i)*10;
        end
        R1m(i) = sqrt((xr1-x_p_N(i))^2+(yr1-y_p_N(i))^2+(zr1-z_p_N(i))^2)+nvx_tmp(i);
    else
        R1m(i) = sqrt((xr1-x_p_N(i))^2+(yr1-y_p_N(i))^2+(zr1-z_p_N(i))^2)+nvx_r(i);
    end
    R2m(i) = sqrt((xr2-x_p_N(i))^2+(yr2-y_p_N(i))^2+(zr2-z_p_N(i))^2)+nvx_r(i);
    R3m(i) = sqrt((xr3-x_p_N(i))^2+(yr3-y_p_N(i))^2+(zr3-z_p_N(i))^2)+nvy_r(i);
    R4m(i) = sqrt((xr4-x_p_N(i))^2+(yr4-y_p_N(i))^2+(zr4-z_p_N(i))^2)+nvy_r(i);
end
end