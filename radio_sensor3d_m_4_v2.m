function [R1m,R2m,R3m,R4m,nvx_r,nvy_r,nvp_r,nvq_r] = radio_sensor3d_m_4_v2(xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,x_p_N,y_p_N,z_p_N,sig_x_r,sig_y_r,sig_p_r,sig_q_r,n,m,simul_nlos)
%radiosensor_err_factor = 1.0; R1m(i) = sqrt((xr1-x_p_N(i))^2+(yr1-y_p_N(i))^2+h^2)+nvx_r(i);
%sig_x_r=radiosensor_err_factor*0.1;              % radio sensor measurement noise in meter/sec
%sig_y_r=radiosensor_err_factor*0.2;              % radio sensor measurement noise in meter/sec in y-direction
nvx_r=normrnd(0,sig_x_r,1,n);
nvy_r=normrnd(0,sig_y_r,1,n);
nvp_r=normrnd(0,sig_p_r,1,n);
nvq_r=normrnd(0,sig_q_r,1,n);
%     nvx_r = screwt(0.1,3,3,sig_x_r,sig_y_r,n);
%     nvy_r = screwt(0.1,3,3,sig_x_r,sig_y_r,n);
%     nvp_r = screwt(0.1,3,3,sig_x_r,sig_y_r,n);
%     nvq_r = screwt(0.1,3,3,sig_x_r,sig_y_r,n);


R1m=zeros(m);
R2m=zeros(m);
R3m=zeros(m);
R4m=zeros(m);
for i=1:n
    
    % not Gussion noise
    nlos = 0;        
%     if i>2500 && i<7500
%         if rand > 0.95
%             nlos = 1.0*rand;
%         end
%         if rand > 0.85 && rand < 0.9
%             nlos = 0.5*rand;
%         end
%         nvx_r(i) = nvx_r(i) + nlos;
%         nvy_r(i) = nvy_r(i) + nlos;
%         nvp_r(i) = nvp_r(i) + nlos;
%         nvq_r(i) = nvq_r(i) + nlos;
%     end   
    
    if simul_nlos == 1
        nvx_tmp(i) = nvx_r(i);
        if i>1000 && i<1500
            nvx_tmp(i) = nvx_r(i)*10;
        end
        R1m(i) = sqrt((xr1-x_p_N(i))^2+(yr1-y_p_N(i))^2+(zr1-z_p_N(i))^2)+nvx_tmp(i);
    else
        R1m(i) = sqrt((xr1-x_p_N(i))^2+(yr1-y_p_N(i))^2+(zr1-z_p_N(i))^2)+nvx_r(i);
    end   
    
    R2m(i) = sqrt((xr2-x_p_N(i))^2+(yr2-y_p_N(i))^2+(zr2-z_p_N(i))^2)+nvy_r(i);
    R3m(i) = sqrt((xr3-x_p_N(i))^2+(yr3-y_p_N(i))^2+(zr3-z_p_N(i))^2)+nvp_r(i);
    R4m(i) = sqrt((xr4-x_p_N(i))^2+(yr4-y_p_N(i))^2+(zr4-z_p_N(i))^2)+nvq_r(i);
end
end