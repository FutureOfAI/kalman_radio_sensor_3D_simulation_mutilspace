function [R1m,R2m,R3m,nvx_r,nvy_r,nvz_r] = radio_sensor2d_m(xr1,yr1,xr2,yr2,xr3,yr3,x_p_N,y_p_N,sig_x_r,sig_y_r,sig_z_r,n,m)
    nvx_r=normrnd(0,sig_x_r,1,n);
    nvy_r=normrnd(0,sig_y_r,1,n);
    nvz_r=normrnd(0,sig_z_r,1,n);
    R1m=zeros(m);
    R2m=zeros(m);
    R3m=zeros(m);
    for i=1:n
        R1m(i) = sqrt((xr1-x_p_N(i))^2+(yr1-y_p_N(i))^2)+nvx_r(i);
        R2m(i) = sqrt((xr2-x_p_N(i))^2+(yr2-y_p_N(i))^2)+nvy_r(i);
        R3m(i) = sqrt((xr3-x_p_N(i))^2+(yr3-y_p_N(i))^2)+nvz_r(i);
    end
end