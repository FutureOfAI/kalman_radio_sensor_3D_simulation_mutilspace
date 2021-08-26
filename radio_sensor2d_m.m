function [R1m,R2m,R3m,nvx_r,nvy_r,nvz_r] = radio_sensor2d_m(xr1,yr1,xr2,yr2,xr3,yr3,x_p_N,y_p_N,sig_x_r,sig_y_r,sig_z_r,n,m)
%     nvx_r=normrnd(0,sig_x_r,1,n);
    nvy_r=normrnd(0,sig_y_r,1,n);
    nvz_r=normrnd(0,sig_z_r,1,n);
    nvx_r = screwt(0.1,3,3,sig_x_r,sig_y_r,n);
%     nvy_r = screwt(0.1,3,3,sig_x_r,sig_y_r,n);
%     nvz_r = screwt(0.1,3,3,sig_x_r,sig_y_r,n);
    
    R1m=zeros(m);
    R2m=zeros(m);
    R3m=zeros(m);

    for i=1:n
        % not Gussion noise
        nlos = 0;        
%         if i>0 && i<1600
%             if rand > 0.95
%                 nlos = 1.0*rand;
%             end
%             if rand > 0.85 && rand < 0.9
%                 nlos = 0.5*rand;
%             end
%             nvx_r(i) = nvx_r(i) + nlos;
%             nvy_r(i) = nvy_r(i) + nlos;
%             nvz_r(i) = nvz_r(i) + nlos;
%         end        
        
        R1m(i) = sqrt((xr1-x_p_N(i))^2+(yr1-y_p_N(i))^2)+nvx_r(i);
        R2m(i) = sqrt((xr2-x_p_N(i))^2+(yr2-y_p_N(i))^2)+nvy_r(i);
        R3m(i) = sqrt((xr3-x_p_N(i))^2+(yr3-y_p_N(i))^2)+nvz_r(i);
    end
end