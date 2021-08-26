function [R1m,R2m,R3m,R4m] = radio_sensor3d_4_v2(xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,x_p_N,y_p_N,z_p_N,n,m)
    R1m=zeros(m);
    R2m=zeros(m);
    R3m=zeros(m);
    R4m=zeros(m);
    for i=1:n
        R1m(i) = sqrt((xr1-x_p_N(i))^2+(yr1-y_p_N(i))^2+(zr1-z_p_N(i))^2);
        R2m(i) = sqrt((xr2-x_p_N(i))^2+(yr2-y_p_N(i))^2+(zr2-z_p_N(i))^2);
        R3m(i) = sqrt((xr3-x_p_N(i))^2+(yr3-y_p_N(i))^2+(zr3-z_p_N(i))^2);
        R4m(i) = sqrt((xr4-x_p_N(i))^2+(yr4-y_p_N(i))^2+(zr4-z_p_N(i))^2);
    end
end