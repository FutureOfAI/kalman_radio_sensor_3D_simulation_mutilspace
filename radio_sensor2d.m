function [R1m,R2m,R3m] = radio_sensor2d(xr1,yr1,xr2,yr2,xr3,yr3,x_p_N,y_p_N,n,m)
    R1m=zeros(m);
    R2m=zeros(m);
    R3m=zeros(m);
    for i=1:n
        R1m(i) = sqrt((xr1-x_p_N(i))^2+(yr1-y_p_N(i))^2);
        R2m(i) = sqrt((xr2-x_p_N(i))^2+(yr2-y_p_N(i))^2);
        R3m(i) = sqrt((xr3-x_p_N(i))^2+(yr3-y_p_N(i))^2);
    end
end