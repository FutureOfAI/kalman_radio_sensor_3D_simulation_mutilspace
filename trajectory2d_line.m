function [x_p,y_p] = trajectory2d_line(radius,apha_0,wt,t0,n)
x_p = radius*sin(wt*t0)*cos(apha_0)+radius+1.0;%radius*ones(1,n)
y_p = radius*sin(wt*t0)*cos(apha_0)+radius+1.0;
end