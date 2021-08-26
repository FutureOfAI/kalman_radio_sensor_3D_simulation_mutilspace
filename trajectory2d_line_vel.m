function [x_p,x_v,y_p,y_v] = trajectory2d_line_vel(radius,apha_0,wt,t0,n)
x_p = radius*sin(wt*t0)*cos(apha_0)+radius+1.0;%radius*ones(1,n)
x_v = radius*wt*cos(wt*t0)*cos(apha_0);
y_p = radius*sin(wt*t0)*cos(apha_0)+radius+1.0;
y_v = radius*wt*cos(wt*t0)*cos(apha_0);
end