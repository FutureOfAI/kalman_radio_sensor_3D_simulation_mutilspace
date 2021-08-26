function [x_p,x_v,y_p,y_v,z_p,z_v] = trajectory3d_line_vel(radius,wt,t0)

x_p = radius*sin(wt*t0)+ radius + 2.0;
x_v = radius*wt*cos(wt*t0);
y_p = radius*sin(wt*t0)+ radius + 2.0;
y_v = radius*wt*cos(wt*t0);
z_p = radius*sin(wt*t0)+ radius + 2.0;% in case under ground
z_v = radius*wt*cos(wt*t0);
end