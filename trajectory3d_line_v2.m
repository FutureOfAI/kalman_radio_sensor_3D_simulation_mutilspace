function [x_p,y_p,z_p] = trajectory3d_line_v2(radius,wt,t0)

x_p = radius*sin(wt*t0)+ radius + 5;
y_p = radius*sin(wt*t0)+ radius + 5;
z_p = radius*sin(wt*t0)+ radius + 5;% in case under ground
end