function [x_p,x_v,x_a,y_p,y_v,y_a,z_p,z_v,z_a] = trajectory3d_line(radius,gama_0,apha_0,wt,t0)

% x_p = radius*sin(wt*t0)*cos(apha_0)*cos(gama_0);
% x_v = radius*wt*cos(wt*t0)*cos(apha_0)*cos(gama_0);
% x_a = - radius*wt^2*sin(wt*t0)*cos(apha_0)*cos(gama_0);
% y_p = radius*sin(wt*t0)*cos(apha_0)*sin(gama_0);
% y_v = radius*wt*cos(wt*t0)*cos(apha_0)*sin(gama_0);
% y_a = - radius*wt^2*sin(wt*t0)*cos(apha_0)*sin(gama_0);
% z_p = radius*sin(wt*t0)*sin(apha_0)+ radius;% in case under ground
% z_v = radius*wt*cos(wt*t0)*sin(apha_0);
% z_a = - radius*wt^2*sin(wt*t0)*sin(apha_0);
x_p = radius*sin(wt*t0)+ radius + 5;
x_v = radius*wt*cos(wt*t0);
x_a = - radius*wt^2*sin(wt*t0);
y_p = radius*sin(wt*t0)+ radius + 5;
y_v = radius*wt*cos(wt*t0);
y_a = - radius*wt^2*sin(wt*t0);
z_p = radius*sin(wt*t0)+ radius + 5;% in case under ground
z_v = radius*wt*cos(wt*t0);
z_a = - radius*wt^2*sin(wt*t0);
end