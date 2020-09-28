function [x_p,x_v,x_a,y_p,y_v,y_a,z_p,z_v,z_a] = trajectory3d_circle(radius,wn,t,amp)

x_p = radius*cos(t);
x_v = - radius*wn*sin(t);
x_a = - radius*wn^2*cos(t);
y_p = radius*sin(t);
y_v = radius*wn*cos(t);
y_a = - radius*wn^2*sin(t);
z_p = radius*t/amp;
z_v = 2*radius;
z_a = 0*t;
end
