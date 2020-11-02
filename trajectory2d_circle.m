function [x_p,x_v,x_a,y_p,y_v,y_a,z_p,z_v,z_a] = trajectory2d_circle(radius,psi,wn)

x_p = radius*cos(psi);
x_v = - radius*wn*sin(psi);
x_a = - radius*wn^2*cos(psi);
y_p = radius*sin(psi);
y_v = radius*wn*cos(psi);
y_a = - radius*wn^2*sin(psi);
z_p = 0*sin(psi)+15;
z_v = 0*wn*cos(psi);
z_a = 0*wn^2*sin(psi);
end