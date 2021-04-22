xxx1 = 1;
xxx2 = 2000;

figure (16)
% subplot(311)
plot3(x_p_N,y_p_N,z_p_N,xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*','linewidth',3)
% plot3(x_p_N,y_p_N,z_p_N)
% plot3(xpm_Nh(500:3991),ypm_Nh(500:3991),zpm_Nh(500:3991),'b-',xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*')
xlabel('X position in m','FontSize',18)
ylabel('Y position in m','FontSize',18)
zlabel('Z position in m','FontSize',18)
text(-30,0,500,'\fontsize{20}\color{black}Screw motion')
grid
figure (19)
plotcube([50 50 25],[-25 -25 0],.1,[0 0 1]);
hold on
plot3(xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*','linewidth',3)
xlabel('X position in m','FontSize',18)
ylabel('Y position in m','FontSize',18)
zlabel('Z position in m','FontSize',18)
text(-40,0,40,'\fontsize{20}\color{black}Anchors distribution')
% axis([-20 20 -20 20 -20 20])
% grid
%
figure (8)
plot3(xpm_Nh(n1:n2),ypm_Nh(n1:n2),zpm_Nh(n1:n2),'b.',xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*')
xlabel('X position in m','FontSize',14)
ylabel('Y position in m','FontSize',14)
% axis([-15 15 -15 15])
grid
figure (4)
subplot(311)
plot(t0,bx_h/g,'g-')
ylabel('X Accel bias in g')
subplot(312)
plot(t0,by_h/g,'g-')
ylabel('Y Accel bias in g')
subplot(313)
plot(t0,bz_h/g,'g-')
ylabel('Z Accel bias in g')
%
figure (5)
subplot(311)
% plot(t0(n1:n2),x_err(n1:n2))
plot(t0(xxx1:xxx2),x_err(xxx1:xxx2),'linewidth',2)
xlabel('Time in seconds','FontSize',14)
ylabel('X-Axis','FontSize',14)
title('Position err in meter','FontSize',18);
subplot(312)
% plot(t0(n1:n2),y_err(n1:n2))
plot(t0(xxx1:xxx2),y_err(xxx1:xxx2),'linewidth',2)
xlabel('Time in seconds','FontSize',14)
ylabel('Y-Axis','FontSize',14)
subplot(313)
% plot(t0(n1:n2),z_err(n1:n2))
plot(t0(xxx1:xxx2),z_err(xxx1:xxx2),'linewidth',2)
xlabel('Time in seconds','FontSize',14)
ylabel('Z-Axis','FontSize',14)
grid

