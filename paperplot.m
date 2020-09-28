xxx1 = 400;
xxx2 = 2000;
% xxx1 = n1;
% xxx2 = n2;

% n2 = k-1;
%
% figure(1)
% plot(time,w_EB_B*r2d)
% xlabel('Time in seconds')
% ylabel('Rates in deg/sec')
% grid
%
% figure (2)
% subplot(311)
% plot(time,dx*r2d,'r')
% xlabel('Time in sec')
% ylabel('X angle error in deg')
% subplot(312)
% plot(time,dy*r2d,'g')
% xlabel('Time in sec')
% ylabel('Y angle error in deg')
% subplot(313)
% plot(time,dz*r2d,'b')
% xlabel('Time in sec')
% ylabel('Z angle error in deg')
% grid
%
figure (3)
subplot(311)
plot(time(xxx1:xxx2),dq11(xxx1:xxx2)*r2d)
xlabel('Time in sec')
ylabel('X angle error in deg','FontSize',8)
subplot(312)
plot(time(xxx1:xxx2),dq21(xxx1:xxx2)*r2d)
xlabel('Time in sec')
ylabel('Y angle error in deg','FontSize',8)
subplot(313)
plot(time(xxx1:xxx2),dq31(xxx1:xxx2)*r2d)
xlabel('Time in sec')
ylabel('Z angle error in deg','FontSize',8)
grid
%
figure (17)
plot(x_p_N,y_p_N,'b',xr1,yr1,'r*',xr2,yr2,'r*',xr3,yr3,'r*',xr4,yr4,'r*');
xlabel('X position in m')
ylabel('Y position in m')
axis([-30 30 -30 30])
figure (18)
plot(x_p_N,z_p_N,'b',xr1,zr1,'r*',xr2,zr2,'r*',xr3,zr3,'r*',xr4,zr4,'r*');
xlabel('X position in m')
ylabel('Z position in m')
% axis([-30 30 -30 30])
figure (16)
% subplot(311)
plot3(x_p_N,y_p_N,z_p_N,xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*')
% plot3(x_p_N,y_p_N,z_p_N)
% plot3(xpm_Nh(500:3991),ypm_Nh(500:3991),zpm_Nh(500:3991),'b-',xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*')
xlabel('X position in m')
ylabel('Y position in m')
grid
% axis([-20 20 -20 20 -20 20])
% grid
% subplot(312)
% plot(t00,x_v_N,'r.',t00,y_v_N,'g-',t00,z_v_N,'b')
% xlabel('Time in sec')
% ylabel('Velocity in m/s')
% grid
% subplot(313)
% plot(t0,x_a_N/g,'r.',t0,y_a_N/g,'g-',t0,z_a_N/g,'b')
% ylabel('X-Y-Z sensor accel in g in N-frame')
% xlabel('Time in sec')
% % axis([t0(1) t0(k-1) -1 1 -1 1])
% grid
%
% figure (8)
% subplot(311)
% plot(t00,acc(1,1:3991),'b')
% %xlabel('Time in sec')
% ylabel('X accel in g')
% subplot(312)
% plot(t00,acc(2,1:3991),'b')
% %xlabel('Time in sec')
% ylabel('Y accel in g')
% subplot(313)
% plot(t00,acc(3,1:3991),'b')
% %xlabel('Time in sec')
% ylabel('z linear accel in g')
%
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
figure (1)
subplot(311)
plot(t0(n1:n2),bx_h(n1:n2)/g,'g-')
ylabel('X Accel bias in g')
subplot(312)
plot(t0(n1:n2),by_h(n1:n2)/g,'g-')
ylabel('Y Accel bias in g')
subplot(313)
plot(t0(n1:n2),bz_h(n1:n2)/g,'g-')
ylabel('Z Accel bias in g')
%
figure(7);
subplot(3,1,1), plot(t0(xxx1:xxx2),(bx(xxx1:xxx2)-bx_h(xxx1:xxx2))/g);         
xlabel('Time in seconds');ylabel('X accel bias err in g','FontSize',8);
subplot(3,1,2), plot(t0(xxx1:xxx2),(by(xxx1:xxx2)-by_h(xxx1:xxx2))/g);         
xlabel('Time in seconds');ylabel('Y accel bias err in g','FontSize',8);
subplot(3,1,3), plot(t0(xxx1:xxx2),(bz(xxx1:xxx2)-bz_h(xxx1:xxx2))/g);
xlabel('Time in seconds');ylabel('Z accel bias err in g','FontSize',8);
%
figure (6)
subplot(3,1,1), plot(t0,(bx-bx_h)/g,'r');         
xlabel('Time in seconds');ylabel('Accel bias err in x-axis in g');
grid
subplot(3,1,2), plot(t0,(by-by_h)/g,'r');         
xlabel('Time in seconds');ylabel('Accel bias err in y-axis in g');
grid
subplot(3,1,3), plot(t0,(bz-bz_h)/g,'r');         
xlabel('Time in seconds');ylabel('Accel bias err in z-axis in g');
grid
%
