n1 = 1;% experiment data @@@ %
n2 = 100;% experiment data @@@ %
n2 = k-1;% experiment data @@@ %
% n1 = 400;
% n2 = 100;
% n2 = k-1;
%
figure(1)
plot(time,w_EB_B*r2d)
xlabel('Time in seconds')
ylabel('Rates in deg/sec')
grid
%
figure (2)
plot (time,dq11*r2d,'b',time,dq21*r2d,'g',time,dq31*r2d,'r')
xlabel('Time in seconds')
ylabel('Angle errors in deg')
grid
%
figure (20)
plot (time,dx*r2d,'b',time,dy*r2d,'g',time,dz*r2d,'r')
xlabel('Time in seconds')
ylabel('Angle errors in deg')
grid
%
figure (3)
subplot(311)
% plot3(x_p_N,y_p_N,z_p_N,'r-',position(1,:),position(2,:),position(3,:),'b.',xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*')
plot3(x_p_N,y_p_N,z_p_N,'r-',xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*')
xlabel('X position in m')
ylabel('Y position in m')
axis([-50 50 -50 50 -50 50])
grid
subplot(312)
plot(t00,x_v_N,'r.',t00,y_v_N,'g-',t00,z_v_N,'b')
xlabel('Time in sec')
ylabel('Velocity in m/s')
grid
subplot(313)
plot(t0,x_a_N/g,'r.',t0,y_a_N/g,'g-',t0,z_a_N/g,'b')
ylabel('X-Y-Z sensor accel in g in N-frame')
xlabel('Time in sec')
% axis([t0(1) t0(k-1) -1 1 -1 1])
grid
%
figure (8)
subplot(311)
plot(t00,x_a_N/g,'r.',t00,x_a_B/g,'g-',t00,axm/g,'b')
%xlabel('Time in sec')
ylabel('X accel in g')
subplot(312)
plot(t00,y_a_N/g,'r.',t00,y_a_B/g,'g-',t00,aym/g,'b')
%xlabel('Time in sec')
ylabel('Y accel in g')
subplot(313)
plot(t00,z_a_N/g,'r.',t00,z_a_B/g,'g-',t00,azm/g,'b')
%xlabel('Time in sec')
ylabel('z linear accel in g')
%
figure (4)
subplot(311)
plot(t00,axm/g,'r.',t00,x_a_B/g,'g-')
%xlabel('Time in sec')
ylabel('X accel in g')
subplot(312)
plot(t00,aym/g,'r.',t00,y_a_B/g,'g-')
%xlabel('Time in sec')
ylabel('Y accel in g')
subplot(313)
plot(t00,azm/g,'r.',t00,z_a_B/g,'g-')
%xlabel('Time in sec')
ylabel('z accel in g')
%
figure(7);
subplot(3,1,1), plot(t0,bx/g,'r',t0,bx_h/g,'g');         
xlabel('Time in seconds');ylabel('Accel bias along x-axis in g');
subplot(3,1,2), plot(t0,by/g,'r',t0,by_h/g,'g');         
xlabel('Time in seconds');ylabel('Accel bias along y-axis in g');
subplot(3,1,3), plot(t0,bz/g,'r',t0,bz_h/g,'g');
xlabel('Time in seconds');ylabel('Accel bias along z-axis in g');
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

% experiment data @@@ %

figure (16)
% subplot(311)
subplot(511)
plot(t0(n1:2:n2),Mu__1(1,n1:n2/2),'b')
axis([0 n2/100 0 2])
ylabel('Not Anchor 1')
xlabel('Time in seconds')
subplot(512)
plot(t0(n1:2:n2),Mu__1(2,n1:n2/2),'b')
axis([0 n2/100 0 2])
ylabel('Not Anchor 2')
xlabel('Time in seconds')
subplot(513)
plot(t0(n1:2:n2),Mu__1(3,n1:n2/2),'b')
axis([0 n2/100 0 2])
ylabel('Not Anchor 3')
xlabel('Time in seconds')
subplot(514)
plot(t0(n1:2:n2),Mu__1(4,n1:n2/2),'b')
axis([0 n2/100 0 2])
ylabel('Not Anchor 4')
xlabel('Time in seconds')
subplot(515)
plot(t0(n1:2:n2),Mu__1(5,n1:n2/2),'b')
axis([0 n2/100 0 2])
ylabel('All Anchor')
xlabel('Time in seconds')
% %% not two anchor 
figure (17)
% subplot(311)
subplot(611)
plot(t0(n1:2:n2),Mu__2(1,n1:n2/2),'b')% not anchor1 anchor2
axis([0 n2/100 0 2])
ylabel('Not Anchor1 & 2')
xlabel('Time in seconds')
subplot(612)
plot(t0(n1:2:n2),Mu__2(2,n1:n2/2),'b')% not anchor1 anchor3
axis([0 n2/100 0 2])
ylabel('Not Anchor1 & 3')
xlabel('Time in seconds')
subplot(613)
plot(t0(n1:2:n2),Mu__2(3,n1:n2/2),'b')% not anchor1 anchor4
axis([0 n2/100 0 2])
ylabel('Not Anchor1 & 4')
xlabel('Time in seconds')
subplot(614)
plot(t0(n1:2:n2),Mu__2(4,n1:n2/2),'b')% not anchor2 anchor3
axis([0 n2/100 0 2])
ylabel('Not Anchor2 & 3')
xlabel('Time in seconds')
subplot(615)
plot(t0(n1:2:n2),Mu__2(5,n1:n2/2),'b')% not anchor2 anchor4
axis([0 n2/100 0 2])
ylabel('Not Anchor2 & 4')
xlabel('Time in seconds')
subplot(616)
plot(t0(n1:2:n2),Mu__2(6,n1:n2/2),'b')% not anchor3 anchor4
axis([0 n2/100 0 2])
ylabel('Not Anchor3 & 4')
xlabel('Time in seconds')
figure (18)
% subplot(311)
subplot(411)
plot(t0(n1:2:n2),Mu__3(1,n1:n2/2),'b')% not anchor1 anchor2
axis([0 n2/100 0 2])
ylabel('Not Anchor1 & 2 & 3')
xlabel('Time in seconds')
subplot(412)
plot(t0(n1:2:n2),Mu__3(2,n1:n2/2),'b')% not anchor1 anchor3
axis([0 n2/100 0 2])
ylabel('Not Anchor1 & 2 & 4')
xlabel('Time in seconds')
subplot(413)
plot(t0(n1:2:n2),Mu__3(3,n1:n2/2),'b')% not anchor1 anchor4
axis([0 n2/100 0 2])
ylabel('Not Anchor1 & 3 & 4')
xlabel('Time in seconds')
subplot(414)
plot(t0(n1:2:n2),Mu__3(4,n1:n2/2),'b')% not anchor2 anchor3
axis([0 n2/100 0 2])
ylabel('Not Anchor2 & 3 & 4')
xlabel('Time in seconds')
