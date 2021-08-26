close all;
clear all;
clc;

% '1214v2.csv' ??light of sight
% 'obsall.csv' ???????
imu = load('output_IMU_static_v1.csv'); 
uwb = load('output_UWB_static_v1.csv');

t = 1:8000;
g = 9.8;
d2r = (pi/180);
acc = imu(:,1:3)'; % in g
gro = imu(:,4:6)'; % in rad/s
mag = imu(:,7:9)'; % in uT
post = uwb(:,7:10)'; % in m
rss = uwb(:,11:14)';

smoothHeadingDegrees = zeros(8000,1);
Mag = 45.466; 
mag_bias = [23.33 31.88 0]; % test by real sensor
% Set declination angle on your location and fix heading
% You can find your declination on: http://magnetic-declination.com/
% (+) Positive or (-) for negative
% For Taipei declination angle is -4'31w (positive)
% Formula: (deg + (min / 60.0)) / (180 / M_PI);
declinationAngle = (4.0 - (31.0 / 60.0)) / (180 / pi);

%% clear bad distance data
for j_1= 2:800
    
    if(post(1,j_1)>15)
        post(1,j_1) = post(1,j_1-1);
    end
    
    if(post(2,j_1)>15)
        post(2,j_1) = post(2,j_1-1);
    end
    
    if(post(3,j_1)>15)
        post(3,j_1) = post(3,j_1-1);
    end
    
    if(post(4,j_1)>15)
        post(4,j_1) = post(4,j_1-1);
    end
end

% anchor bias
% post(1,:) = post(1,:)+0.057;
% post(2,:) = post(2,:)+0.766;
% post(3,:) = post(3,:)+0.138;
% post(4,:) = post(4,:)+0.415;
% group 1
% post(1,:) = post(1,:)+0.5;
% post(2,:) = post(2,:)+0.3;
% post(3,:) = post(3,:)-0.3;
% post(4,:) = post(4,:)+0.5;
% group 2
% post(1,:) = post(1,:)+0.52;
% post(2,:) = post(2,:)+0.32;
% post(3,:) = post(3,:)-0.32;
% post(4,:) = post(4,:)+0.52;
% group rec
post(1,:) = post(1,:)+1.0;
post(2,:) = post(2,:)+0.4;
post(3,:) = post(3,:)+0.0;
post(4,:) = post(4,:)+0.0;
% post(1,:) = post(1,:)+1.22;
% post(2,:) = post(2,:)+0.86;
% post(3,:) = post(3,:)+0.24;
% post(4,:) = post(4,:)+0.62;
% group Experiment supplement
% post(1,:) = post(1,:)+0.5;
% post(2,:) = post(2,:)+0.24;
% post(3,:) = post(3,:)-0.1;
% post(4,:) = post(4,:)+0.36;
% anchor psi90deg
% post(1,:) = post(1,:)-0.185;
% post(2,:) = post(2,:)+0.525;
% post(3,:) = post(3,:)+0.145;
% post(4,:) = post(4,:)+0.175;

pos(:,1) = post(:,1);
for i=1:10:8000
   for j=1:10
      pos(:,i+j) = post(:,fix(i/10)+1); 
   end
end

for k = 1:8000          
%        mag(:,k) = (cz(-90*d2r)*cx(180*d2r)*mag(:,k));% rotation to north-west-up
%        mag(:,k) = (cx(180*d2r)*mag(:,k));% rotation to north-west-up  
        [smoothHeadingDegrees] = Fixed_Heading(mag,mag_bias,declinationAngle,k);
        mag_Z_heading(k) = -smoothHeadingDegrees(k)*(pi/180);
        % East-North-Up Navigation frame
        mag_LP(:,k) = [Mag*cos(mag_Z_heading(k)) Mag*sin(mag_Z_heading(k)) 0];
        mag_LP(:,k) = (cz(-90*d2r)*cx(180*d2r))*mag_LP(:,k);
end


an1_los = post(1,1:800);
an2_los = post(2,1:800);
an3_los = post(3,1:800);
an4_los = post(4,1:800);
%
figure (2)
subplot(411)
% plot(1:800,uwb(1:800,7))
plot(1:800,post(1,1:800))
ylabel('r_1 (m)','FontSize',12)
subplot(412)
% plot(1:800,uwb(1:800,8))
plot(1:800,post(2,1:800))
ylabel('r_2 (m)','FontSize',12)
subplot(413)
% plot(1:800,uwb(1:800,9))
plot(1:800,post(3,1:800))
ylabel('r_3 (m)','FontSize',12)
subplot(414)
% plot(1:800,uwb(1:800,10))
plot(1:800,post(4,1:800))
ylabel('r_4 (m)','FontSize',12)
xlabel('samples','FontSize',12)

figure (30)
subplot(411)
plot(1:800,rss(1,1:800))
ylabel('Anchor1 NLOS situation')
subplot(412)
plot(1:800,rss(2,1:800))
ylabel('Anchor2 NLOS situation')
subplot(413)
plot(1:800,rss(3,1:800))
ylabel('Anchor3 NLOS situation')
subplot(414)
plot(1:800,rss(4,1:800))
ylabel('Anchor4 NLOS situation')

%
figure (10)
title('Range measurements')
subplot(411)
plot(1:8000,pos(1,1:8000))
xlabel('time')
ylabel('Anchor1 distance (m)')
subplot(412)
plot(1:8000,pos(2,1:8000))
xlabel('time')
ylabel('Anchor2 distance (m)')
subplot(413)
plot(1:8000,pos(3,1:8000))
xlabel('time')
ylabel('Anchor3 distance (m)')
subplot(414)
plot(1:8000,pos(4,1:8000))
xlabel('time')
ylabel('Anchor4 distance (m)')
%
figure (11)
title('axm aym azm measurements')
subplot(311)
plot(t,acc(1,:),'b')
% axis([0 count/100 -0.1 0.1])
xlabel('Time(s)')
ylabel('axm in g')
subplot(312)
plot(t,acc(2,:),'b')
% axis([0 count/100 -0.1 0.1])
xlabel('Time(s)')
ylabel('aym in g')
subplot(313)
plot(t,acc(3,:),'b')
% axis([0 count/100 -10 0])
xlabel('Time(s)')
ylabel('azm in g')
%
figure (12)
title('wxm wym wzm measurements')
subplot(311)
plot(t,gro(1,:)*180/pi,'b')
% axis([0 count/100 -0.1 0.1])
xlabel('Time(s)')
ylabel('wxm in degree/s')
subplot(312)
plot(t,gro(2,:)*180/pi,'b')
% axis([0 count/100 -0.1 0.1])
xlabel('Time(s)')
ylabel('wym in degree/s')
subplot(313)
plot(t,gro(3,:)*180/pi,'b')
% axis([0 count/100 -10 0])
xlabel('Time(s)')
ylabel('wzm in degree/s')
%
figure (13)
title('magxm magym magzm measurements')
subplot(311)
plot(t,mag(1,:),'b',t,mag_LP(1,:),'r')
% axis([0 count/100 -0.1 0.1])
xlabel('Time(s)')
ylabel('mag xm in uT')
subplot(312)
plot(t,mag(2,:),'b',t,mag_LP(2,:),'r')
% axis([0 count/100 -0.1 0.1])
xlabel('Time(s)')
ylabel('mag ym in uT')
subplot(313)
plot(t,mag(3,:),'b',t,mag_LP(3,:),'r')
% axis([0 count/100 -10 0])
xlabel('Time(s)')
ylabel('mag zm in uT')


