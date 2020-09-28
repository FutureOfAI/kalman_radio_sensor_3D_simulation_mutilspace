 % %%
% close all;
% clear all;
% clc;
% load('3D_BOARD_TILTED.mat');
% load('3D_BOARD_HORILZON.mat');
% load('aj_static_16.mat');
load('multisapce_3D_data_180715.mat');
% load('straight line 40s.mat');
d2r = (pi/180);

count = 4000;
count_for = count-1;
data_new = zeros (32,count);
distance_cnt = 1;
inertial_cnt = 1;
t = 1:count;
acc = zeros(3,count);
gro = zeros(3,count);
mag = zeros(3,count);
pos = zeros(4,count);
smoothHeadingDegrees = zeros(count_for,1);
Mag = 45.466; 
mag_bias = [23.33 31.88 0]; % test by real sensor
% Set declination angle on your location and fix heading
% You can find your declination on: http://magnetic-declination.com/
% (+) Positive or (-) for negative
% For Taipei declination angle is -4'31w (positive)
% Formula: (deg + (min / 60.0)) / (180 / M_PI);
declinationAngle = (4.0 - (31.0 / 60.0)) / (180 / pi);

for j_1=1:count_for
    for i_1=1:32 
        %%
        if data(i_1,j_1) == 11
            if i_1+10 <= 32 && data(i_1+10,j_1) == 128
                if rem(sum( data(i_1:(i_1+8),j_1) ),256) == data(i_1+9,j_1)
                    data_new(1:11,distance_cnt) = data(i_1:(i_1+10),j_1);
                    distance_cnt = distance_cnt+1;
                end
            end
            if i_1+10 < 34&& i_1+10 > 32 && data(i_1-22,j_1+1) == 128
                if  rem(sum( data(i_1:i_1+8,j_1) ) ,256) == data(i_1+9,j_1)                    
                    data_new(1:(33-i_1),distance_cnt) = data(i_1:32,j_1);
                    data_new((34-i_1):11,distance_cnt) = data(1:(i_1-22),j_1+1);
                    distance_cnt = distance_cnt+1;
                end
            end
            if i_1+10 <= 34&& i_1+10 > 33 && data(i_1-22,j_1+1) == 128
                if  rem(sum( data(i_1:i_1+8,j_1) ) ,256) == data(i_1-23,j_1+1)                    
                    data_new(1:(33-i_1),distance_cnt) = data(i_1:32,j_1);
                    data_new((34-i_1):11,distance_cnt) = data(1:(i_1-22),j_1+1);
                    distance_cnt = distance_cnt+1;
                end
            end
            
            if i_1+10 > 34 && data(i_1-22,j_1+1) == 128
                if  rem(sum( data(i_1:32,j_1) ) + sum( data(1:(i_1-24),j_1+1) ),256) == data((i_1-23),j_1+1)
                    data_new(1:(33-i_1),distance_cnt) = data(i_1:32,j_1);
                    data_new((34-i_1):11,distance_cnt) = data(1:(i_1-22),j_1+1);
                    distance_cnt = distance_cnt+1;
                end                
            end
        end
        %%bitand(a, uint16(377));bitand(data(i+19,j),uint16(255))   bitand(data((i-11),j+1),uint16(255))
        if data(i_1,j_1) == 8
            if i_1+20 <= 32 && data(i_1+20,j_1) == 128
                if  rem(sum( data(i_1:(i_1+18),j_1) ),256) == data(i_1+19,j_1);
                    data_new(12:32,inertial_cnt) = data(i_1:(i_1+20),j_1);
                    inertial_cnt = inertial_cnt+1;
                end
            end
            if i_1+20 <= 33 && i_1+20 > 32 && data(i_1-12,j_1+1) == 128
                if  rem(sum( data(i_1:(i_1+18),j_1) ),256) == data(i_1+19,j_1);
                    data_new(12:(44-i_1),inertial_cnt) = data(i_1:32,j_1);
                    data_new((45-i_1):32,inertial_cnt) = data(1:(i_1-12),j_1+1);
                    inertial_cnt = inertial_cnt+1;
                end                
            end       
            
            if i_1+20 <= 34 && i_1+20 > 33 && data(i_1-12,j_1+1) == 128
                if  rem(sum( data(i_1:(i_1+18),j_1) ),256) == data(i_1+13,j_1+1);
                    data_new(12:(44-i_1),inertial_cnt) = data(i_1:32,j_1);
                    data_new((45-i_1):32,inertial_cnt) = data(1:(i_1-12),j_1+1);
                    inertial_cnt = inertial_cnt+1;
                end                
            end       
            if i_1+20 > 34 && data(i_1-12,j_1+1) == 128
                if rem((sum( data(i_1:32,j_1) ) + sum( data(1:(i_1-14),j_1+1))),256)== data((i_1-13),j_1+1)
                    data_new(12:(44-i_1),inertial_cnt) = data(i_1:32,j_1);
                    data_new((45-i_1):32,inertial_cnt) = data(1:(i_1-12),j_1+1);
                    inertial_cnt = inertial_cnt+1;
                end                
            end           
        end
%     if inertial_cnt+1 ~= distance_cnt
%       w=1;  
%     end
    end
end
for j_1=1:count_for
%     for i_1=1:30 
        gro(1,j_1) = (bitshift(data_new(13,j_1),8)+data_new(14,j_1));% gro x
        gro(2,j_1) = (bitshift(data_new(15,j_1),8)+data_new(16,j_1));% gro y
        gro(3,j_1) = (bitshift(data_new(17,j_1),8)+data_new(18,j_1));% gro z
        acc(1,j_1) = (bitshift(data_new(19,j_1),8)+data_new(20,j_1));% acc x
        acc(2,j_1) = (bitshift(data_new(21,j_1),8)+data_new(22,j_1));% acc y
        acc(3,j_1) = (bitshift(data_new(23,j_1),8)+data_new(24,j_1));% acc z
        mag(1,j_1) = (bitshift(data_new(25,j_1),8)+data_new(26,j_1));% mag x
        mag(2,j_1) = (bitshift(data_new(27,j_1),8)+data_new(28,j_1));% mag y
        mag(3,j_1) = (bitshift(data_new(29,j_1),8)+data_new(30,j_1));% mag z
        
        pos(1,j_1) = (bitshift(data_new(2,j_1),8)+data_new(3,j_1))/1024;
        pos(2,j_1) = (bitshift(data_new(4,j_1),8)+data_new(5,j_1))/1024;
        pos(3,j_1) = (bitshift(data_new(6,j_1),8)+data_new(7,j_1))/1024;
        pos(4,j_1) = (bitshift(data_new(8,j_1),8)+data_new(9,j_1))/1024;
%     end
end
%% clear bad distance data
for j_1= 2:1:count_for
    
    if(pos(1,j_1)>15)
        pos(1,j_1) = pos(1,j_1-1);
    end
    
    if(pos(2,j_1)>15)
        pos(2,j_1) = pos(2,j_1-1);
    end
    
    if(pos(3,j_1)>15)
        pos(3,j_1) = pos(3,j_1-1);
    end
    
    if(pos(4,j_1)>15)
        pos(4,j_1) = pos(4,j_1-1);
    end
end

for k = 1:count_for        
    for m = 1:3
        if acc(m,k) >= 2^15
            acc(m,k) = acc(m,k) - 2^16;
        end    
        if gro(m,k) >= 2^15
            gro(m,k) = gro(m,k) - 2^16;
        end    
        if mag(m,k) >= 2^15
            mag(m,k) = mag(m,k) - 2^16;
        end         
        acc(m,k) = acc(m,k)/8192/2; %I don't know why  ADC to real?      
        gro(m,k) = (gro(m,k)/131.072/2*(pi/180));    
        mag(m,k) = (mag(m,k)/6.83);        
%         flow(m,k) = flow(m,k)/bitshift(1,16); %I don't know why  ADC to real?
    end   
%        mag(:,k) = (cz(-90*d2r)*cx(180*d2r)*mag(:,k));% rotation to north-west-up
%        mag(:,k) = (cx(180*d2r)*mag(:,k));% rotation to north-west-up  
        [smoothHeadingDegrees] = Fixed_Heading(mag,mag_bias,declinationAngle,k);
        mag_Z_heading(k) = -smoothHeadingDegrees(k)*(pi/180);
        % East-North-Up Navigation frame
        mag_LP(:,k) = [Mag*cos(mag_Z_heading(k)) Mag*sin(mag_Z_heading(k)) 0];
        mag_LP(:,k) = (cz(-90*d2r)*cx(180*d2r))*mag_LP(:,k);
end

% Accelerometers
% for k = 1:count_for        
%     x_a_B(k) = [C11 C12 C13]*[x_a_N(k) y_a_N(k) z_a_N(k) + g]';
%     y_a_B(k) = [C21 C22 C23]*[x_a_N(k) y_a_N(k) z_a_N(k) + g]';
%     z_a_B(k) = [C31 C32 C33]*[x_a_N(k) y_a_N(k) z_a_N(k) + g]';% linear accel in z-axis
% end
    
% order = 1;
% filtCutOff = 0.1*2;
% samplePeriod = 1/100;
% [b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'low');
% mag_LP(1,:) = filtfilt(b, a, mag(1,:));  
% mag_LP(2,:) = filtfilt(b, a, mag(2,:));
% mag_LP(3,:) = filtfilt(b, a, mag(3,:));

%
figure (10)
title('Range measurements')
subplot(411)
plot(1:count_for,pos(1,1:count_for))
xlabel('time')
ylabel('Anchor1 distance (m)')
subplot(412)
plot(1:count_for,pos(2,1:count_for))
xlabel('time')
ylabel('Anchor2 distance (m)')
subplot(413)
plot(1:count_for,pos(3,1:count_for))
xlabel('time')
ylabel('Anchor3 distance (m)')
subplot(414)
plot(1:count_for,pos(4,1:count_for))
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
plot(t(1,1:count_for),mag(1,1:count_for),'b',t(1,1:count_for),mag_LP(1,1:count_for),'r')
% axis([0 count/100 -0.1 0.1])
xlabel('Time(s)')
ylabel('mag xm in uT')
subplot(312)
plot(t(1,1:count_for),mag(2,1:count_for),'b',t(1,1:count_for),mag_LP(2,1:count_for),'r')
% axis([0 count/100 -0.1 0.1])
xlabel('Time(s)')
ylabel('mag ym in uT')
subplot(313)
plot(t(1,1:count_for),mag(3,1:count_for),'b',t(1,1:count_for),mag_LP(3,1:count_for),'r')
% axis([0 count/100 -10 0])
xlabel('Time(s)')
ylabel('mag zm in uT')
