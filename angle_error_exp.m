% Program name: radio_sensor_6_15.m
% 6-state Kalman filter only using accelerometers and magnetometers to calibrate
% the gyro biases and correct the platform attitude errors
% 6/15/2017
% Edited by Andy Wu
close all;
clear all;
clc;
format short e
% define some constants
r2d = (180/pi);
d2r = (pi/180);
g = 9.8;
% ---------------------------------
% Earth magnetic field values in E-frame  |
% ---------------------------------
Mag = 45;           % uT (Micro Tesla) F component
Angle_I = -1*35*d2r;  % inclination angle - 0 deg inclination angle for equator location
%Angle_I = -90*d2r;  % 90 inclination angle foe north pole location
Angle_D = -1*4*d2r;   % declination angle
mag_E = [Mag*cos(Angle_I)*cos(Angle_D) Mag*cos(Angle_I)*sin(Angle_D) Mag*sin(Angle_I)];
% ---------------------------------
r1 = [1 0 0];
r2 = [0 1 0];
r3 = [0 0 1];
%
dt = 0.01;          % 100 Hz sampling rate
t=0;
%======================================================
% Design parameters to be tuned
%=====================================================
% Select motion profiles
% trajectory is generated in navigation frame (N-frame)
%================================
    % Set motion profile flags
    % profile_flag = 1:         straight motion
    % profile_flag = 2;         circular motion
    % profile_flag = 3;         race track motion
    % =====================================================
    % Straight motion with phi, theta and psi in constant angle, or p = q = r = 0
    % =====================================================
        xr1 = 0;% in meter
        yr1 = 25;% in meter
        zr1 = 30;% in meter
        xr2 = 0;
        yr2 = -25;
        zr2 = 30;
        xr3 = 25;% in meter
        yr3 = 0;% in meter
        zr3 = 30;
        xr4 = -25;
        yr4 = 0;
        zr4 = 30;% in meter
        %
  % ================================
        fn = 1*0.05;
        T = 6*(1/fn);
 %       T = 0.2;
  % ===============================
        t0 = 0:dt:T;
        t0_1 = t0';
        n = length(t0);
        m = size(t0_1);
        t00 = t0;
        %
        phi_0 = 0*45*d2r;%x
        theta_0 = 0*45*d2r;%y
        psi_0 = 0*45*d2r;%z
   %     wn = 2*pi*fn*d2r;
        wn = 2*pi*fn;% rad
        wx(1:n) = 0*wn;
        wy(1:n) = 0*wn;
        wz(1:n) = 1*wn;
        phi_N(1) = phi_0 + wx(1)*t0(1);
        theta_N(1) = theta_0 + wy(1)*t0(1);
        psi_N(1) = psi_0 + wz(1)*t0(1);
    % =================================================================
    % add gyro bias and noise
    % psi theta phi not constant
    % gyroscope
    % =================================================================
    gyro_err_flag = 1;                        % Flag to set up the gyro errors
    gyro_bias_flag = 0;
    bgx0=gyro_bias_flag*0.5*d2r;                            % initial gyro bias in rad/sec
    bgy0=gyro_bias_flag*(-0.5)*d2r;                            % initial gyro bias in rad/sec
    bgz0=gyro_bias_flag*0.5*d2r;                            % initial gyro bias in rad/sec
    %=============================================================================================================
    % gyroscope(bias & noise)
    %=====================================
    sig_x_arw_0 = gyro_err_flag*0.02;                       % gyro angle random walk = 0.02 deg/sqrt(sec)
    sig_y_arw_0 = gyro_err_flag*0.02;                       % gyro angle random walk = 0.02 deg/sqrt(sec)
    sig_z_arw_0 = gyro_err_flag*0.02;                       % gyro angle random walk = 0.02 deg/sqrt(sec)
    sig_x_rrw_0 = gyro_err_flag*0.02/3600;                    % gyro rate random walk = 0.02 deg/3600/sec-sqrt(sec);
    sig_y_rrw_0 = gyro_err_flag*0.02/3600;                    % gyro rate random walk = 0.02 deg/3600/sec-sqrt(sec);
    sig_z_rrw_0 = gyro_err_flag*0.02/3600;                    % gyro rate random walk = 0.02 deg/3600/sec-sqrt(sec);
% calculator "s6_Q" use bias & noise
%     sig_x_arw = sig_x_arw_0;                     
%     sig_y_arw = sig_y_arw_0;                     
%     sig_z_arw = sig_z_arw_0;                     
%     sig_x_rrw = sig_x_rrw_0;                 
%     sig_y_rrw = sig_y_rrw_0;              
%     sig_z_rrw = sig_z_rrw_0;            

%    [wbx]=Biasbg1(dt,n,m,bx0,d2r,sig_x_rrw_0);
%    [wby]=Biasbg1(dt,n,m,by0,d2r,sig_y_rrw_0);
%    [wbz]=Biasbg1(dt,n,m,bz0,d2r,sig_z_rrw_0);
%    [wxm]=Wzm1(wx,wbx,m,n,d2r,sig_x_arw_0);
%    [wym]=Wzm1(wy,wby,m,n,d2r,sig_y_arw_0);
%    [wzm]=Wzm1(wz,wbz,m,n,d2r,sig_z_arw_0);
    mag_angle = 1;
    for i = 2:n,
        
%        phi(i) = phi(i-1) + wxm(i-1)*dt;
%        theta(i) = theta(i-1) + wym(i-1)*dt;
%        psi(i) = psi(i-1) + wzm(i-1)*dt;
        
        
         phi_N(i) = phi_N(i-1) + wx(i-1)*dt;
         theta_N(i) = theta_N(i-1) + wy(i-1)*dt;
%          psi_N(i) = psi_N(i-1) + wz(i-1)*dt;
         psi_N(i) = mag_angle*sin(2*pi*fn*(i-1)*dt);
         wz(i) = 2*pi*fn*mag_angle*cos(2*pi*fn*(i-1)*dt);
    end
% ====================================================
% Generate platform motion in N-frame
% ===================================================
    ft =0* 0.05;
    wt = 2*pi*ft;
    radius = 25;
    gama_0 = 1*45*d2r;
    apha_0 = 1*45*d2r;
%     [x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N,z_p_N,z_v_N,z_a_N] = trajectory3d_line(radius,gama_0,apha_0,wt,t0);
    [x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N,z_p_N,z_v_N,z_a_N] = trajectory3d_circle(radius,wn,10*psi_N,mag_angle);
    
    figure (16)
    % subplot(311)
    plot3(x_p_N,y_p_N,z_p_N,xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*')
    % plot3(x_p_N,y_p_N,z_p_N)
    % plot3(xpm_Nh(500:3991),ypm_Nh(500:3991),zpm_Nh(500:3991),'b-',xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*')
    xlabel('X position in m')
    ylabel('Y position in m') 
% ====================================================
% quaternion_propagation.m
% get direction cosine matrix
% ====================================================
% =========================================================================
% Generate w_EB_B
% =========================================================================
for n1 = 1:n,
    time(n1) = t;
% generate inertial rates
% =========================================================
    T_1 = r1'*r1 + inv(cx(phi_N(n1)))*(r2'*r2) + inv(cx(phi_N(n1)))*inv(cy(theta_N(n1)))*(r3'*r3);
    w_EB_B(1:3,n1) = T_1*[wx(n1) wy(n1) wz(n1)]';
% =========================================================
    t = t + dt;
end
% ============================================
% Add gyro noises to w_EB_B
% ============================================
    [wbxm]=Biasbg1(dt,n,m,bgx0,d2r,sig_x_rrw_0);
    [wbym]=Biasbg1(dt,n,m,bgy0,d2r,sig_y_rrw_0);
    [wbzm]=Biasbg1(dt,n,m,bgz0,d2r,sig_z_rrw_0);
    [w_EB_B_xm]=Wzm1(w_EB_B(1,:),wbxm,m,n,d2r,sig_x_arw_0);
    [w_EB_B_ym]=Wzm1(w_EB_B(2,:),wbym,m,n,d2r,sig_y_arw_0);
    [w_EB_B_zm]=Wzm1(w_EB_B(3,:),wbzm,m,n,d2r,sig_z_arw_0);

% =========================================================================
% Quaternion propagation
% =========================================================================
dq1 = zeros(n,1);
dq2 = zeros(n,1);
dq3 = zeros(n,1);
CE_B(:,:,1) = cz(psi_N(1))*cy(theta_N(1))*cx(phi_N(1));
CB_E(:,:,1) = CE_B(:,:,1)';
QE_B(:,1) = DTQ(CE_B(:,:,1));
QE_B_m = QE_B(:,1);
QB_E_m = -QE_B_m;
QB_E_m(4,1) = QE_B_m(4,1);
%DC_E_B_m(:,:,1) = [1 0 0;0 1 0; 0 0 1];% Direction Cosine Matrix
DC_E_B_m(:,:,1) = CE_B(:,:,1);
DC_B_E_m(:,:,1) = CE_B(:,:,1)';
dQ = [0 0 0 1]';
dq1(1) = 2*dQ(1,1);
dq2(1) = 2*dQ(2,1);
dq3(1) = 2*dQ(3,1);
for jj = 2:n,
%    CE_B = cz(psi(jj))*cy(theta(jj))*cx(phi(jj));
    CE_B(:,:,jj) = cz(psi_N(jj))*cy(theta_N(jj))*cx(phi_N(jj));
%     CE_B_save(:,:,jj) = CE_B;
    QE_B(:,jj) = DTQ(CE_B(:,:,jj));
%     QE_B_save(:,jj) = QE_B;
    QB_E = -QE_B(:,jj);
    QB_E(4,1) = QE_B(4,jj);
    Qtmp = [QB_E(4) QB_E(1) QB_E(2) QB_E(3)];
    euler(:,jj) = quatern2euler(Qtmp);
% ============================================
% quaternion propagation algorithm
% ============================================
    d1 = w_EB_B_xm(jj)*dt/2;
    d2 = w_EB_B_ym(jj)*dt/2;
    d3 = w_EB_B_zm(jj)*dt/2;
    d1p = w_EB_B_xm(jj-1)*dt/2;
    d2p = w_EB_B_ym(jj-1)*dt/2;
    d3p = w_EB_B_zm(jj-1)*dt/2;
    d0_s = d1^2 + d2^2 + d3^2;
    q1 = d1 - (d0_s*d1 + d3p*d2 - d2p*d3)/6;
    q2 = d2 - (d0_s*d2 + d1p*d3 - d3p*d1)/6;
    q3 = d3 - (d0_s*d3 + d2p*d1 - d1p*d2)/6;
    q4 = 1 - d0_s/2;
    delta_Q = [q1 q2 q3 q4]';
    QE_B_m = qmult(delta_Q,QE_B_m); 
    DC_E_B_m(:,:,jj) = q2dc(QE_B_m);% calculate DCM measurement
    DC_B_E_m(:,:,jj) = DC_E_B_m(:,:,jj)';
% =============================================
% check the angle errors
% =============================================
    inv_QE_B_m = -QE_B_m;
    inv_QE_B_m(4,1) = QE_B_m(4,1);
    Qtmp = [inv_QE_B_m(4) inv_QE_B_m(1) inv_QE_B_m(2) inv_QE_B_m(3)];
    eulerm(:,jj) = quatern2euler(Qtmp);
    dQ1 = qmult(QE_B(:,jj),inv_QE_B_m);
    dq1(jj) = 2*dQ1(1,1);
    dq2(jj) = 2*dQ1(2,1);
    dq3(jj) = 2*dQ1(3,1);
end
%
figure (11)
plot(time,dq1*r2d,'b',time,dq2*r2d,'r',time,dq3*r2d,'g')
xlabel('Time in seconds')
ylabel('Attitude errors in deg')
grid
%

% deuler = euler-eulerm;
% deuler(find(deuler>pi)) = deuler(find(deuler>pi)-1);
% deuler(find(deuler<-pi)) = deuler(find(deuler<-pi)-1);
% 
% figure (12)
% plot(time,deuler(1,:)*r2d,'b',time,deuler(2,:)*r2d,'r',time,deuler(3,:)*r2d,'g')
% xlabel('Time in seconds')
% ylabel('Attitude errors in deg')
% grid

