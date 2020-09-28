% Program name: radio_sensor_08_04.m
% 9-state Kalman filter with 4 radio sensors for 6DOF motion tracking
% simulation with 6-state EKF with accel and magnetometers as measurements
% ????uwb NLOS???????
% 9/10/2020
close all;
clear all;
clc;
format short e

% define some constants
r2d = (180/pi);
d2r = (pi/180);
g = 9.8;
acc_g = [0 0 g];
r1 = [1 0 0];
r2 = [0 1 0];
r3 = [0 0 1];
%
dt = 0.1;
t=0;

simul_nlos = 0; % ??NLOS????

% ==================================================================
% Earth magnetic field values in E-frame  |
% ==================================================================
Mag = 45;           % uT (Micro Tesla) F component
Angle_I = 0*35*d2r;  % inclination angle - 0 deg inclination angle for equator location
%Angle_I = -90*d2r;  % 90 inclination angle foe north pole location
Angle_D = -0*4*d2r;   % declination angle
mag_E_0 = [Mag*cos(Angle_I)*cos(Angle_D) Mag*cos(Angle_I)*sin(Angle_D) Mag*sin(Angle_I)];
% Need to convert into North-West-Up Navigation frame and consistent with
% the positive psi angle rotation
mag_E = (cz(-90*d2r)*cx(180*d2r)*mag_E_0')';
%mag_E = (cx(180*d2r)*mag_E_0')';
% =====================================================
% Set up for running Monte Caro runs
% ======================================================
% Define four UWB sensors locations
% =====================================================
        xr1 = 0;% in meter
        yr1 = 25;% in meter
        zr1 = 30;% in meter
        xr2 = 0;
        yr2 = -25;
        zr2 = 30;
        xr3 = 25;% in meter
        yr3 = 0;% in meter
        zr3 = 0;
        xr4 = -25;
        yr4 = 0;
        zr4 = 0;% in meter
% ================================
% Set the simulation run time
% ================================
 % ================================================
        fn = 1*0.005;
        fz = 1*0.005;
        T = 4*(1/fz);
        TT1 = T;
        TT2 = 0.5*T;
        TT3 = 0.75*T;
        gama_0 = 1*45*d2r;
        apha_0 = 1*45*d2r;
        ft =1*0.01;
        wt = 2*pi*ft;
        radius = 1*25;
  %      T = 1;
        delta_t = dt;                                  % delta time for simulating the true dynamics = 0.01 sec
        delta_s = 5*delta_t;                           % sampling at every 0.5 second for the Kalman filter
        err_flag = 1;                                   % flag for turning on the magnetometer and accelerometer errors
        err_flag_g = 1;                                 % flag for turning in the gyro noises
  % ================================================
        t0 = 0:dt:T;
        t0_1 = t0';
        n = length(t0);
        m = size(t0_1);
        t00 = t0;
  % ================================================
        phi_0 = 0*1.5*d2r;%x
        theta_0 = -0*1.5*d2r;%y
        psi_0 = 0*4.5*d2r;%z
   %     wn = 2*pi*fn*d2r;
        wn = 2*pi*fn;% rad
        wx(1:n) = 0*wn;
        wy(1:n) = 0*wn;
        wz(1:n) = 1*wn;
   % ==================================================================
   % Create other angular motion (such as sinusoidal motion) to try 
   % to improve the bias estimation performance
   % ==================================================================
   %     phi_N(1) = phi_0 + wx(1)*t0(1);
        phi_N(1) = phi_0;
   %     theta_N(1) = theta_0 + wy(1)*t0(1);
        theta_N(1) = theta_0;
   %     psi_N(1) = psi_0 + wz(1)*t0(1);
        psi_N(1) = psi_0;
% =========================================================================
% Note that the angles phi_N, theta_N, psi_N are the angles from B-frame to
% N-frame or E-frame
% =========================================================================

%=====================================================
% Select motion profiles
% trajectory is generated in navigation frame (N-frame)
%================================
    % Set motion profile flags
%     profile_flag = 1;        % straight motion
    profile_flag = 2;         % screw motion
    % profile_flag = 3;         race track motion
    %
if (profile_flag ==1),
    mag_angle = 1.000*d2r;
    for i = 2:n,
%        phi(i) = phi(i-1) + wxm(i-1)*dt;
%        theta(i) = theta(i-1) + wym(i-1)*dt;
%        psi(i) = psi(i-1) + wzm(i-1)*dt;
        
        
         phi_N(i) = phi_N(i-1) + wx(i-1)*dt;
%         phi_N(i) = phi_0*cos(2*pi*fn*(i-1)*dt);
%         wx(i) = -2*pi*fn*phi_0*sin(2*pi*0.1*fn*(i-1)*dt);
         theta_N(i) = theta_N(i-1) + wy(i-1)*dt;
%         psi_N(i) = psi_N(i-1) + wz(i-1)*dt;
         psi_N(i) = mag_angle*sin(2*pi*fz*(i-1)*dt);
         wz(i) = 2*pi*fz*mag_angle*cos(2*pi*fz*(i-1)*dt);
    end
%    wx(1) = 0;
    wz(1) = 2*pi*fz*mag_angle;
% ====================================================
% Generate platform motion in N-frame
% ===================================================
%    [x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N,z_p_N,z_v_N,lina_z_a_N] = trajectory3d_line(radius,gama_0,apha_0,wt,t0);
    [x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N,z_p_N,z_v_N,z_a_N] = trajectory3d_line(radius,gama_0,apha_0,wt,t0);
end

if (profile_flag == 2),
    mag_angle = 1; % 4*pi
    for i = 2:n,
%        phi(i) = phi(i-1) + wxm(i-1)*dt;
%        theta(i) = theta(i-1) + wym(i-1)*dt;
%        psi(i) = psi(i-1) + wzm(i-1)*dt;
        
        
         phi_N(i) = phi_N(i-1) + wx(i-1)*dt;
%         phi_N(i) = phi_0*cos(2*pi*fn*(i-1)*dt);
%         wx(i) = -2*pi*fn*phi_0*sin(2*pi*0.1*fn*(i-1)*dt);
         theta_N(i) = theta_N(i-1) + wy(i-1)*dt;
%         psi_N(i) = psi_N(i-1) + wz(i-1)*dt;
         psi_N(i) = mag_angle*sin(2*pi*fz*(i-1)*dt);
         wz(i) = 2*pi*fz*mag_angle*cos(2*pi*fz*(i-1)*dt);
    end
%    wx(1) = 0;
    wz(1) = 2*pi*fz*mag_angle;
% ====================================================
% Generate platform motion in N-frame
% ===================================================
    [x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N,z_p_N,z_v_N,z_a_N] = trajectory3d_circle(radius,wn,10*psi_N,mag_angle);
end

% figure (16)
% % subplot(311)
% plot3(x_p_N,y_p_N,z_p_N,xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*')
% % plot3(x_p_N,y_p_N,z_p_N)
% % plot3(xpm_Nh(500:3991),ypm_Nh(500:3991),zpm_Nh(500:3991),'b-',xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*')
% xlabel('X position in m')
% ylabel('Y position in m')

    % =================================================================
    % add gyro bias and noise
    % psi theta phi not constant
    % gyroscope
    % =================================================================
    gyro_err_flag = 1;                        % Flag to set up the gyro errors
    gyro_bias_flag = 1;
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
    [wbxm]=Biasbg1(dt,n,m,err_flag_g*bgx0,d2r,err_flag_g*sig_x_rrw_0);
    [wbym]=Biasbg1(dt,n,m,err_flag_g*bgy0,d2r,err_flag_g*sig_y_rrw_0);
    [wbzm]=Biasbg1(dt,n,m,err_flag_g*bgz0,d2r,err_flag_g*sig_z_rrw_0);
    [w_EB_B_xm]=Wzm1(w_EB_B(1,:),wbxm,m,n,d2r,err_flag_g*sig_x_arw_0);
    [w_EB_B_ym]=Wzm1(w_EB_B(2,:),wbym,m,n,d2r,err_flag_g*sig_y_arw_0);
    [w_EB_B_zm]=Wzm1(w_EB_B(3,:),wbzm,m,n,d2r,err_flag_g*sig_z_arw_0);
% =========================================================================
% =========================================================================
% Quaternion propagation
% =========================================================================
dq1 = zeros(n,1);
dq2 = zeros(n,1);
dq3 = zeros(n,1);
% Generate the true quaternion
% The following direction matrix is CB_E not CE_B
%CB_E(:,:,1) = cz(psi_N(1))*cy(theta_N(1))*cx(phi_N(1));
%CE_B(:,:,1) = CB_E(:,:,1)';
CE_B(:,:,1) = cz(psi_N(1))*cy(theta_N(1))*cx(phi_N(1));
CB_E(:,:,1) = CE_B(:,:,1)';
Q_E_B(:,1) = DTQ(CE_B(:,:,1));
%
QE_B_m = Q_E_B(:,1);
Q_E_B_m(:,1) = QE_B_m;
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
    CE_B(:,:,jj) = cz(psi_N(jj))*cy(theta_N(jj))*cx(phi_N(jj));
    CB_E(:,:,jj) = CE_B(:,:,jj)';
% Generate the true quaternion
    Q_E_B(:,jj) = DTQ(CE_B(:,:,jj));
%     QE_B_save(:,jj) = QE_B;
    QB_E = -Q_E_B(:,jj);
    QB_E(4,1) = Q_E_B(4,jj);
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
    Q_E_B_m(:,jj) = QE_B_m;
    DC_E_B_m(:,:,jj) = q2dc(QE_B_m);% calculate DCM measurement
    DC_B_E_m(:,:,jj) = DC_E_B_m(:,:,jj)';
% =============================================
% check the angle errors
% =============================================
    inv_QE_B_m = -QE_B_m;
    inv_QE_B_m(4,1) = QE_B_m(4,1);
    Qtmp = [inv_QE_B_m(4) inv_QE_B_m(1) inv_QE_B_m(2) inv_QE_B_m(3)];
    eulerm(:,jj) = quatern2euler(Qtmp);
    dQ1 = qmult(Q_E_B(:,jj),inv_QE_B_m);
    dq1(jj) = 2*dQ1(1,1);
    dq2(jj) = 2*dQ1(2,1);
    dq3(jj) = 2*dQ1(3,1);
end

figure (11)
subplot(311)
plot(time,dq1*r2d,'b')
subplot(312)
plot(time,dq2*r2d,'r')
subplot(313)
plot(time,dq3*r2d,'g')
xlabel('Time in seconds')
ylabel('Attitude errors in deg')
grid
%
%stop
%
% ====================================================
% Need to convert into body frame (B-frame) for accelerometer sensings and
% magnetic sensing since they are mounted on the body frame
% ====================================================
x_m_B = zeros(1,n);
y_m_B = zeros(1,n);
z_m_B = zeros(1,n);
%
x_a_B = zeros(1,n);
y_a_B = zeros(1,n);
z_a_B = zeros(1,n);
% ================================
for i = 1:n,
    C11 = CE_B(1,1,i);
    C12 = CE_B(1,2,i);
    C13 = CE_B(1,3,i);
    C21 = CE_B(2,1,i);
    C22 = CE_B(2,2,i);
    C23 = CE_B(2,3,i);
    C31 = CE_B(3,1,i);
    C32 = CE_B(3,2,i);
    C33 = CE_B(3,3,i);
% accelerometers
    x_a_B(i) = [C11 C12 C13]*[x_a_N(i) y_a_N(i) z_a_N(i) + g]';
    y_a_B(i) = [C21 C22 C23]*[x_a_N(i) y_a_N(i) z_a_N(i) + g]';
    z_a_B(i) = [C31 C32 C33]*[x_a_N(i) y_a_N(i) z_a_N(i) + g]';% linear accel in z-axis
% Magnetometers
    x_m_B(i) = [C11 C12 C13]*mag_E';
    y_m_B(i) = [C21 C22 C23]*mag_E';
    z_m_B(i) = [C31 C32 C33]*mag_E';
end
% =====================================
% Magnetometers (biases & noises)
% =====================================
mag_bx0=1*0.01*Mag;                              % initial magnetic bias in uT in x-direction
mag_by0=-1*0.01*Mag;                             % initial magnetic bias in uT in y-direction
mag_bz0=1*0.01*Mag;                              % initial magnetic bias in uT in z-direction

mag_sig_x_arw_0 = 0.01*Mag;
mag_sig_y_arw_0 = 0.01*Mag;
mag_sig_z_arw_0 = 0.01*Mag;
% magnetometer bias stability
mag_sig_x_rrw_0 = 0.01*Mag/3600;
mag_sig_y_rrw_0 = 0.01*Mag/3600;
mag_sig_z_rrw_0 = 0.01*Mag/3600;
%
mag_sig_x_arw = mag_sig_x_arw_0;
mag_sig_y_arw = mag_sig_y_arw_0;
mag_sig_z_arw = mag_sig_z_arw_0;
mag_sig_x_rrw = mag_sig_x_rrw_0;
mag_sig_y_rrw = mag_sig_y_rrw_0;
mag_sig_z_rrw = mag_sig_z_rrw_0;
% magnetometer biases
[mag_bx]=Biasba1(dt,n,m,err_flag*mag_bx0,err_flag*mag_sig_x_arw_0);
[mag_by]=Biasbp1(dt,n,m,err_flag*mag_by0,err_flag*mag_sig_y_arw_0);
[mag_bz]=Biasbp1(dt,n,m,err_flag*mag_bz0,err_flag*mag_sig_z_arw_0);
% nmagnetometer noises
[mag_xm,mag_ym,mag_zm]=transform3d_m(x_m_B,mag_bx,y_m_B,mag_by,z_m_B,mag_bz,m,n,err_flag*mag_sig_x_rrw_0,err_flag*mag_sig_y_rrw_0,err_flag*mag_sig_z_rrw_0);
% ========================================================
% Define inertial sensor parameters "accelerate mpu9150"
% ========================================================
% accelerometer (biases and noises)
%=========================================================
bx0=1*0.1*g;                              % initial accel bias in g in along-direction
by0=-1*0.1*g;                             % initial accel bias in g in perpenticular-direction
bz0=1*0.1*g;                              % initial accel bias in g in perpenticular-direction
err_factor = 1;
%
sig_xr_0 = err_factor*0.1*g/3600;         % accel bias stability in g/sec-sqrt(sec) in along-direction
sig_yr_0 = err_factor*0.1*g/3600;         % accel bias stability in g/sec-sqrt(sec) in penpenticular-direction
sig_zr_0 = err_factor*0.1*g/3600;         % accel bias stability in g/sec-sqrt(sec) in penpenticular-direction

% accelerate (noise)
sig_bx_0 = err_factor*0.01*g;              % accel noise in g in along-direction
sig_by_0 = err_factor*0.01*g;              % accel noise in g in penpenticular-direction
sig_bz_0 = err_factor*0.01*g;              % accel noise in g in penpenticular-direction

% accelerometer biases
[bx]=Biasba1(dt,n,m,err_flag*bx0,err_flag*sig_xr_0);
[by]=Biasbp1(dt,n,m,err_flag*by0,err_flag*sig_yr_0);
[bz]=Biasbp1(dt,n,m,err_flag*bz0,err_flag*sig_zr_0);
% accelerometer noises
[axm,aym,azm]=transform3d_m(x_a_B,bx,y_a_B,by,z_a_B,bz,m,n,err_flag*sig_bx_0,err_flag*sig_by_0,err_flag*sig_bz_0);
% =====================================================================================
% calculator "Q" used for gyro random walk noises and rate stability noises
% =====================================================================================
sig_x_arw = sig_x_arw_0;                     
sig_y_arw = sig_y_arw_0;                     
sig_z_arw = sig_z_arw_0;                     
sig_x_rrw = sig_x_rrw_0;                 
sig_y_rrw = sig_y_rrw_0;              
sig_z_rrw = sig_z_rrw_0;
% =====================================
% calculator "Q" used for accel/magnet bias & noise
% =====================================
sig_bx=sig_bx_0;%(noise)
sig_by=sig_by_0;%(noise)
sig_bz=sig_bz_0;%(noise)
sig_xr=sig_xr_0;%(bias)
sig_yr=sig_yr_0;%(bias)
sig_zr=sig_zr_0;%(bias)
% =====================================================================================
% Define ""4 radio sensor"" parameters (noise)
% =====================================================================================
radiosensor_err_factor = 1;
sig_x_r=radiosensor_err_factor*0.1;              % radio sensor measurement noise in meters x-direction
sig_y_r=radiosensor_err_factor*0.1;              % radio sensor measurement noise in meters y-direction
%======================================================================================
[R1m,R2m,R3m,R4m,nvx_r,nvy_r] = radio_sensor3d_m_4(xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,x_p_N,y_p_N,z_p_N,sig_x_r,sig_y_r,n,m,simul_nlos);%4 four radio sensors
%================================================================
[sensor_step,propagation_step]=propagate_step(T,delta_t,delta_s);
%================================================================
% Define the initial conditions for the inertial sensors and the navigation
% states
%===============================================================
% Introduce initial position and velocity estimate error in N-frame
xverr = 0.1;        % in meters/sec
yverr = -0.1;       % in meters/sec
zverr = 0.1;        % in meters/sec
xperr = 0.5;        % in m
yperr = -0.5;       % in m
zperr = 0.5;        % in m
xaerr = 0;          % in m/sec^2
yaerr = 0;          % in m/sec^2
zaerr = 0;          % in m/sec^2
[xpm_Nh,ypm_Nh,zpm_Nh,xvm_Nh,yvm_Nh,zvm_Nh,xam_Nh,yam_Nh,zam_Nh,axm_h,aym_h,azm_h,bx_h,by_h,bz_h]=initial_estimate_value9_radio(m,x_a_N,y_a_N,z_a_N,x_v_N,y_v_N,z_v_N,x_p_N,y_p_N,z_p_N,axm,aym,azm,xverr,yverr,zverr,xperr,yperr,zperr,xaerr,yaerr,zaerr);
x_err(1) = x_p_N(1) - xpm_Nh(1);
y_err(1) = y_p_N(1) - ypm_Nh(1);
z_err(1) = z_p_N(1) - zpm_Nh(1);
%================================================================
% Define the initial conditions for the 9-states Kalman Filter
% ==============================================================
% It is noted that we increase (100 times) the initial covariance matrix values
% in P00_z(7,7), P00_z(8,8),P00_z(9,9) to make accel bias errors converge faster
% ==============================================================
[xz_h,P00_z]=define_initial_condition_9(bx0,by0,bz0,xperr,yperr,zperr,xverr,yverr,zverr);
% ==============================================================
% for 9-states filter
F_z=zeros(9);
%F_z = [0 0 0  1 0 0  0 0 0
%       0 0 0  0 1 0  0 0 0
%       0 0 0  0 0 1  0 0 0

%       0 0 0  0 0 0  -C11 -C12 -C13
%       0 0 0  0 0 0  -C21 -C22 -C23
%       0 0 0  0 0 0  -C31 -C32 -C33

%       0 0 0  0 0 0  0 0 0
%       0 0 0  0 0 0  0 0 0
%       0 0 0  0 0 0  0 0 0];
F_z(1,4) = 1;
F_z(2,5) = 1;
F_z(3,6) = 1;

F_z(4,7) = -DC_E_B_m(1,1,1);F_z(4,8) = -DC_E_B_m(2,1,1);F_z(4,9) = -DC_E_B_m(3,1,1);%_m
F_z(5,7) = -DC_E_B_m(1,2,1);F_z(5,8) = -DC_E_B_m(2,2,1);F_z(5,9) = -DC_E_B_m(3,2,1);
F_z(6,7) = -DC_E_B_m(1,3,1);F_z(6,8) = -DC_E_B_m(2,3,1);F_z(6,9) = -DC_E_B_m(3,3,1);

Q_z=zeros(9);
%
%================================================================
% Define the initial conditions for the three angle error in N-frame
%================================================================
dtheda_x = 0*phi_0;
dtheda_y = 0*theta_0;
dtheda_z = 0*psi_0;
phierr = 1*1.5;       % x in deg
thetaerr = -1*1.5;     % y in deg
psierr = 1*1.5;       % z in deg
[dtheda_xh,dtheda_yh,dtheda_zh,bgx_h,bgy_h,bgz_h]=initial_estimate_value6_radio(m,dtheda_x,dtheda_y,dtheda_z,phierr,thetaerr,psierr,d2r);
dq11 = zeros(n,1);
dq21 = zeros(n,1);
dq31 = zeros(n,1);
dq11(1) = (dtheda_x - dtheda_xh(1))/2;
dq21(1) = (dtheda_y - dtheda_yh(1))/2;
dq31(1) = (dtheda_z - dtheda_zh(1))/2;
dx = zeros(n,1);
dy = zeros(n,1);
dz = zeros(n,1);
dx(1) = dtheda_xh(1);
dy(1) = dtheda_yh(1);
dz(1) = dtheda_zh(1);
% ============================================================
% Define the initial conditions for the 6-state EKF
% ============================================================
[s6_xz_h,s6_P00_z]=define_initial_condition_6(bgx0,bgy0,bgz0,phierr,thetaerr,psierr,d2r);
%
s6_F_z=zeros(6);
%s6_F_z = [0 0 0  -C11 -C21 -C31
%          0 0 0  -C12 -C22 -C32
%          0 0 0  -C13 -C23 -C33

%          0 0 0  0 0 0
%          0 0 0  0 0 0
%          0 0 0  0 0 0];
%DC_E_B_m(:,:,1),
s6_F_z(1,4) = -DC_E_B_m(1,1,1);s6_F_z(1,5) = -DC_E_B_m(2,1,1);s6_F_z(1,6) = -DC_E_B_m(3,1,1);%_m
s6_F_z(2,4) = -DC_E_B_m(1,2,1);s6_F_z(2,5) = -DC_E_B_m(2,2,1);s6_F_z(2,6) = -DC_E_B_m(3,2,1);
s6_F_z(3,4) = -DC_E_B_m(1,3,1);s6_F_z(3,5) = -DC_E_B_m(2,3,1);s6_F_z(3,6) = -DC_E_B_m(3,3,1);
% Define the measurement equation matrix H = s6_H and the equivalent measurement noise
% covariance matrix R = s6_R from the TRIAD algorithm
s6_H = zeros(3,6);
s6_H(1,1) = 1;
s6_H(2,2) = 1;
s6_H(3,3) = 1;

s6_R = 350*[(1.5*d2r)^2 0 0                     % TRIAD attitude determination error in x - axis
    0 (1.5*d2r)^2 0                           % TRIAD attitude determination error in y - axis
    0 0 (1.5*d2r)^2];                         % TRIAD attitude determination error in z - axis
% Define the process noise matrix Q = s6_Q_z associated with the gyro errors
s6_Q_z=zeros(6);
s6_Q_z0(1:3,1:3) = [sig_x_arw^2 0 0 
                    0 sig_y_arw^2 0 
                    0 0 sig_z_arw^2]; 
s6_Q_z(1:3,1:3) = 0.21*s6_Q_z0;
s6_Q_z(4,4) = 1*sig_x_rrw^2;
s6_Q_z(5,5) = 1*sig_y_rrw^2;
s6_Q_z(6,6) = 1*sig_z_rrw^2;
% ============================================================
% ============================================================
% Start the simulation run                                   |
% ============================================================
% ============================================================
% Need to re-define the initial valus for Q_E_B_m;
% ============================================================
dQerr = [dq11(1) dq21(1) dq31(1) sqrt(1 - dq11(1)^2 - dq21(1)^2 - dq31(1)^2)]';
QE_B_m = qmult(dQerr,Q_E_B(:,1));
Q_E_B_e = zeros(4,n);
k1 = 1;
for i=1:sensor_step
	for j=1:propagation_step
        k=1+j+(i-1)*propagation_step;
   % accel biases     
        bx_h(k)=bx_h(k-1);
        by_h(k)=by_h(k-1);
        bz_h(k)=bz_h(k-1);
   % gyro biases     
        bgx_h(k)=bgx_h(k-1);
        bgy_h(k)=bgy_h(k-1);
        bgz_h(k)=bgz_h(k-1);
% ===========================================================
% Perform inertial attitude computations
% =========================================
        w_EB_B_xm(k) = w_EB_B_xm(k) - bgx_h(k);
        w_EB_B_ym(k) = w_EB_B_ym(k) - bgy_h(k);
        w_EB_B_zm(k) = w_EB_B_zm(k) - bgz_h(k);
        % ===========================================
        % quaternion propagation algorithm
        % ============================================
        d1 = w_EB_B_xm(k)*dt/2;
        d2 = w_EB_B_ym(k)*dt/2;
        d3 = w_EB_B_zm(k)*dt/2;
        d1p = w_EB_B_xm(k-1)*dt/2;
        d2p = w_EB_B_ym(k-1)*dt/2;
        d3p = w_EB_B_zm(k-1)*dt/2;
        d0_s = d1^2 + d2^2 + d3^2;
        q1 = d1 - (d0_s*d1 + d3p*d2 - d2p*d3)/6;
        q2 = d2 - (d0_s*d2 + d1p*d3 - d3p*d1)/6;
        q3 = d3 - (d0_s*d3 + d2p*d1 - d1p*d2)/6;
        q4 = 1 - d0_s/2;
        delta_Q = [q1 q2 q3 q4]';
        QE_B_m = qmult(delta_Q,QE_B_m);
        %
        Q_E_B_m(:,k) = QE_B_m;
        DC_E_B_m(:,:,k) = q2dc(QE_B_m);
        delta_C = DC_E_B_m(:,:,k)'*CE_B(:,:,k);
        dx(k) = delta_C(3,2);
        dy(k) = delta_C(1,3);
        dz(k) = delta_C(2,1);
        %DC_E_B_m(:,:,k),
    % =========================================
    % Compute the attitude errors
        inv_QE_B_m = - QE_B_m;
        inv_QE_B_m(4,1) = QE_B_m(4,1);
        dQ1 = qmult(Q_E_B(:,k),inv_QE_B_m);
    %    dQ1 = qmult(inv_QE_B_m,QE_B);
        dq11(k) = 2*dQ1(1,1);
        dq21(k) = 2*dQ1(2,1);
        dq31(k) = 2*dQ1(3,1);
% =========================================
% Perform inertial navigation computations
% =========================================
        [xpm_Nh,ypm_Nh,zpm_Nh,xvm_Nh,yvm_Nh,zvm_Nh,xam_Nh,yam_Nh,zam_Nh,axm_h,aym_h,azm_h]=inertial_navigation_computation9_radio(xvm_Nh,yvm_Nh,zvm_Nh,xpm_Nh,ypm_Nh,zpm_Nh,xam_Nh,yam_Nh,zam_Nh,axm,aym,azm,bx_h,by_h,bz_h,DC_E_B_m,k,dt);
        x_err(k) = x_p_N(k) - xpm_Nh(k);
        y_err(k) = y_p_N(k) - ypm_Nh(k);
        z_err(k) = z_p_N(k) - zpm_Nh(k);
% ===========================================================
% Perform Kalman filter propagation for 9-state Kalman filter
% ===========================================================
        [phi_z,Q_z,F_z]=define_Dymamic_equation9_radio(F_z,Q_z,sig_bx,sig_by,sig_bz,sig_xr,sig_yr,sig_zr,DC_E_B_m,k,dt,simul_nlos);
        [xz_h,P00_z]=Kalman_Filter_estimate1_radio(xz_h,phi_z,P00_z,Q_z,dt);
% ===========================================================
% Perform Kalman filter propagation for 6-state Kalman filter
% ===========================================================
        s6_F_z(1,4) = -DC_E_B_m(1,1,k-1);
        s6_F_z(1,5) = -DC_E_B_m(2,1,k-1);
        s6_F_z(1,6) = -DC_E_B_m(3,1,k-1);
        s6_F_z(2,4) = -DC_E_B_m(1,2,k-1);
        s6_F_z(2,5) = -DC_E_B_m(2,2,k-1);
        s6_F_z(2,6) = -DC_E_B_m(3,2,k-1);
        s6_F_z(3,4) = -DC_E_B_m(1,3,k-1);
        s6_F_z(3,5) = -DC_E_B_m(2,3,k-1);
        s6_F_z(3,6) = -DC_E_B_m(3,3,k-1);

        s6_phi_z = expm(s6_F_z*dt);
        %
        %[s6_xz_h,s6_P00_z]=Kalman_Filter_estimate6_radio(s6_xz_h,s6_phi_z,s6_P00_z,s6_Q_z,dt);
        s6_xz_h=s6_phi_z*s6_xz_h;
        s6_P00_z=s6_phi_z*s6_P00_z*(s6_phi_z') + s6_Q_z*dt;
% ========================================================
    end                 % end of filter propagation step
% ========================================================
% Perform Kalman filter updates for 9-states filter
% =======================================================
    [H,R,R1m_h,R2m_h,R3m_h,R4m_h]=radio_discrete_5_3_EKF(xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,xpm_Nh,ypm_Nh,zpm_Nh,sig_x_r,sig_y_r,k,simul_nlos);
%   
    [P00_z,K_z,z_update,Mu_z]=Kalman_Filter_update_5_3_radio(P00_z,R1m,R2m,R3m,R4m,H,R,R1m_h,R2m_h,R3m_h,R4m_h,k);
%    
    [xpm_Nh,ypm_Nh,zpm_Nh,xvm_Nh,yvm_Nh,zvm_Nh,bx_h,by_h,bz_h]=upon_radiosensor_measurement_5_3(xpm_Nh,ypm_Nh,zpm_Nh,xvm_Nh,yvm_Nh,zvm_Nh,k,bx_h,by_h,bz_h,z_update);
    % Update the current accel measurements
    axm_h(k)=axm(k)-bx_h(k);
    aym_h(k)=aym(k)-by_h(k);
    azm_h(k)=azm(k)-bz_h(k);
    %
    x_err(k) = x_p_N(k) - xpm_Nh(k);
    y_err(k) = y_p_N(k) - ypm_Nh(k);
    z_err(k) = z_p_N(k) - zpm_Nh(k);
% reset the errors after the filter updates
    xz_h=zeros(9,1);
% ========================================================
% =========================================================
% Upon the measurements from accel and magnetometers
% ==========================================================
% Set up the time to turn on the 6-state EKF updates
% ==========================================================
k1 = k1 + 1;
    if ((time(k) <= TT1)||((time(k)<=TT2)&&(time(k)>=TT3))),
    if (time(k) >= 3.0),
% =======================================================
% Perform Kalman filter updates for 6-states filter
% =======================================================
    [C_E_B_e] = TRIAD(axm_h,aym_h,azm_h,x_a_N,y_a_N,z_a_N,mag_xm,mag_ym,mag_zm,mag_E,k);
%    [C_E_B_e] = TRIAD(axm_h,aym_h,azm_h,xam_Nh,yam_Nh,zam_Nh,mag_xm,mag_ym,mag_zm,mag_E,k);    
    Q_E_B_e(:,k) = DTQ(C_E_B_e);
    psi_Nh =  atan2(mag_xm(k),mag_ym(k));
    psi_err(k1-1) = psi_N(k) - psi_Nh;
    %Q_E_B_e,
    [s6_P00_z,s6_K_z,s6_z_update,s6_Mu_z,dQ,dQ11]=Kalman_Filter_update_7_8_radio(s6_P00_z,Q_E_B_e,QE_B_m,s6_H,s6_R,Q_E_B,k);
%    Q_E_B_e,
%    [time(k) k],
%    [dQ dQ11],
%
    [dtheda_xh,dtheda_yh,dtheda_zh,bgx_h,bgy_h,bgz_h]=upon_radiosensor_measurement_6_1(dtheda_xh,dtheda_yh,dtheda_zh,k,bgx_h,bgy_h,bgz_h,s6_z_update);
% Update the current gyro measurements
    w_EB_B_xm(k) = w_EB_B_xm(k) - bgx_h(k);
    w_EB_B_ym(k) = w_EB_B_ym(k) - bgy_h(k);
    w_EB_B_zm(k) = w_EB_B_zm(k) - bgz_h(k);
    
    s6_xz_h = zeros(6,1);
% ========================================================
% Update the current attitude and the attidue errors
% ========================================================
    dQ2(1,1) = dtheda_xh(k)/2;
    dQ2(2,1) = dtheda_yh(k)/2;
    dQ2(3,1) = dtheda_zh(k)/2;
    dQ2(4,1) = sqrt(1 - dQ2(1,1)^2 - dQ2(2,1)^2 - dQ2(3,1)^2);
    QE_B_m = qmult(dQ2,QE_B_m);
    %QE_B_m = qmult(QE_B_m,dQ2);
    %
    Q_E_B_m(:,k) = QE_B_m;
    DC_E_B_m(:,:,k) = q2dc(QE_B_m);
    delta_C = CE_B(:,:,k)'*DC_E_B_m(:,:,k);
    dx(k) = delta_C(3,2);
    dy(k) = delta_C(1,3);
    dz(k) = delta_C(2,1);
% ========================================================
% Update the attitude errors
    inv_QE_B_m = -QE_B_m;
    inv_QE_B_m(4,1) = QE_B_m(4,1);
    dQ1 = qmult(Q_E_B(:,k),inv_QE_B_m);
%    dQ1 = qmult(inv_QE_B_m,QE_B);
    dq11(k) = 2*dQ1(1,1);
    dq21(k) = 2*dQ1(2,1);
    dq31(k) = 2*dQ1(3,1);       % end of filter updates
% =======================================================
    else
    end
    else
    end
end                     % end of one Monte Caro run

%
n1 = 0.5*(k-1);
n2 = k-1;

[(bgx0-bgx_h(n2))*r2d (bgy0-bgy_h(n2))*r2d (bgz0-bgz_h(n2))*r2d],
[mean(x_err(n1:n2)) mean(y_err(n1:n2)) mean(z_err(n1:n2))],
[std(x_err(n1:n2)) std(y_err(n1:n2)) std(z_err(n1:n2))],
paperplot;
plot58;