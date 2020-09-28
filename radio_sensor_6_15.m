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
        zr1 = 8;% in meter
        xr2 = 0;
        yr2 = -25;
        zr2 = 1;
        xr3 = 25;% in meter
        yr3 = 0;% in meter
        zr3 = 7;
        xr4 = -25;
        yr4 = 0;
        zr4 = 2;% in meter
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
        wz(1:n) = 10*wn;
        phi_N(1) = phi_0 + wx(1)*t0(1);
        theta_N(1) = theta_0 + wy(1)*t0(1);
        psi_N(1) = psi_0 + wz(1)*t0(1);
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

    for i = 2:n,
%        phi(i) = phi(i-1) + wxm(i-1)*dt;
%        theta(i) = theta(i-1) + wym(i-1)*dt;
%        psi(i) = psi(i-1) + wzm(i-1)*dt;
        
        
         phi_N(i) = phi_N(i-1) + wx(i-1)*dt;
         theta_N(i) = theta_N(i-1) + wy(i-1)*dt;
         psi_N(i) = psi_N(i-1) + wz(i-1)*dt;        
    end
% ====================================================
% Generate platform motion in N-frame
% ===================================================
    ft =0* 0.05;
    wt = 2*pi*ft;
    radius = 25;
    gama_0 = 1*45*d2r;
    apha_0 = 1*45*d2r;
    [x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N,z_p_N,z_v_N,z_a_N] = trajectory3d_line(radius,gama_0,apha_0,wt,t0);
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
QE_B = DTQ(CE_B(:,:,1));
QE_B_m = QE_B;
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
    QE_B = DTQ(CE_B(:,:,jj));
%     QE_B_save(:,jj) = QE_B;
    QB_E = -QE_B;
    QB_E(4,1) = QE_B(4,1);
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
    dQ1 = qmult(QE_B,inv_QE_B_m);
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
%stop
%
% ====================================================
% Need to convert into body frame (B-frame) for accelerometer sensings and
% magnetometer sensing since they are mounted on the body frame
% ====================================================
x_a_B = zeros(1,n);
y_a_B = zeros(1,n);
z_a_B = zeros(1,n);
%
x_m_B = zeros(1,n);
y_m_B = zeros(1,n);
z_m_B = zeros(1,n);
% ================================
for i = 1:n,
% rotation matrix N-frame to B-frame
    C11 = CE_B(1,1,i);
    C12 = CE_B(1,2,i);
    C13 = CE_B(1,3,i);
    C21 = CE_B(2,1,i);
    C22 = CE_B(2,2,i);
    C23 = CE_B(2,3,i);
    C31 = CE_B(3,1,i);
    C32 = CE_B(3,2,i);
    C33 = CE_B(3,3,i);
% Accelerometers
    x_a_B(i) = [C11 C12 C13]*[x_a_N(i) y_a_N(i) z_a_N(i) + g]';
    y_a_B(i) = [C21 C22 C23]*[x_a_N(i) y_a_N(i) z_a_N(i) + g]';
    z_a_B(i) = [C31 C32 C33]*[x_a_N(i) y_a_N(i) z_a_N(i) + g]';% linear accel in z-axis
% Magnetometers
    x_m_B(i) = [C11 C12 C13]*mag_E';
    y_m_B(i) = [C21 C22 C23]*mag_E';
    z_m_B(i) = [C31 C32 C33]*mag_E';
end
%=============================================================================================================
% Magnetometers (biases & noises)
%=====================================
mag_bx0=1*0.01*Mag;                              % initial magnetic bias in uT in x-direction
mag_by0=-1*0.01*Mag;                             % initial magnetic bias in uT in y-direction
mag_bz0=1*0.01*Mag;                              % initial magnetic bias in uT in z-direction
mag_err_flag = 1;                                % Flag to set up the magnetic errors
mag_sig_x_arw_0 = mag_err_flag*0.01*Mag;
mag_sig_y_arw_0 = mag_err_flag*0.01*Mag;
mag_sig_z_arw_0 = mag_err_flag*0.01*Mag;
% magnetometer bias stability
mag_sig_x_rrw_0 = mag_err_flag*0.01*Mag/3600;
mag_sig_y_rrw_0 = mag_err_flag*0.01*Mag/3600;
mag_sig_z_rrw_0 = mag_err_flag*0.01*Mag/3600;
%
mag_sig_x_arw = mag_sig_x_arw_0;
mag_sig_y_arw = mag_sig_y_arw_0;
mag_sig_z_arw = mag_sig_z_arw_0;
mag_sig_x_rrw = mag_sig_x_rrw_0;
mag_sig_y_rrw = mag_sig_y_rrw_0;
mag_sig_z_rrw = mag_sig_z_rrw_0;
% magnetometer biases
[mag_bx]=Biasba1(dt,n,m,1*mag_bx0,1*mag_sig_x_arw_0);
[mag_by]=Biasbp1(dt,n,m,1*mag_by0,1*mag_sig_y_arw_0);
[mag_bz]=Biasbp1(dt,n,m,1*mag_bz0,1*mag_sig_z_arw_0);
% nmagnetometer noises
[mag_xm,mag_ym,mag_zm]=transform3d_m(x_m_B,mag_bx,y_m_B,mag_by,z_m_B,mag_bz,m,n,1*mag_sig_x_rrw_0,1*mag_sig_y_rrw_0,1*mag_sig_z_rrw_0);

%=============================================================================================================
% accelerometer (biases and noises)
%=====================================
bx0=1*0.01*g;                              % initial accel bias in g in along-direction
by0=-1*0.01*g;                             % initial accel bias in g in perpenticular-direction
bz0=1*0.01*g;                              % initial accel bias in g in perpenticular-direction
err_factor = 1.0;
% accelerometer bias stability
sig_xr_0 = err_factor*0.01*g/3600;         % accel bias stability in g/sec-sqrt(sec) in along-direction
sig_yr_0 = err_factor*0.01*g/3600;         % accel bias stability in g/sec-sqrt(sec) in penpenticular-direction
sig_zr_0 = err_factor*0.01*g/3600;         % accel bias stability in g/sec-sqrt(sec) in penpenticular-direction

% accelerate (noise)
sig_bx_0 = err_factor*0.01*g;              % accel noise in g in along-direction
sig_by_0 = err_factor*0.01*g;              % accel noise in g in penpenticular-direction
sig_bz_0 = err_factor*0.01*g;              % accel noise in g in penpenticular-direction

% accelerometer biases
[bx]=Biasba1(dt,n,m,1*bx0,1*sig_xr_0);
[by]=Biasbp1(dt,n,m,1*by0,1*sig_yr_0);
[bz]=Biasbp1(dt,n,m,1*bz0,1*sig_zr_0);
% accelerometer noises
[axm,aym,azm]=transform3d_m(x_a_B,bx,y_a_B,by,z_a_B,bz,m,n,1*sig_bx_0,1*sig_by_0,1*sig_bz_0);
% =====================================================================================
% calculator "Q" for gyro random walk noises and rate stability noises
% =====================================================================================
sig_x_arw = sig_x_arw_0;                     
sig_y_arw = sig_y_arw_0;                     
sig_z_arw = sig_z_arw_0;                     
sig_x_rrw = sig_x_rrw_0;                 
sig_y_rrw = sig_y_rrw_0;              
sig_z_rrw = sig_z_rrw_0;
% =========================================================================
delta_t = dt;                                  % delta time for simulating the true dynamics = 0.01 sec
delta_s = 50*delta_t;                           % sampling at every 0.5 second for the Kalman filter
%================================================================
T1 = T;
%T1 = 0.2;
[sensor_step,propagation_step]=propagate_step(T1,delta_t,delta_s);
%================================================================
% Define the initial conditions for the three angle error in N-frame
%================================================================
dtheda_x = 0;
dtheda_y = 0;
dtheda_z = 0;
phierr = 1*0.5;       % x in deg
thetaerr = -1*0.5;     % y in deg
psierr = 10*0.5;       % z in deg
[dtheda_xh,dtheda_yh,dtheda_zh,bgx_h,bgy_h,bgz_h]=initial_estimate_value6_radio(m,dtheda_x,dtheda_y,dtheda_z,phierr,thetaerr,psierr,d2r);
dq11 = zeros(n,1);
dq21 = zeros(n,1);
dq31 = zeros(n,1);
dq11(1) = dtheda_x - dtheda_xh(1);
dq21(1) = dtheda_y - dtheda_yh(1);
dq31(1) = dtheda_z - dtheda_zh(1);
dx = zeros(n,1);
dy = zeros(n,1);
dz = zeros(n,1);
dx(1) = dtheda_xh(1);
dy(1) = dtheda_yh(1);
dz(1) = dtheda_zh(1);
%================================================================
% Define the initial conditions for the 6-states Kalman Filter
% ==============================================================
[s6_xz_h,s6_P00_z]=define_initial_condition_6(bgx0,bgy0,bgz0,phierr,thetaerr,psierr,d2r);
% ==============================================================
% for 6-states filter
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

s6_Q_z=zeros(6);
% ============================================================
% ============================================================
% Start the simulation run                                   |
% ============================================================
% ============================================================
% k=2;
for i=1:sensor_step
	for j=1:propagation_step
        k=1+j+(i-1)*propagation_step;
        
        bgx_h(k)=bgx_h(k-1);
        bgy_h(k)=bgy_h(k-1);
        bgz_h(k)=bgz_h(k-1);
% =========================================
% 6-state EKF
% Perform inertial attitude computations
% =========================================
        w_EB_B_xm(k) = w_EB_B_xm(k) - bgx_h(k);
        w_EB_B_ym(k) = w_EB_B_ym(k) - bgy_h(k);
        w_EB_B_zm(k) = w_EB_B_zm(k) - bgz_h(k);
        %w_EB_B_xm(k-1) = w_EB_B_xm(k-1) - bgx_h(k-1);
        %w_EB_B_ym(k-1) = w_EB_B_ym(k-1) - bgy_h(k-1);
        %w_EB_B_zm(k-1) = w_EB_B_zm(k-1) - bgz_h(k-1);
        [QE_B_m]=inertial_attitude_computation6_radio(w_EB_B_xm,w_EB_B_ym,w_EB_B_zm,bgx_h,bgy_h,bgz_h,QE_B_m,k,dt);
        DC_E_B_m(:,:,k) = q2dc(QE_B_m);
        delta_C = CE_B(:,:,k)'*DC_E_B_m(:,:,k);
        dx(k) = delta_C(3,2);
        dy(k) = delta_C(1,3);
        dz(k) = delta_C(2,1);
        %DC_E_B_m(:,:,k),
% =========================================
% Compute the attitude errors
    inv_QE_B_m = -QE_B_m;
    inv_QE_B_m(4,1) = QE_B_m(4,1);
    dQ1 = qmult(QE_B,inv_QE_B_m);
%    dQ1 = qmult(inv_QE_B_m,QE_B);
    dq11(k) = 2*dQ1(1,1);
    dq21(k) = 2*dQ1(2,1);
    dq31(k) = 2*dQ1(3,1);
% ===========================================================
% Perform Kalman filter propagation for 6-state Kalman filter
% ===========================================================
        %s6_F_z,
        [s6_phi_z,s6_Q_z,s6_F_z]=define_Dymamic_equation6_radio(s6_F_z,s6_Q_z,sig_x_arw,sig_y_arw,sig_z_arw,sig_x_rrw,sig_y_rrw,sig_z_rrw,DC_E_B_m,k,dt);
        [s6_xz_h,s6_P00_z]=Kalman_Filter_estimate6_radio(s6_xz_h,s6_phi_z,s6_P00_z,s6_Q_z,dt);
    end                 % end of filter propagation step 
% =======================================================
% Perform Kalman filter updates for 6-states filter
% =======================================================
    [s6_H,s6_R]=attitude_determination_6_1_EKF(x_a_N,y_a_N,z_a_N,sig_bx_0,sig_by_0,sig_bz_0,bx0,by0,bz0,mag_sig_x_arw,mag_sig_y_arw,mag_sig_z_arw,mag_bx0,mag_by0,mag_bz0,DC_E_B_m,mag_E,dt,k);
    %
    [s6_P00_z,s6_K_z,s6_z_update,s6_Mu_z]=Kalman_Filter_update_6_1_radio(s6_P00_z,axm,aym,azm,x_a_N,y_a_N,z_a_N,mag_xm,mag_ym,mag_zm,mag_E,DC_E_B_m,s6_H,s6_R,k);
    %
    [dtheda_xh,dtheda_yh,dtheda_zh,bgx_h,bgy_h,bgz_h]=upon_radiosensor_measurement_6_1(dtheda_xh,dtheda_yh,dtheda_zh,k,bgx_h,bgy_h,bgz_h,s6_z_update);
    w_EB_B_xm(k) = w_EB_B_xm(k) - bgx_h(k);
    w_EB_B_ym(k) = w_EB_B_ym(k) - bgy_h(k);
    w_EB_B_zm(k) = w_EB_B_zm(k) - bgz_h(k);
    s6_xz_h = zeros(6,1);
% ========================================================
% Update the attitude and the attidue errors
% ========================================================
dQ2(1,1) = dtheda_xh(k)/2;
dQ2(2,1) = dtheda_yh(k)/2;
dQ2(3,1) = dtheda_zh(k)/2;
dQ2(4,1) = sqrt(1 - dQ2(1,1)^2 - dQ2(2,1)^2 - dQ2(3,1)^2);
QE_B_m = qmult(dQ2,QE_B_m);
%QE_B_m = qmult(QE_B_m,dQ2);
DC_E_B_m(:,:,k) = q2dc(QE_B_m);
delta_C = CE_B(:,:,k)'*DC_E_B_m(:,:,k);
dx(k) = delta_C(3,2);
dy(k) = delta_C(1,3);
dz(k) = delta_C(2,1);
% ========================================================
% Update the attitude errors
    inv_QE_B_m = -QE_B_m;
    inv_QE_B_m(4,1) = QE_B_m(4,1);
    dQ1 = qmult(QE_B,inv_QE_B_m);
%    dQ1 = qmult(inv_QE_B_m,QE_B);
    dq11(k) = 2*dQ1(1,1);
    dq21(k) = 2*dQ1(2,1);
    dq31(k) = 2*dQ1(3,1);
% =======================================================
end                     % end of one Monte Caro run

%
n1 =(k-1)*0.1;
%n1 = 1;
n2 = k-1;
plot63;
%plot58;