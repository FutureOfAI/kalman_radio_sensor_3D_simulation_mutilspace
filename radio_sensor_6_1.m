% Program name: radio_sensor_6_1.m
% 15-state Kalman filter with 4 radio sensors for6DOF motion tracking
% simulation
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
% Earth magnetic field parameter  |
% ---------------------------------
Mag = 45;           % uT F component
Angle_I = -35*d2r;  % inclination angle
Angle_D = -4*d2r;   % declination angle
mag_N = [Mag*cos(Angle_I)*cos(Angle_D) Mag*cos(Angle_I)*sin(Angle_D) Mag*sin(Angle_I)];
% ---------------------------------
r1 = [1 0 0];
r2 = [0 1 0];
r3 = [0 0 1];
%
dt = 0.01;
t=0;
%

for ii = 1:1,
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
    profile_flag = 1;
    %
    if (profile_flag ==1),
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
        T = 20;
        T = 20;
        t0 = 0:dt:T;
        t0_1 = t0';
        n = length(t0);
        m = size(t0_1);
        t00 = t0;
        %
        fn = 1*0.05;
        phi_0 = 1*45*d2r;%x
        theta_0 = 1*45*d2r;%y
        psi_0 = 1*45*d2r;%z
        wn = 2*pi*fn*d2r;
        wx(1:n) = wn;
        wy(1:n) = wn;
        wz(1:n) = wn;
        phi_N(1) = phi_0 + wx(1)*t0(1);
        theta_N(1) = theta_0 + wy(1)*t0(1);
        psi_N(1) = psi_0 + wz(1)*t0(1);
        
        gama_0 = 1*45*d2r;
        apha_0 = 1*45*d2r;
        
    end
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
    sig_x_arw = sig_x_arw_0;                     
    sig_y_arw = sig_y_arw_0;                     
    sig_z_arw = sig_z_arw_0;                     
    sig_x_rrw = sig_x_rrw_0;                 
    sig_y_rrw = sig_y_rrw_0;              
    sig_z_rrw = sig_z_rrw_0;            

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
    ft =1* 0.05;
    wt = 2*pi*ft;
    radius = 25;
%    [x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N,z_p_N,z_v_N,lina_z_a_N] = trajectory3d_line(radius,gama_0,apha_0,wt,t0);
    [x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N,z_p_N,z_v_N,z_a_N] = trajectory3d_line(radius,gama_0,apha_0,wt,t0);
%    z_a_N = lina_z_a_N + g;% z-axis real accel in N-frame which subscribe she gravity
end

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
CE_B(:,:,1) = cz(psi_N(1))*cy(theta_N(1))*cx(phi_N(1));
QE_B = DTQ(CE_B(:,:,1));
QE_B_m = QE_B;
QB_E_m = -QE_B_m;
QB_E_m(4,1) = QE_B_m(4,1);
%DC_E_B_m(:,:,1) = [1 0 0;0 1 0; 0 0 1];% Direction Cosine Matrix
DC_E_B_m(:,:,1) = CE_B(:,:,1);
dQ = [0 0 0 1]';
dq1(1) = 2*dQ(1,1);
dq2(2) = 2*dQ(2,1);
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
    dq11(jj) = 2*dQ1(1,1);
    dq21(jj) = 2*dQ1(2,1);
    dq31(jj) = 2*dQ1(3,1);
end
% ====================================================
% Need to convert into body frame (B-frame) for accelerometer sensings and
% magnetometer sensing since they are mounted on the body frame
% ====================================================
x_a_B = zeros(1,n);
y_a_B = zeros(1,n);
z_a_B = zeros(1,n);
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
    x_m_B(i) = [C11 C12 C13]*mag_N';
    y_m_B(i) = [C21 C22 C23]*mag_N';
    z_m_B(i) = [C31 C32 C33]*mag_N';
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
mag_sig_x_rrw_0 = mag_err_flag*Mag/3600;
mag_sig_y_rrw_0 = mag_err_flag*Mag/3600;
mag_sig_z_rrw_0 = mag_err_flag*Mag/3600;
%
mag_sig_x_arw = mag_sig_x_arw_0;
mag_sig_y_arw = mag_sig_y_arw_0;
mag_sig_z_arw = mag_sig_z_arw_0;
mag_sig_x_rrw = mag_sig_x_rrw_0;
mag_sig_y_rrw = mag_sig_y_rrw_0;
mag_sig_z_rrw = mag_sig_z_rrw_0;
% magnetometer biases
[mag_bx]=Biasba1(dt,n,m,mag_bx0,mag_sig_x_arw_0);
[mag_by]=Biasbp1(dt,n,m,mag_by0,mag_sig_y_arw_0);
[mag_bz]=Biasbp1(dt,n,m,mag_bz0,mag_sig_z_arw_0);
% nmagnetometer noises
[mag_xm,mag_ym,mag_zm]=transform3d_m(x_m_B,mag_bx,y_m_B,mag_by,z_m_B,mag_bz,m,n,mag_sig_x_rrw_0,mag_sig_y_rrw_0,mag_sig_z_rrw_0);

%=============================================================================================================
% accelerometer (biases and noises)
%=====================================
bx0=1*0.01*g;                              % initial accel bias in g in along-direction
by0=-1*0.01*g;                             % initial accel bias in g in perpenticular-direction
bz0=1*0.01*g;                              % initial accel bias in g in perpenticular-direction
err_factor = 1.0;
% accelerometer bias stability
sig_xr_0 = err_factor*0.1*g/3600;         % accel bias stability in g/sec-sqrt(sec) in along-direction
sig_yr_0 = err_factor*0.1*g/3600;         % accel bias stability in g/sec-sqrt(sec) in penpenticular-direction
sig_zr_0 = err_factor*0.1*g/3600;         % accel bias stability in g/sec-sqrt(sec) in penpenticular-direction

% accelerate (noise)
sig_bx_0 = err_factor*0.01*g;              % accel noise in g in along-direction
sig_by_0 = err_factor*0.01*g;              % accel noise in g in penpenticular-direction
sig_bz_0 = err_factor*0.01*g;              % accel noise in g in penpenticular-direction

% accelerometer biases
[bx]=Biasba1(dt,n,m,bx0,sig_xr_0);
[by]=Biasbp1(dt,n,m,by0,sig_yr_0);
[bz]=Biasbp1(dt,n,m,bz0,sig_zr_0);
% accelerometer noises
[axm,aym,azm]=transform3d_m(x_a_B,bx,y_a_B,by,z_a_B,bz,m,n,sig_bx_0,sig_by_0,sig_bz_0);
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
% Define ""4 radio sensor"" parameters (noise)
radiosensor_err_factor = 1.0;
sig_x_r=radiosensor_err_factor*0.1;              % radio sensor measurement noise in meters x-direction
sig_y_r=radiosensor_err_factor*0.1;              % radio sensor measurement noise in meters y-direction
%=========================================================
[R1m,R2m,R3m,R4m,nvx_r,nvy_r] = radio_sensor3d_m_4(xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,x_p_N,y_p_N,z_p_N,sig_x_r,sig_y_r,n,m);%4 four radio sensors
% calculate position by least square
% position =zeros(3,n);
% for kk = 1:n        
% 
% 
% distance_sum = -R4m(kk)^2+yr4^2+xr4^2+zr4^2;
% 
% b=[R1m(kk)^2-yr1^2-xr1^2-zr1^2+distance_sum
%    R2m(kk)^2-yr2^2-xr2^2-zr2^2+distance_sum
%    R3m(kk)^2-yr3^2-xr3^2-zr3^2+distance_sum];
% %  d4^2-x_anchor_4^2-y_anchor_4^2+distance_sum
% A=-2.*[yr1-yr4 xr1-xr4 zr1-zr4 
%        yr2-yr4 xr2-xr4 zr2-zr4 
%        yr3-yr4 xr3-xr4 zr3-zr4];
% %    x_anchor_4-x_anchor_5 y_anchor_4-y_anchor_5
% 
% 
% position(:,kk)=inv(A'*A)*A'*b;
% end


%=========================================================
delta_t = dt;                                  % delta time for simulating the true dynamics = 0.01 sec
delta_s = 5*delta_t;                           % sampling at every 0.05 second for the Kalman filter
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
%================================================================
% Define the initial conditions for Euler angle error in N-frame
%================================================================
phierr = 0.5;       % x in deg
thetaerr = 0.5;     % y in deg
psierr = 0.5;       % z in deg
[phim_Nh,thetam_Nh,psim_Nh,bgx_h,bgy_h,bgz_h]=initial_estimate_value6_radio(m,phi_N,theta_N,psi_N,phierr,thetaerr,psierr,d2r);

        %================================================================
        % Define the initial conditions for the 9-states Kalman Filter
        % ==============================================================
        % It is noted that we increase (100 times) the initial covariance matrix values
        % in P00_z(7,7), P00_z(8,8),P00_z(9,9) to make accel bias errors converge faster
        % ==============================================================
        [xz_h,P00_z]=define_initial_condition_9(bx0,by0,bz0,xperr,yperr,zperr,xverr,yverr,zverr);
        
%================================================================
% Define the initial conditions for the 6-states Kalman Filter
% ==============================================================
[s6_xz_h,s6_P00_z]=define_initial_condition_6(bgx0,bgy0,bgz0,phierr,thetaerr,psierr,d2r);

        % ==============================================================
        % for 9-states filter
        F_z=zeros(9);
        %F_z = [0 0 0  1 0 0  0 0 0
        %       0 0 0  0 1 0  0 0 0
        %       0 0 0  0 0 1  0 0 0

        %       0 0 0  0 0 0  -C11 -C21 -C31
        %       0 0 0  0 0 0  -C12 -C22 -C32
        %       0 0 0  0 0 0  -C13 -C23 -C33

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

% ==============================================================
% for 6-states filter
s6_F_z=zeros(6);
%s6_F_z = [0 0 0  -C11 -C21 -C31
%          0 0 0  -C12 -C22 -C32
%          0 0 0  -C13 -C23 -C33

%          0 0 0  0 0 0
%          0 0 0  0 0 0
%          0 0 0  0 0 0];

s6_F_z(1,4) = -DC_E_B_m(1,1,1);s6_F_z(1,5) = -DC_E_B_m(2,1,1);s6_F_z(1,6) = -DC_E_B_m(3,1,1);%_m
s6_F_z(2,4) = -DC_E_B_m(1,2,1);s6_F_z(2,5) = -DC_E_B_m(2,2,1);s6_F_z(2,6) = -DC_E_B_m(3,2,1);
s6_F_z(3,4) = -DC_E_B_m(1,3,1);s6_F_z(3,5) = -DC_E_B_m(2,3,1);s6_F_z(3,6) = -DC_E_B_m(3,3,1);

s6_Q_z=zeros(6);


% ============================================================
% Start the simulation run
% ============================================================
% k=2;
for i=1:sensor_step
	for j=1:propagation_step
        k=1+j+(i-1)*propagation_step;
        
        bx_h(k)=bx_h(k-1);
        by_h(k)=by_h(k-1);
        bz_h(k)=bz_h(k-1);
        
        bgx_h(k)=bgx_h(k-1);
        bgy_h(k)=bgy_h(k-1);
        bgz_h(k)=bgz_h(k-1);        
        % =========================================
        % Perform inertial navigation computations
        % =========================================
        [xpm_Nh,ypm_Nh,zpm_Nh,xvm_Nh,yvm_Nh,zvm_Nh,xam_Nh,yam_Nh,zam_Nh,axm_h,aym_h,azm_h]=inertial_navigation_computation9_radio(xvm_Nh,yvm_Nh,zvm_Nh,xpm_Nh,ypm_Nh,zpm_Nh,xam_Nh,yam_Nh,zam_Nh,axm,aym,azm,bx_h,by_h,bz_h,DC_E_B_m,k,dt);

% =========================================
% 6-state EKF
% Perform inertial attitude computations
% =========================================        
        [phim_Nh,thetam_Nh,psim_Nh]=inertial_attitude_computation6_radio(w_EB_B_xm,w_EB_B_ym,w_EB_B_zm,phi_N,theta_N,psi_N,bgx_h,bgy_h,bgz_h,DC_E_B_m,k,dt);

        % ===========================================================
        % Perform Kalman filter propagation for 9-state Kalman filter
        % ===========================================================
        [phi_z,Q_z,F_z]=define_Dymamic_equation9_radio(F_z,Q_z,sig_bx,sig_by,sig_bz,sig_xr,sig_yr,sig_zr,DC_E_B_m,k,dt);
        [xz_h,P00_z]=Kalman_Filter_estimate1_radio(xz_h,phi_z,P00_z,Q_z,dt);
        
% ===========================================================
% Perform Kalman filter propagation for 6-state Kalman filter
% ===========================================================     
        [s6_phi_z,s6_Q_z,s6_F_z]=define_Dymamic_equation6_radio(s6_F_z,s6_Q_z,sig_x_arw,sig_y_arw,sig_z_arw,sig_x_rrw,sig_y_rrw,sig_z_rrw,DC_E_B_m,k,dt);
        [s6_xz_h,s6_P00_z]=Kalman_Filter_estimate1_radio(s6_xz_h,s6_phi_z,s6_P00_z,s6_Q_z,dt);
    end                 % end of filter propagation step
    
    % =======================================================
    % Perform Kalman filter updates for 9-states filter
    % =======================================================
    [H,R,R1m_h,R2m_h,R3m_h,R4m_h]=radio_discrete_5_3_EKF(xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,xpm_Nh,ypm_Nh,zpm_Nh,sig_x_r,sig_y_r,k);
    %   
    [P00_z,K_z,z_update,Mu_z]=Kalman_Filter_update_5_3_radio(P00_z,R1m,R2m,R3m,R4m,H,R,R1m_h,R2m_h,R3m_h,R4m_h,k);

    [xpm_Nh,ypm_Nh,zpm_Nh,xvm_Nh,yvm_Nh,zvm_Nh,bx_h,by_h,bz_h]=upon_radiosensor_measurement_5_3(xpm_Nh,ypm_Nh,zpm_Nh,xvm_Nh,yvm_Nh,zvm_Nh,k,bx_h,by_h,bz_h,z_update);
    % reset the errors after the filter updates
    xz_h=zeros(9,1);
    
% =======================================================
% Perform Kalman filter updates for 6-states filter
% =======================================================
    [s6_H,s6_R]=attitude_determination_6_1_EKF(bx_h,by_h,bz_h,sig_x_rrw,sig_y_rrw,sig_z_rrw,mag_sig_x_arw,mag_sig_y_arw,mag_sig_z_arw,mag_sig_x_rrw,mag_sig_y_rrw,mag_sig_z_rrw,DC_E_B_m,mag_N,k);
    %
    [s6_P00_z,s6_K_z,s6_z_update,s6_Mu_z]=Kalman_Filter_update_6_1_radio(s6_P00_z,axm_h,aym_h,azm_h,x_a_B,y_a_B,z_a_B,x_m_B,y_m_B,z_m_B,mag_xm,mag_ym,mag_zm,s6_H,s6_R,k);
    %
    [phim_Nh,thetam_Nh,psim_Nh,bgx_h,bgy_h,bgz_h]=upon_radiosensor_measurement_6_1(phim_Nh,thetam_Nh,psim_Nh,k,bgx_h,bgy_h,bgz_h,s6_z_update);

end                     % end of one Monte Caro run

%
plot53;
plot58;