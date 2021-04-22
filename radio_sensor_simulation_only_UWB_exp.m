% Program name: radio_sensor_simulation_only_UWB.m
% 9-state Kalman filter with 4 radio sensors for 6DOF motion tracking
% simulation with 6-state EKF with accel and magnetometers as measurements
% uwb NLOS simulation
% 2/5/2021
% put project to github.com, teacher could review my changes
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clear all;
clc;
format short e

LS_positioning
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
        yr1 = 0;% in meter
        zr1 = 2.52;% in meter
        xr2 = 5.79;
        yr2 = 0;
        zr2 = 2.52;
        xr3 = 1.98;% in meter
        yr3 = 5.79*cos(20*d2r);% in meter
        zr3 = 2.52;
        xr4 = 0.41;
        yr4 = 0.64;
        zr4 = 0.464;% in meter
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
        radius = 1*10;
  %      T = 1;
        delta_t = dt;                                  % delta time for simulating the true dynamics = 0.01 sec
        delta_s = 5*delta_t;                           % sampling at every 0.5 second for the Kalman filter
        err_flag = 0;                                   % flag for turning on the magnetometer and accelerometer errors
  % ================================================
        t0 = 0:dt:T;
        t0_1 = t0';
        n = length(t0);
        m = size(t0_1);
        t00 = t0;
  % ================================================
        phi_0 = 0*1.5*d2r;%x
        theta_0 = -0*1.5*d2r;%y
        psi_0 = 1*4.5*d2r;%z
   %     wn = 2*pi*fn*d2r;
        wn = 2*pi*fn;% rad
        wx(1:n) = 0*wn;
        wy(1:n) = 0*wn;
        wz(1:n) = 0*wn;
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
    profile_flag = 1;        % straight motion
%     profile_flag = 2;         % screw motion
%     profile_flag = 3;        % circular motion
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
    mag_angle = 1.000; % 4*pi
    for i = 2:n,
%        phi(i) = phi(i-1) + wxm(i-1)*dt;
%        theta(i) = theta(i-1) + wym(i-1)*dt;
%        psi(i) = psi(i-1) + wzm(i-1)*dt;
        
        
         phi_N(i) = phi_N(i-1) + wx(i-1)*dt;
%         phi_N(i) = phi_0*cos(2*pi*fn*(i-1)*dt);
%         wx(i) = -2*pi*fn*phi_0*sin(2*pi*0.1*fn*(i-1)*dt);
         theta_N(i) = theta_N(i-1) + wy(i-1)*dt;
         psi_N(i) = psi_N(i-1) + wz(i-1)*dt;
         t3(i) = mag_angle*sin(2*pi*fz*(i-1)*dt);
%          wz(i) = 2*pi*fz*mag_angle*cos(2*pi*fz*(i-1)*dt);
    end
%    wx(1) = 0;
%     wz(1) = 2*pi*fz*mag_angle;
% ====================================================
% Generate platform motion in N-frame
% ===================================================
    [x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N,z_p_N,z_v_N,z_a_N] = trajectory3d_circle(radius,wn,10*t3,mag_angle);
end

if (profile_flag == 3)
    mag_angle = 2.000;
    for i = 2:n,
         phi_N(i) = phi_N(i-1) + wx(i-1)*dt;
         theta_N(i) = theta_N(i-1) + wy(i-1)*dt;
         psi_N(i) = psi_N(i-1) + wz(i-1)*dt;
         t3(i) = mag_angle*sin(2*pi*fz*(i-1)*dt);
%          wz(i) = 2*pi*fz*mag_angle*cos(2*pi*fz*(i-1)*dt);
    end
    [x_p_N,x_v_N,x_a_N,y_p_N,y_v_N,y_a_N,z_p_N,z_v_N,z_a_N] = trajectory2d_circle (radius,t3,wt);%round cycle
end

% ========================================================
% accelerometer (biases and noises)
%=========================================================
bx0=0*0.1*g;                              % initial accel bias in g in along-direction
by0=-0*0.1*g;                             % initial accel bias in g in perpenticular-direction
bz0=0*0.1*g;                              % initial accel bias in g in perpenticular-direction
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
[axm,aym,azm]=transform3d_m(x_a_N,bx,y_a_N,by,z_a_N,bz,m,n,err_flag*sig_bx_0,err_flag*sig_by_0,err_flag*sig_bz_0);
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
[xpm_Nh,ypm_Nh,zpm_Nh,xvm_Nh,yvm_Nh,zvm_Nh,xam_Nh,yam_Nh,zam_Nh,axm_h,aym_h,azm_h,bx_h,by_h,bz_h]=initial_estimate_value9_radio_onlyUWB(m,x_a_N,y_a_N,z_a_N,x_v_N,y_v_N,z_v_N,x_p_N,y_p_N,z_p_N,axm,aym,azm,xverr,yverr,zverr,xperr,yperr,zperr,xaerr,yaerr,zaerr);
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

%       0 0 0  0 0 0  1 0 0
%       0 0 0  0 0 0  0 1 0
%       0 0 0  0 0 0  0 0 1

%       0 0 0  0 0 0  0 0 0
%       0 0 0  0 0 0  0 0 0
%       0 0 0  0 0 0  0 0 0];
F_z(1,4) = 1;
F_z(2,5) = 1;
F_z(3,6) = 1;

F_z(4,7) = 1;
F_z(5,8) = 1;
F_z(6,9) = 1;

Q_z=zeros(9);
% ============================================================
% ============================================================
% Start the simulation run                                   |
% ============================================================
% ============================================================
for i=1:sensor_step
	for j=1:propagation_step
        k=1+j+(i-1)*propagation_step;
   % accel biases     
        bx_h(k)=bx_h(k-1);
        by_h(k)=by_h(k-1);
        bz_h(k)=bz_h(k-1);
% =========================================
% Perform inertial navigation computations
% =========================================
        [xpm_Nh,ypm_Nh,zpm_Nh,xvm_Nh,yvm_Nh,zvm_Nh,xam_Nh,yam_Nh,zam_Nh,axm_h,aym_h,azm_h]=motion_navigation_computation9_radio(xvm_Nh,yvm_Nh,zvm_Nh,xpm_Nh,ypm_Nh,zpm_Nh,xam_Nh,yam_Nh,zam_Nh,axm,aym,azm,bx_h,by_h,bz_h,k,dt);
        x_err(k) = x_p_N(k) - xpm_Nh(k);
        y_err(k) = y_p_N(k) - ypm_Nh(k);
        z_err(k) = z_p_N(k) - zpm_Nh(k);
% ===========================================================
% Perform Kalman filter propagation for 9-state Kalman filter
% ===========================================================
        [phi_z,Q_z]=define_Dymamic_onlyUWB_equation9_radio(F_z,Q_z,sig_xr,sig_yr,sig_zr,dt);
        [xz_h,P00_z]=Kalman_Filter_estimate1_radio(xz_h,phi_z,P00_z,Q_z,dt);
% ========================================================
    end                 % end of filter propagation step
% ========================================================
% Perform Kalman filter updates for 9-states filter
% =======================================================
    [H,R,R1m_h,R2m_h,R3m_h,R4m_h]=radio_discrete_5_3_EKF(xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,xpm_Nh,ypm_Nh,zpm_Nh,sig_x_r,sig_y_r,k,simul_nlos);
%   
    [P00_z,K_z,z_update,Mu_z]=Kalman_Filter_update_5_3_radio(P00_z,pos(1,:)',pos(2,:)',pos(3,:)',pos(4,:)',H,R,R1m_h,R2m_h,R3m_h,R4m_h,k);% experiment data @@
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
end                     % end of one Monte Caro run

%
n1 = 0.5*(k-1);
% n1 = 500;
n2 = k-1;

% ======================================================
% Three sigma value computations
% ======================================================
D = [sort(abs(x_err(n1:n2)')) sort(abs(y_err(n1:n2)')) sort(abs(z_err(n1:n2)'))];
N = length(D);
number = fix(0.9973*N);
D1 = [D(number,1) D(number,2) D(number,3)]

F = [sort(abs(x_err(1000:1500)')) sort(abs(y_err(1000:1500)')) sort(abs(z_err(1000:1500)'))];
N = length(F);
number = fix(0.9973*N);
F1 = [F(number,1) F(number,2) F(number,3)]
% paperplot;
% plot58;
plot_only_UWB;