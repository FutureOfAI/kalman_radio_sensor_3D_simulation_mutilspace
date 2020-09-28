function [s6_H,s6_R]=attitude_determination_6_1_EKF(x_a_N,y_a_N,z_a_N,sig_bx_0,sig_by_0,sig_bz_0,bx0,by0,bz0,mag_sig_x_arw,mag_sig_y_arw,mag_sig_z_arw,mag_bx0,mag_by0,mag_bz0,DC_E_B_m,mag_E,dt,k)
s6_H = zeros(6);
accel_E(1) = x_a_N(k);
accel_E(2) = y_a_N(k);
accel_E(3) = z_a_N(k) + 9.8;
accel_Nm=[0 -accel_E(3) accel_E(2)
          accel_E(3) 0 -accel_E(1)
         -accel_E(2) accel_E(1) 0];
mag_Nm=[0 -mag_E(3) mag_E(2)
        mag_E(3) 0 -mag_E(1)
       -mag_E(2) mag_E(1) 0];
s6_H(1:3,1:3)=DC_E_B_m(:,:,k)*accel_Nm;
s6_H(4:6,1:3)=DC_E_B_m(:,:,k)*mag_Nm;

% s6_R = 1*[sig_bx_0^2+(bx0*dt)^2 0 0 0 0 0
%         0 sig_by_0^2+(by0*dt)^2 0 0 0 0
%         0 0 sig_bz_0^2+(bz0*dt)^2 0 0 0
%         0 0 0 mag_sig_x_arw^2+(mag_bx0*dt)^2 0 0
%         0 0 0 0 mag_sig_y_arw^2+(mag_by0*dt)^2 0
%         0 0 0 0 0 mag_sig_z_arw^2+(mag_bz0*dt)^2]; 

s6_R = 1*[(sig_bx_0 + bx0*dt)^2 0 0 0 0 0
        0 (sig_by_0 + by0*dt)^2 0 0 0 0
        0 0 (sig_bz_0 + bz0*dt)^2 0 0 0
        0 0 0 (mag_sig_x_arw + mag_bx0*dt)^2 0 0
        0 0 0 0 (mag_sig_y_arw + mag_by0*dt)^2 0
        0 0 0 0 0 (mag_sig_z_arw + mag_bz0*dt)^2]; 

end