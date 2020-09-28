function [s6_P00_z,s6_K_z,s6_z_update,s6_Mu_z]=Kalman_Filter_update_6_1_radio(s6_P00_z,axm,aym,azm,x_a_N,y_a_N,z_a_N,mag_xm,mag_ym,mag_zm,mag_E,DC_E_B_m,s6_H,s6_R,k)
C11 = DC_E_B_m(1,1,k);
C12 = DC_E_B_m(1,2,k);
C13 = DC_E_B_m(1,3,k);
C21 = DC_E_B_m(2,1,k);
C22 = DC_E_B_m(2,2,k);
C23 = DC_E_B_m(2,3,k);
C31 = DC_E_B_m(3,1,k);
C32 = DC_E_B_m(3,2,k);
C33 = DC_E_B_m(3,3,k);
% estimated accelerations in body frame
x_a_B_h(k) = [C11 C12 C13]*[x_a_N(k) y_a_N(k) z_a_N(k) + 9.8]';
y_a_B_h(k) = [C21 C22 C23]*[x_a_N(k) y_a_N(k) z_a_N(k) + 9.8]';
z_a_B_h(k) = [C31 C32 C33]*[x_a_N(k) y_a_N(k) z_a_N(k) + 9.8]';% linear accel in z-axis
% estimated magnet fields in body frame
x_m_B_h(k) = [C11 C12 C13]*mag_E';
y_m_B_h(k) = [C21 C22 C23]*mag_E';
z_m_B_h(k) = [C31 C32 C33]*mag_E';

acc_zxm_z1(k) = axm(k)- x_a_B_h(k);
acc_zxm_z2(k) = aym(k)- y_a_B_h(k);
acc_zxm_z3(k) = azm(k)- z_a_B_h(k);
mag_zxm_z4(k) = mag_xm(k) - x_m_B_h(k);
mag_zxm_z5(k) = mag_ym(k) - y_m_B_h(k);
mag_zxm_z6(k) = mag_zm(k) - z_m_B_h(k);

s6_Mu_z = [acc_zxm_z1(k)
        acc_zxm_z2(k)
        acc_zxm_z3(k)
        mag_zxm_z4(k)
        mag_zxm_z5(k)
        mag_zxm_z6(k)];
    
s6_K_z=s6_P00_z*s6_H'/(s6_H*s6_P00_z*s6_H'+s6_R);

s6_z_update = s6_K_z*s6_Mu_z;
I=eye(6);
s6_P00_z=(I-s6_K_z*s6_H)*s6_P00_z;  % P(k|k)

end