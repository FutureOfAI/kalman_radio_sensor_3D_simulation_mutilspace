function [P00_z,K_z,z_update,Mu_z]=Kalman_Filter_update_5_3_radio(P00_z,R1m,R2m,R3m,R4m,H,R,R1m_h,R2m_h,R3m_h,R4m_h,k)

zxm_z1(k) = R1m(k)-R1m_h(k)' ;
zxm_z2(k) = R2m(k)-R2m_h(k)' ;
zxm_z3(k) = R3m(k)-R3m_h(k)' ;
zxm_z4(k) = R4m(k)-R4m_h(k)' ;

Mu_z=[zxm_z1(k)
    zxm_z2(k)
    zxm_z3(k)
    zxm_z4(k)];
K_z=P00_z*H'/(H*P00_z*H'+R);

z_update = K_z*Mu_z;
I=eye(9);
P00_z=(I-K_z*H)*P00_z;                     % P(k|k)

end

