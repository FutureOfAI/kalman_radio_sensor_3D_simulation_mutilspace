function [P00_z,K_z,z_update,Mu_z]=Kalman_Filter_update_uwb_SHFAF(P00_z,z_update,R1m,R2m,R3m,R4m,H,R,R1m_h,R2m_h,R3m_h,R4m_h,k)

zxm_z1(k) = R1m(k)-R1m_h(k)' ;
zxm_z2(k) = R2m(k)-R2m_h(k)' ;
zxm_z3(k) = R3m(k)-R3m_h(k)' ;
zxm_z4(k) = R4m(k)-R4m_h(k)' ;

Mu_z=[zxm_z1(k)
    zxm_z2(k)
    zxm_z3(k)
    zxm_z4(k)];

% SHFAF
err_z_h = H*P00_z*H'+R + Mu_z*Mu_z';
G = [err_z_h(1,1);err_z_h(2,2);err_z_h(3,3);err_z_h(4,4)];
err_z = H*P00_z*H'+R + H*z_update*z_update'*H';
D = [err_z(1,1);err_z(2,2);err_z(3,3);err_z(4,4)];
Th = 700; % 1.5 for simul  700 for rec exp
Fr = zeros(4);

for i=1:4
    if D(i) ~= 0
        M(i) = G(i)/D(i);
    else
        M(i) = 1;
    end
    if M(i)>Th
       Fr(i,i) =  1/sqrt(M(i));
    else
        Fr(i,i) = 1;
    end
end

K_z=P00_z*H'/(H*P00_z*H'+R); % R/Fr

z_update = K_z*Mu_z;
I=eye(9);
P00_z=(I-K_z*H)*P00_z;                     % P(k|k)

end

