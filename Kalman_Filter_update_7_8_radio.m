function [s6_P00_z,s6_K_z,s6_z_update,s6_Mu_z,dQ,dQ11]=Kalman_Filter_update_7_8_radio(s6_P00_z,Q_E_B_e,QE_B_m,s6_H,s6_R,Q_E_B,k)
% Testing the TRIAD algorithm
QE_B = Q_E_B(:,k);
QB_E = -QE_B;
QB_E(4,1) = QE_B(4,1);
dQ11 = 2*qmult(Q_E_B_e(:,k)',QB_E');
%
Q_B_E_m = - QE_B_m;
Q_B_E_m(4,1) = QE_B_m(4,1);
dQ = 2*qmult(Q_E_B_e(:,k)',Q_B_E_m');
%dQ = 2*qmult(Q_E_B_e(:,k)',QB_E');
% Form the measurement residuals or mu
s6_Mu_z(1:3) =  [dQ(1,1) dQ(2,1) dQ(3,1)];
%s6_Mu_z(1:3) =  [dQ(3,1) dQ(2,1) dQ(1,1)];
% Computer the Kalman filter gain matrix K
s6_K_z=s6_P00_z*s6_H'/(s6_H*s6_P00_z*s6_H'+s6_R);
% Computer the correction vectors
s6_z_update = s6_K_z*s6_Mu_z';
% Perform the Kalman filter error covariance matrix P updates
I=eye(6);
s6_P00_z=(I-s6_K_z*s6_H)*s6_P00_z;  % P(k|k)

end