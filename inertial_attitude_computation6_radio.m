function [QE_B_m]=inertial_attitude_computation6_radio(w_EB_B_xm,w_EB_B_ym,w_EB_B_zm,bgx_h,bgy_h,bgz_h,QE_B_m,k,dt)
%w_EB_B_xm(k) = w_EB_B_xm(k) - bgx_h(k-1);
%w_EB_B_ym(k) = w_EB_B_ym(k) - bgy_h(k-1);
%w_EB_B_zm(k) = w_EB_B_zm(k) - bgz_h(k-1);
%w_EB_B_xm(k-1) = w_EB_B_xm(k-1) - bgx_h(k-1);
%w_EB_B_ym(k-1) = w_EB_B_ym(k-1) - bgy_h(k-1);
%w_EB_B_zm(k-1) = w_EB_B_zm(k-1) - bgz_h(k-1);
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
%    DC_E_B_m(:,:,k) = q2dc(QE_B_m);% calculate DCM measurement
end
%