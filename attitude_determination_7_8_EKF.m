function [s6_H,s6_R]=attitude_determination_7_8_EKF(sig_bx_0,sig_by_0,sig_bz_0)
s6_H = zeros(3,6);
s6_H(1,1) = 1;
s6_H(2,2) = 1;
s6_H(3,3) = 1;

s6_R = 1*[sig_bx_0^2 0 0
    0 sig_by_0^2 0
    0 0 sig_bz_0^2];
end