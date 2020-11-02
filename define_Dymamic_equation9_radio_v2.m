function[phi_z,Q_z,F_z]=define_Dymamic_equation9_radio_v2(F_z,Q_z,sig_bx,sig_by,sig_bz,sig_xr,sig_yr,sig_zr,DC_E_B_m,k,dt,axm,aym,azm,dq11,dq21,dq31,simul_nlos)
F_z(4,7) = -DC_E_B_m(1,1,k-1);
F_z(4,8) = -DC_E_B_m(2,1,k-1);
F_z(4,9) = -DC_E_B_m(3,1,k-1);
F_z(5,7) = -DC_E_B_m(1,2,k-1);
F_z(5,8) = -DC_E_B_m(2,2,k-1);
F_z(5,9) = -DC_E_B_m(3,2,k-1);
F_z(6,7) = -DC_E_B_m(1,3,k-1);
F_z(6,8) = -DC_E_B_m(2,3,k-1);
F_z(6,9) = -DC_E_B_m(3,3,k-1);

phi_z = expm(F_z*dt);

if simul_nlos == 1

    if k>1000 && k<1500
        lamda =  1;%0.02
    else
        lamda = 1;
    end

else
%     lamda = 0.15;
    lamda = 100;
    tmpk = 1;
end

tmp = DC_E_B_m(:,:,k-1)*[axm,aym,azm]';
tmpx = zeros(3);
tmpEphi = zeros(3);
tmpx(1,1) = tmp(1);
tmpx(2,2) = tmp(2);
tmpx(3,3) = tmp(3);
tmpEphi(1,1) = dq11*dq11;
tmpEphi(2,2) = dq21*dq21;
tmpEphi(3,3) = dq31*dq31;

% test = tmpx*tmpEphi*tmpx';

Q_z0(1:3,1:3) = lamda*[sig_bx^2 0 0 
    0 sig_by^2 0 
    0 0 sig_bz^2]; 
Q_z(4:6,4:6) = DC_E_B_m(:,:,k-1)'*Q_z0*DC_E_B_m(:,:,k-1) + tmpk*tmpx*tmpEphi*tmpx';
Q_z(7,7) = sig_xr^2;
Q_z(8,8) = sig_yr^2;
Q_z(9,9) = sig_zr^2;

end