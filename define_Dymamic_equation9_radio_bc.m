function[phi_z,Q_z,F_z]=define_Dymamic_equation9_radio(F_z,Q_z,sig_bx,sig_by,sig_bz,sig_xr,sig_yr,sig_zr,DC_E_B_m,k,dt)
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

Q_z0(1:3,1:3) = [sig_bx^2 0 0 
    0 sig_by^2 0 
    0 0 sig_bz^2]; 
Q_z(4:6,4:6) = DC_E_B_m(:,:,k-1)'*Q_z0*DC_E_B_m(:,:,k-1);
Q_z(7,7) = sig_xr^2;
Q_z(8,8) = sig_yr^2;
Q_z(9,9) = sig_zr^2;

end