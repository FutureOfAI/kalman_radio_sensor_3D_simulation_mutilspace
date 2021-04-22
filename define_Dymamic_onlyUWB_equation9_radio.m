function[phi_z,Q_z]=define_Dymamic_onlyUWB_equation9_radio(F_z,Q_z,sig_xr,sig_yr,sig_zr,dt)

phi_z = expm(F_z*dt);
lamda = 0.001; % simul 0.001
Q_z(7,7) = lamda*sig_xr^2;
Q_z(8,8) = lamda*sig_yr^2;
Q_z(9,9) = lamda*sig_zr^2;

end