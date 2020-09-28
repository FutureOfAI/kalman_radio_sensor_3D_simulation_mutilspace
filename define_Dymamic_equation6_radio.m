function [s6_phi_z ]=define_Dymamic_equation6_radio(DC_E_B_m,k,dt)
s6_F_z(1,4) = -DC_E_B_m(1,1,k-1);
s6_F_z(1,5) = -DC_E_B_m(2,1,k-1);
s6_F_z(1,6) = -DC_E_B_m(3,1,k-1);
s6_F_z(2,4) = -DC_E_B_m(1,2,k-1);
s6_F_z(2,5) = -DC_E_B_m(2,2,k-1);
s6_F_z(2,6) = -DC_E_B_m(3,2,k-1);
s6_F_z(3,4) = -DC_E_B_m(1,3,k-1);
s6_F_z(3,5) = -DC_E_B_m(2,3,k-1);
s6_F_z(3,6) = -DC_E_B_m(3,3,k-1);

s6_phi_z = expm(s6_F_z*dt);
end