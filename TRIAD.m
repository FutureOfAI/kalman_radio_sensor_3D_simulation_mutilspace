function [C_E_B_e]=TRIAD(axm,aym,azm,x_a_N,y_a_N,z_a_N,mag_xm,mag_ym,mag_zm,mag_E,k)
x_a_N(k) = 0;
y_a_N(k) = 0;
z_a_N(k) = 0;
a_B = [axm(k) aym(k) azm(k)];
q_B = a_B/sqrt((a_B*a_B'));
m_B = [mag_xm(k) mag_ym(k) mag_zm(k)];
m_B_u = m_B/sqrt((m_B*m_B'));
r_B_n = cross(q_B, m_B_u);
r_B = r_B_n/sqrt(r_B_n*r_B_n');
s_B = cross(q_B, r_B);
%M_B = [q_B; r_B; s_B];
M_B = [s_B; r_B; q_B];

acc_g = [x_a_N(k) y_a_N(k) z_a_N(k) + 9.8];
q_E = acc_g/sqrt((acc_g*acc_g'));
mag_E_u = mag_E/sqrt((mag_E*mag_E'));
r_E_n = cross(q_E, mag_E_u);
r_E = r_E_n/sqrt(r_E_n*r_E_n');
s_E = cross(q_E, r_E);
%M_E = [q_E; r_E; s_E];
M_E = [s_E; r_E; q_E];

C_E_B_e = M_B*M_E';
%Q_E_B_e(:,k) = DTQ(C_E_B_e),
end