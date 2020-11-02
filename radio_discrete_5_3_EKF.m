function[H,R,R1m_h,R2m_h,R3m_h,R4m_h]=radio_discrete_5_3_EKF(xr1,yr1,zr1,xr2,yr2,zr2,xr3,yr3,zr3,xr4,yr4,zr4,xpm_Nh,ypm_Nh,zpm_Nh,sig_x_r,sig_y_r,k,simul_nlos)

R1m_h(k) = sqrt((xr1-xpm_Nh(k))^2+(yr1-ypm_Nh(k))^2+(zr1-zpm_Nh(k))^2);
R2m_h(k) = sqrt((xr2-xpm_Nh(k))^2+(yr2-ypm_Nh(k))^2+(zr2-zpm_Nh(k))^2);
R3m_h(k) = sqrt((xr3-xpm_Nh(k))^2+(yr3-ypm_Nh(k))^2+(zr3-zpm_Nh(k))^2);
R4m_h(k) = sqrt((xr4-xpm_Nh(k))^2+(yr4-ypm_Nh(k))^2+(zr4-zpm_Nh(k))^2);


r1_partial_x =-(xr1-xpm_Nh(k))/R1m_h(k);
r1_partial_y =-(yr1-ypm_Nh(k))/R1m_h(k);
r1_partial_z =-(zr1-zpm_Nh(k))/R1m_h(k);

r2_partial_x =-(xr2-xpm_Nh(k))/R2m_h(k); 
r2_partial_y =-(yr2-ypm_Nh(k))/R2m_h(k);
r2_partial_z =-(zr2-zpm_Nh(k))/R2m_h(k);

r3_partial_x =-(xr3-xpm_Nh(k))/R3m_h(k);
r3_partial_y =-(yr3-ypm_Nh(k))/R3m_h(k);
r3_partial_z =-(zr3-zpm_Nh(k))/R3m_h(k);

r4_partial_x =-(xr4-xpm_Nh(k))/R4m_h(k); 
r4_partial_y =-(yr4-ypm_Nh(k))/R4m_h(k);
r4_partial_z =-(zr4-zpm_Nh(k))/R4m_h(k);

H = [r1_partial_x r1_partial_y r1_partial_z 0 0 0 0 0 0
     r2_partial_x r2_partial_y r2_partial_z 0 0 0 0 0 0
     r3_partial_x r3_partial_y r3_partial_z 0 0 0 0 0 0
     r4_partial_x r4_partial_y r4_partial_z 0 0 0 0 0 0 ];

if simul_nlos == 1 
 
    if k>1000 && k<1500
        lamda =  350; %0.4
    else
        lamda = 1;
    end

else
%     lamda = 0.001; % 0.001
    lamda = 1;
end
 
R = lamda*[sig_x_r^2 0 0 0
    0 sig_y_r^2 0 0
    0 0 sig_x_r^2 0
    0 0 0 sig_y_r^2];    
end