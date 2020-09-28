function [xpm_Nh,ypm_Nh,zpm_Nh,xvm_Nh,yvm_Nh,zvm_Nh,xam_Nh,yam_Nh,zam_Nh,axm_h,aym_h,azm_h,bx_h,by_h,bz_h]=initial_estimate_value9_radio(m,x_a_N,y_a_N,z_a_N,x_v_N,y_v_N,z_v_N,x_p_N,y_p_N,z_p_N,axm,aym,azm,xverr,yverr,zverr,xperr,yperr,zperr,xaerr,yaerr,zaerr)
%
axm_h=zeros(m);
aym_h=zeros(m);
azm_h=zeros(m);
by_h=zeros(m);
bx_h=zeros(m);
bz_h=zeros(m);

xvm_Nh=zeros(m);
yvm_Nh=zeros(m);
zvm_Nh=zeros(m);

xpm_Nh=zeros(m);
ypm_Nh=zeros(m);
zpm_Nh=zeros(m);

xam_Nh=zeros(m);
yam_Nh=zeros(m);
zam_Nh=zeros(m);

axm_h(1)=axm(1);
aym_h(1)=aym(1);
azm_h(1)=azm(1);

by_h(1)=0;
bx_h(1)=0;
bz_h(1)=0;

xvm_Nh(1)=x_v_N(1) + xverr*rand;
yvm_Nh(1)=y_v_N(1) + yverr*rand;
zvm_Nh(1)=z_v_N(1) + zverr*rand;

xpm_Nh(1)=x_p_N(1) + xperr*rand;
ypm_Nh(1)=y_p_N(1) + yperr*rand;
zpm_Nh(1)=z_p_N(1) + zperr*rand;

xam_Nh(1)=x_a_N(1) + xaerr*rand;
yam_Nh(1)=y_a_N(1) + yaerr*rand;
zam_Nh(1)=z_a_N(1) + zaerr*rand;

end