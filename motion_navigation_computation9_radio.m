function[xpm_Nh,ypm_Nh,zpm_Nh,xvm_Nh,yvm_Nh,zvm_Nh,xam_Nh,yam_Nh,zam_Nh,axm_h,aym_h,azm_h]=motion_navigation_computation9_radio(xvm_Nh,yvm_Nh,zvm_Nh,xpm_Nh,ypm_Nh,zpm_Nh,xam_Nh,yam_Nh,zam_Nh,axm,aym,azm,bx_h,by_h,bz_h,k,dt)

axm_h(k)=axm(k)-bx_h(k);
aym_h(k)=aym(k)-by_h(k);
azm_h(k)=azm(k)-bz_h(k);
axm_h(k-1)=axm(k-1)-bx_h(k-1);
aym_h(k-1)=aym(k-1)-by_h(k-1);
azm_h(k-1)=azm(k-1)-bz_h(k-1);

% compute N-frame accelerations, velocities, and positions
xam_Nh(k) = axm_h(k);
yam_Nh(k) = aym_h(k);
zam_Nh(k) = azm_h(k);
%
xvm_Nh(k) = xvm_Nh(k-1)+(xam_Nh(k)+xam_Nh(k-1))*dt/2;
yvm_Nh(k) = yvm_Nh(k-1)+(yam_Nh(k)+yam_Nh(k-1))*dt/2;
zvm_Nh(k) = zvm_Nh(k-1)+(zam_Nh(k)+zam_Nh(k-1))*dt/2;

xpm_Nh(k) = xpm_Nh(k-1)+ (xvm_Nh(k)+xvm_Nh(k-1))*dt/2;
ypm_Nh(k) = ypm_Nh(k-1)+ (yvm_Nh(k)+yvm_Nh(k-1))*dt/2;
zpm_Nh(k) = zpm_Nh(k-1)+ (zvm_Nh(k)+zvm_Nh(k-1))*dt/2;

end