function[xpm_Nh,ypm_Nh,zpm_Nh,xvm_Nh,yvm_Nh,zvm_Nh,bx_h,by_h,bz_h]=upon_radiosensor_measurement_5_3(xpm_Nh,ypm_Nh,zpm_Nh,xvm_Nh,yvm_Nh,zvm_Nh,k,bx_h,by_h,bz_h,z_update)

xpm_Nh(k)=xpm_Nh(k)+z_update(1);
ypm_Nh(k)=ypm_Nh(k)+z_update(2);
zpm_Nh(k)=zpm_Nh(k)+z_update(3);

xvm_Nh(k)=xvm_Nh(k)+z_update(4);
yvm_Nh(k)=yvm_Nh(k)+z_update(5);
zvm_Nh(k)=zvm_Nh(k)+z_update(6);

bx_h(k) = bx_h(k) + z_update(7);
by_h(k) = by_h(k) + z_update(8);
bz_h(k) = bz_h(k) + z_update(9);
end