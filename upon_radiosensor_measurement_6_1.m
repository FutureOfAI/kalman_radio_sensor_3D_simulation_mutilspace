function[dtheda_xh,dtheda_yh,dtheda_zh,bgx_h,bgy_h,bgz_h]=upon_radiosensor_measurement_6_1(dtheda_xh,dtheda_yh,dtheda_zh,k,bgx_h,bgy_h,bgz_h,s6_z_update)

dtheda_xh(k) = dtheda_xh(k) + s6_z_update(1);
dtheda_yh(k) = dtheda_yh(k) + s6_z_update(2);
dtheda_zh(k) = dtheda_zh(k) + s6_z_update(3);

bgx_h(k)=bgx_h(k) + s6_z_update(4);
bgy_h(k)=bgy_h(k) + s6_z_update(5);
bgz_h(k)=bgz_h(k) + s6_z_update(6);
end