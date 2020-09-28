function[dtheda_xh,dtheda_yh,dtheda_zh]=upon_radiosensor_measurement_6_1_B(dtheda_xh,dtheda_yh,dtheda_zh,k,s3_z_update)

dtheda_xh(k) = dtheda_xh(k) + s3_z_update(1);
dtheda_yh(k) = dtheda_yh(k) + s3_z_update(2);
dtheda_zh(k) = dtheda_zh(k) + s3_z_update(3);

end