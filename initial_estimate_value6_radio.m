function[dtheda_xh,dtheda_yh,dtheda_zh,bgx_h,bgy_h,bgz_h]=initial_estimate_value6_radio(m,dtheda_x,dtheda_y,dtheda_z,phierr,thetaerr,psierr,d2r)

dtheda_xh=zeros(m);
dtheda_yh=zeros(m);
dtheda_zh=zeros(m);
bgx_h=zeros(m);
bgy_h=zeros(m);
bgz_h=zeros(m);

dtheda_xh(1) = dtheda_x + phierr*rand*d2r;
dtheda_yh(1) = dtheda_y + thetaerr*rand*d2r;
dtheda_zh(1) = dtheda_z + psierr*rand*d2r;

bgx_h(1) = 0;
bgy_h(1) = 0;
bgz_h(1) = 0;
end