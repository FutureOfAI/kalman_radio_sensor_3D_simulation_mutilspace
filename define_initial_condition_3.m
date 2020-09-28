function [s3_xz_h,s3_P00_z]=define_initial_condition_3(phierr,thetaerr,psierr,d2r)

%s3_xz_h=zeros(3,1);      

s3_xz_h(1,1) = phierr*d2r;              % x1(1),angle error in x-axis (rad)
s3_xz_h(2,1) = thetaerr*d2r;              % x2(1),angle error in y-axis (rad)
s3_xz_h(3,1) = psierr*d2r;              % x3(1),angle error in z-axis (rad)

s3_P00_z = zeros(3,3);

s3_P00_z(1,1) = (phierr*d2r)^2;
s3_P00_z(2,2) = (thetaerr*d2r)^2;
s3_P00_z(3,3) = 1*(psierr*d2r)^2;


end