function [s6_xz_h,s6_P00_z]=define_initial_condition_6(bgx0,bgy0,bgz0,phierr,thetaerr,psierr,d2r)

s6_xz_h=zeros(6,1);      

s6_xz_h(1,1) = phierr*d2r;              % x1(1),angle error in x-axis (rad)
s6_xz_h(2,1) = thetaerr*d2r;              % x2(1),angle error in y-axis (rad)
s6_xz_h(3,1) = psierr*d2r;              % x3(1),angle error in z-axis (rad)
s6_xz_h(4,1) = bgx0;                % x4(1),gyro bias error in x-axis
s6_xz_h(5,1) = bgy0;                % x5(1),gyro bias error in y-axis
s6_xz_h(6,1) = bgz0;                % x6(1),gyro bias error in z-axis

s6_P00_z = zeros(6,6);

s6_P00_z(1,1) = (phierr*d2r)^2;
s6_P00_z(2,2) = (thetaerr*d2r)^2;
s6_P00_z(3,3) = 1*(psierr*d2r)^2;
s6_P00_z(4,4) = 1*bgx0^2;
s6_P00_z(5,5) = 1*bgy0^2;
s6_P00_z(6,6) = 1*bgz0^2;

end