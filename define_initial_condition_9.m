function [xz_h,P00_z]=define_initial_condition_9(bx0,by0,bz0,xperr,yperr,zperr,xverr,yverr,zverr)

xz_h=zeros(9,1);      

xz_h(1,1) = xperr;              % x1(1),position error in x-axis (m)
xz_h(2,1) = yperr;              % x2(1),position error in y-axis (m)
xz_h(3,1) = zperr;              % x3(1),position error in z-axis (m)

xz_h(4,1) = xverr;              % x4(1),velocity error in x-axis (m/sec)
xz_h(5,1) = yverr;              % x5(1),velocity error in y-axis (m/sec)
xz_h(6,1) = zverr;              % x6(1),velocity error in z-axis (m/sec)

xz_h(7,1) = bx0;                % x7(1),accel bias error in x-axis (m/sec^2)
xz_h(8,1) = by0;                % x8(1),accel bias error in y-axis (m/sec^2)
xz_h(9,1) = bz0;                % x9(1),accel bias error in z-axis (m/sec^2)


P00_z = zeros(9,9);
P00_z(1,1) = xperr^2;
P00_z(2,2) = yperr^2;
P00_z(3,3) = zperr^2;
P00_z(4,4) = xverr^2;
P00_z(5,5) = yverr^2;
P00_z(6,6) = zverr^2;
P00_z(7,7) = 100*bx0^2;
P00_z(8,8) = 100*by0^2;
P00_z(9,9) = 100*bz0^2;
end