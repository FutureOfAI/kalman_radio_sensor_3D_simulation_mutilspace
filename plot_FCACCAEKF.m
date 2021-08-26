close all;
clear all;
clc;

data = load('linear_actual_pos_a0_16_v3.mat');
ap = data.actual_pos;
data = load('linear_measure_pos_FCACCAEKF_v3.mat');
mpfca = data.measure_pos;
data = load('linear_measure_pos_SHAF_v3.mat');
mpa = data.measure_pos;
data = load('linear_measure_pos_EKF_v3.mat');
mp = data.measure_pos;
data = load('linear_UWB_pos_a0_16_v3.mat');
uwb = data.r;

figure (1)
tt1 = 1500; % 8000
tt2 = 2500; % 9000
plot3(ap(1,tt1:tt2),ap(2,tt1:tt2),ap(3,tt1:tt2),'k','linewidth',1)
hold on
plot3(uwb(1,tt1:20:tt2),uwb(2,tt1:20:tt2),uwb(3,tt1:20:tt2),'m+','linewidth',1)
hold on
plot3(mp(1,tt1:tt2),mp(2,tt1:tt2),mp(3,tt1:tt2),'b-.','linewidth',2)
hold on
plot3(mpa(1,tt1:tt2),mpa(2,tt1:tt2),mpa(3,tt1:tt2),'g--','linewidth',2)
hold on
plot3(mpfca(1,tt1:tt2),mpfca(2,tt1:tt2),mpfca(3,tt1:tt2),'r','linewidth',2)
xlabel('x(m)','FontSize',18)
ylabel('y(m)','FontSize',18)
zlabel('z(m)','FontSize',18)
% view([20 25 10])
legend('Reference','UWB','EKF','SHFAF','ACCAEKF');
grid


figure (2)
tt = 2000;
plot(ap(1,1:tt),ap(2,1:tt),'k','linewidth',2)
hold on
plot(uwb(1,1:20:tt),uwb(2,1:20:tt),'m+','linewidth',1)
hold on
plot(mp(1,1:tt),mp(2,1:tt),'b-.','linewidth',2)
hold on
plot(mpa(1,1:tt),mpa(2,1:tt),'g--','linewidth',2)
hold on
plot(mpfca(1,1:tt),mpfca(2,1:tt),'r:','linewidth',2)
xlabel('x(m)','FontSize',18)
ylabel('y(m)','FontSize',18)
legend('Reference','UWB','EKF','SHFAF','ACCAEKF');
grid

dekf = mp - ap;
err = sqrt(dekf(1,:).^2 + dekf(2,:).^2 + dekf(3,:).^2);
dekfa = mpa - ap;
erra = sqrt(dekfa(1,:).^2 + dekfa(2,:).^2 + dekfa(3,:).^2);
dekfca = mpfca - ap;
errfca = sqrt(dekfca(1,:).^2 + dekfca(2,:).^2 + dekfca(3,:).^2);
t = 100:20000;
figure (3)
plot(t,err(t),'b-.',t,erra(t),'g',t,errfca(t),'r--','linewidth',1)
xlabel('samples','FontSize',14)
ylabel('position error (m)','FontSize',14)
legend('EKF','SHFAF','ACCAEKF');


my_avg = [mean(err), mean(erra), mean(errfca)]
my_std = [std(err), std(erra), std(errfca)]


