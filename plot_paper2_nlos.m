close all;
clear all;
clc;

% n1 = fix(0.75*7990);
n1 = fix(0.45*7990);
% n1 = 1;
n2 = fix(0.74*7990);
% n2 = 7991;

static = [2.47,2.81,0.62];
horizon = [2.55,2.92,0.7
           3.55,2.92,0.7];
tilt = [2.0,2.75,0.1
        2.77,2.9,0.5];

rect = [1.6,0.2,0.62;
        1.6,3.8,0.62;
        4.2,3.8,0.62;
        4.2,0.2,0.62;
        1.6,0.2,0.62];

data = load('staticwithacc_nlos_x.mat');
rwx = data.xpm_Nh;
data = load('staticwithacc_nlos_y.mat');
rwy = data.ypm_Nh;
data = load('staticwithacc_nlos_z.mat');
rwz = data.zpm_Nh;
data = load('staticwithoutacc_nlos_x.mat');
rwox = data.xpm_Nh;
data = load('staticwithoutacc_nlos_y.mat');
rwoy = data.ypm_Nh;
data = load('staticwithoutacc_nlos_z.mat');
rwoz = data.zpm_Nh;
data = load('staticwith_nlos_fcacc_x.mat');
rwfax = data.xpm_Nh;
data = load('staticwith_nlos_fcacc_y.mat');
rwfay = data.ypm_Nh;
data = load('staticwith_nlos_fcacc_z.mat');
rwfaz = data.zpm_Nh;
data = load('staticwithSHFAF_nlos_x.mat');
rwshx = data.xpm_Nh;
data = load('staticwithSHFAF_nlos_y.mat');
rwshy = data.ypm_Nh;
data = load('staticwithSHFAF_nlos_z.mat');
rwshz = data.zpm_Nh;

ttt = n1:10:n2;
figure(1)
% rwshx(ttt),rwshy(ttt),'m:', rwx(ttt),rwy(ttt)
plot(rwox(ttt),rwoy(ttt),'g-.', rwfax(ttt),rwfay(ttt),'r--',rwx(ttt),rwy(ttt),'b',rwshx(ttt),rwshy(ttt),'m:','linewidth',2); 
hold on
% plot(rect(:,1),rect(:,2),'k','linewidth',2);
plot(static(1),static(2),'k^','linewidth',2);
% plot(tilt(:,1),tilt(:,2),'k-.','linewidth',3);
% plot(horizon(:,1),horizon(:,2),'k','linewidth',2);
xlabel('x (m)','FontSize',14)
ylabel('y (m)','FontSize',14)
% title('Horizontal rectangle motion','FontSize',18);
legend('ACCAEKF without IFC','ACCAEKF with IFC','EKF','SHFAF','Reference','FontSize',10);
% legend('EKF','SHFAF','ACCAEKF with IFC','Reference','FontSize',8);
% staic
% axis([2.3 2.6 2.6 2.85])
% tilt
% axis([1.9 2.9 2.6 3.2])
% rec
% axis([0 7 -1 5])
% horizon
% axis([2 4 2.5 3.3])
set(gca, 'FontSize', 14)

figure(5)
% plot3(rwox(ttt),rwoy(ttt),rwoz(ttt)+0.3,'g^',rwshx(ttt),rwshy(ttt),rwshz(ttt)+0.3,'b+',rwfax(ttt),rwfay(ttt),rwfaz(ttt)+0.3,'r*','linewidth',1); 
plot3(rwx(ttt),rwy(ttt),rwz(ttt),'g-.',rwfax(ttt),rwfay(ttt),rwfaz(ttt),'r--',rwox(ttt),rwoy(ttt),rwoz(ttt),'b',rwshx(ttt),rwshy(ttt),rwshz(ttt),'m:','linewidth',2); 
hold on
% plot3(rect(:,1),rect(:,2),rect(:,3),'k','linewidth',2);
% plot3(tilt(:,1),tilt(:,2),tilt(:,3),'k','linewidth',2);
plot3(static(1),static(2),static(3),'k^','linewidth',2);
xlabel('x (m)','FontSize',14)
ylabel('y (m)','FontSize',14)
zlabel('z (m)','FontSize',14)
% legend('EKF','SHFAF','ACCAEKF with IFC','Reference','FontSize',8);
legend('ACCAEKF without IFC','ACCAEKF with IFC','EKF','SHFAF','Reference','FontSize',8);
set(gca, 'FontSize', 14)

