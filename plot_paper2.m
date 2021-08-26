close all;
clear all;
clc;

% n1 = fix(0.75*7990);
n1 = fix(0.5*7990);
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


data = load('staticwithacc_x.mat');
rwx = data.xpm_Nh;
data = load('staticwithacc_y.mat');
rwy = data.ypm_Nh;
data = load('staticwithacc_z.mat');
rwz = data.zpm_Nh;
data = load('staticwithoutacc_x.mat');
rwox = data.xpm_Nh;
data = load('staticwithoutacc_y.mat');
rwoy = data.ypm_Nh;
data = load('staticwithoutacc_z.mat');
rwoz = data.zpm_Nh;
data = load('staticwithfcacc_x.mat');
rwfax = data.xpm_Nh;
data = load('staticwithfcacc_y.mat');
rwfay = data.ypm_Nh;
data = load('staticwithfcacc_z.mat');
rwfaz = data.zpm_Nh;
data = load('staticwithSHFAF_x.mat');
rwshx = data.xpm_Nh;
data = load('staticwithSHFAF_y.mat');
rwshy = data.ypm_Nh;
data = load('staticwithSHFAF_z.mat');
rwshz = data.zpm_Nh;

ttt = n1:1:n2;
figure(1)
% rwshx(ttt),rwshy(ttt),'m:', rwx(ttt),rwy(ttt)
plot(rwx(ttt),rwy(ttt),'g-.',rwshx(ttt),rwshy(ttt),'m:',rwfax(ttt),rwfay(ttt),'r--',rwox(ttt),rwoy(ttt),'b','linewidth',2); 
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
axis([2.3 2.6 2.6 2.85])
% tilt
% axis([1.9 2.9 2.6 3.2])
% rec
% axis([0 7 -1 5])
% horizon
% axis([2 4 2.5 3.3])
set(gca, 'FontSize', 14)

figure(5)
% plot3(rwox(ttt),rwoy(ttt),rwoz(ttt),'g-.',rwshx(ttt),rwshy(ttt),rwshz(ttt),'m:',rwfax(ttt),rwfay(ttt),rwfaz(ttt),'r--','linewidth',2); 
plot3(rwx(ttt),rwy(ttt),rwz(ttt),'g-.',rwfax(ttt),rwfay(ttt),rwfaz(ttt),'r--',rwox(ttt),rwoy(ttt),rwoz(ttt),'b',rwshx(ttt),rwshy(ttt),rwshz(ttt),'m:','linewidth',2); 
hold on
% plot3(rect(:,1),rect(:,2),rect(:,3),'k','linewidth',2);
% plot3(tilt(:,1),tilt(:,2),tilt(:,3),'k','linewidth',3);
plot3(static(1),static(2),static(3),'k^','linewidth',2);
xlabel('x (m)','FontSize',14)
ylabel('y (m)','FontSize',14)
zlabel('z (m)','FontSize',14)
% legend('EKF','SHFAF','ACCAEKF with IFC','Reference','FontSize',8);
legend('ACCAEKF without IFC','ACCAEKF with IFC','EKF','SHFAF','Reference','FontSize',8);
set(gca, 'FontSize', 14)


figure (2)
tt = n1:n2;
% tt = 1:1918;
% plot(tt,0.7*ones(1,1918),'k','linewidth',1); % 0.62
% hold on
plot(tt,rwz(tt),'g-.',tt,rwfaz(tt),'r--',tt,rwoz(tt),'b','linewidth',2);
xlabel('samples','FontSize',14)
ylabel('z(m)','FontSize',14)
% legend('Reference','EKF','ACCAEKF','FCACCAEKF');
legend('ACCAEKF','FCACCAEKF','EKF');

figure (3)
t = tt;
plot(t,abs(rwz(t)-0.62),'g-.',t,abs(rwfaz(t)-0.62),'r--',t,abs(rwoz(t)-0.62),'b','linewidth',2);
xlabel('time(m)','FontSize',14)
ylabel('z error (m)','FontSize',14)
legend('ACCAEKF','FCACCAEKF','EKF');

% my_mean = [mean(abs(rwoz(t)-0.62)), mean(abs(rwz(t)-0.62)), mean(abs(rwfaz(t)-0.62))]
% my_std = [std(abs(rwoz(t)-0.62)), std(abs(rwz(t)-0.62)), std(abs(rwfaz(t)-0.62))]



dekf(1,:) = rwox - static(1);
dekf(2,:) = rwoy - static(2);
dekf(3,:) = rwoz - static(3);
err = sqrt(dekf(1,:).^2 + dekf(2,:).^2 + dekf(3,:).^2);
dekfa(1,:) = rwx - static(1);
dekfa(2,:) = rwy - static(2);
dekfa(3,:) = rwz - static(3);
erra = sqrt(dekfa(1,:).^2 + dekfa(2,:).^2 + dekfa(3,:).^2);
dekfca(1,:) = rwfax - static(1);
dekfca(2,:) = rwfay - static(2);
dekfca(3,:) = rwfaz - static(3);
errfca = sqrt(dekfca(1,:).^2 + dekfca(2,:).^2 + dekfca(3,:).^2);
% deksh(1,:) = rwshx - static(1);
% deksh(2,:) = rwshy - static(2);
% deksh(3,:) = rwshz - static(3);
% errsh = sqrt(deksh(1,:).^2 + deksh(2,:).^2 + deksh(3,:).^2);
figure (4)
t1 = tt;
plot(t1,err(t1),'b',t1,erra(t1),'g-.',t1,errfca(t1),'r--','linewidth',2) % t1,errsh(t1),'m:',
xlabel('time(s)','FontSize',14)
ylabel('position error (m)','FontSize',14)
% legend('EKF','ACCAEKF without IFC','ACCAEKF with IFC','SHFAF','FontSize',10);
% set(gca, 'XTick', 4000:1000:8000, 'FontSize', 14)
% set(gca, 'YTick', 0:0.04:0.2, 'FontSize', 14)


% my_mean = [mean(err(t1)), mean(erra(t1)), mean(errfca(t1)), mean(errsh(t1))]
% my_std = [std(err(t1)), std(erra(t1)), std(errfca(t1)), std(errsh(t1))]

% figure(5)
% plot3(rwx(n1:n2),rwy(n1:n2),rwz(n1:n2),'r',rwox(n1:n2),rwoy(n1:n2),rwoz(n1:n2),'g','linewidth',1);
% hold on
% plot3(tilt(:,1),tilt(:,2),tilt(:,3),'k','linewidth',2);
% xlabel('x(m)','FontSize',14)
% ylabel('y(m)','FontSize',14)
% zlabel('z(m)','FontSize',14)
% title('Tilt straight motion','FontSize',18);
% legend('','','Reference');
% % tilt
% axis([2 3.5 2 3.5 0 0.5])
% 
