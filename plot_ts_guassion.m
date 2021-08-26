close all;
clear all;
clc;

n1 = 500;
n2 = 3900;

data = load('linewithGu_x.mat');
rwox = data.xpm_Nh;
data = load('linewithGu_y.mat');
rwoy = data.ypm_Nh;
data = load('linewithGu_z.mat');
rwoz = data.zpm_Nh;
data = load('linewithTS_x.mat');
rwx = data.xpm_Nh;
data = load('linewithTS_y.mat');
rwy = data.ypm_Nh;
data = load('linewithTS_z.mat');
rwz = data.zpm_Nh;
data = load('linewithSHFAF_x.mat');
rwshx = data.xpm_Nh;
data = load('linewithSHFAF_y.mat');
rwshy = data.ypm_Nh;
data = load('linewithSHFAF_z.mat');
rwshz = data.zpm_Nh;
data = load('lineRSS.mat');
rsst = data.rss;

rss(:,1) = rsst(:,1);
for i=1:10:7990
   for j=1:10
      rss(:,i+j) = rsst(:,fix(i/10)+1); 
   end
end

line = [2.6,-1.9,1.0
        2.6,5.9,1.0];

% nlos01 = find(rss(1,:)>mean(rss(1,:)));
% nlos02 = find(rss(2,:)>mean(rss(2,:)));
% nlos03 = find(rss(3,:)>mean(rss(3,:)));
% nlos04 = find(rss(4,:)>mean(rss(4,:)));

nlos = sqrt(rss(1,:).^2 + rss(2,:).^2 + rss(3,:).^2 + rss(4,:).^2);
nlos_inx1 = find(nlos>mean(nlos));
nlos_sort1 = find(nlos_inx1>500 & nlos_inx1 <3900);

nlos_inx2 = find(nlos<=mean(nlos));
nlos_sort2 = find(nlos_inx2>500 & nlos_inx2 <3900);

ttt = n1:1:n2;
figure(1)
% rwshx(ttt),rwshy(ttt),'m:', rwx(ttt),rwy(ttt)
plot(rwox(ttt),rwoy(ttt),'b--',rwshx(ttt),rwshy(ttt),'g','linewidth',2); 
hold on
plot(line(:,1),line(:,2),'k','linewidth',2);
% hold on
plot(rwx(ttt),rwy(ttt),'r-.','linewidth',2);
% plot(rwx(nlos_inx2(nlos_sort2)),rwy(nlos_inx2(nlos_sort2)),'r.',rwx(nlos_inx1(nlos_sort1)),rwy(nlos_inx1(nlos_sort1)),'y.','linewidth',2);
% plot(rect(:,1),rect(:,2),'k','linewidth',2);
% plot(static(1),static(2),'k^','linewidth',2);
% plot(tilt(:,1),tilt(:,2),'k-.','linewidth',3);
% plot(horizon(:,1),horizon(:,2),'k','linewidth',2);
xlabel('x (m)','FontSize',14)
ylabel('y (m)','FontSize',14)
% title('Horizontal rectangle motion','FontSize',18);
% legend('ACCAEKF without IFC','ACCAEKF with IFC','EKF','SHFAF','Reference','FontSize',10);
legend('ACCAEKF without NLOS SVM','SHFAF','Reference','ACCAEKF with NLOS SVM','FontSize',8);
% staic
% axis([2.3 2.6 2.6 2.85])
% tilt
% axis([1.9 2.9 2.6 3.2])
% rec
% axis([0 7 -1 5])
% horizon
% axis([2 4 2.5 3.3])
set(gca, 'FontSize', 14)
axis equal

figure (2)
plot(rwox(ttt),rwoy(ttt),'b--','linewidth',2); 
hold on
plot(rwx(nlos_inx2(nlos_sort2)),rwy(nlos_inx2(nlos_sort2)),'g*',rwx(nlos_inx1(nlos_sort1)),rwy(nlos_inx1(nlos_sort1)),'y*','linewidth',1);
xlabel('x (m)','FontSize',14)
ylabel('y (m)','FontSize',14)
legend('ACCAEKF without NLOS SVM','LOS','NLOS','FontSize',8);
set(gca, 'FontSize', 14)
axis equal


