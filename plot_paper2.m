close all;
clear all;
clc;

n1 = 0.5*7990;
n2 = fix(0.74*7990);

staic = [2.47,2.81,0.62];
horizon = [2.55,2.81,0.7
           3.55,2.81,0.7];
tilt = [2.2,2.75,0.1
        2.65,2.95,0.7];

rect = [1.6,0.2,0.62;
        1.6,4.2,0.62;
        4.4,4.2,0.62;
        4.4,0.2,0.62;
        1.6,0.2,0.62];


data = load('recwithacc_x.mat');
rwx = data.xpm_Nh;
data = load('recwithacc_y.mat');
rwy = data.ypm_Nh;
data = load('recwithacc_z.mat');
rwz = data.zpm_Nh;
data = load('recwithoutacc_x.mat');
rwox = data.xpm_Nh;
data = load('recwithoutacc_y.mat');
rwoy = data.ypm_Nh;
data = load('recwithoutacc_z.mat');
rwoz = data.zpm_Nh;

figure(1)
plot(rwx(n1:n2),rwy(n1:n2),'r',rwox(n1:n2),rwoy(n1:n2),'g',rect(:,1),rect(:,2),'k--','linewidth',1);
xlabel('x(m)','FontSize',14)
ylabel('y(m)','FontSize',14)
title('Horizontal rectangle motion','FontSize',18);
legend('','','Reference');
% staic
% axis([2 3 2.5 3])
% horizon
% axis([2 4 2 4])
% rec
axis([0 7 -1 5])

figure(2)
plot3(rwx(n1:n2),rwy(n1:n2),rwz(n1:n2),'r',rwox(n1:n2),rwoy(n1:n2),rwoz(n1:n2),'g','linewidth',1);
hold on
plot3(rect(:,1),rect(:,2),rect(:,3),'k--','linewidth',2);
xlabel('x(m)','FontSize',14)
ylabel('y(m)','FontSize',14)
zlabel('z(m)','FontSize',14)
title('Tilt straight motion','FontSize',18);
legend('','','Reference');
% tilt
% axis([2 3.5 2 3.5 0 0.5])

