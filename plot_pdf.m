close all;
clear all;
clc;

data = load('_2d_20mpsi_5.mat');
psi05 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_10.mat');
psi10 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_20.mat');
psi20 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_30.mat');
psi30 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_40.mat');
psi40 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_50.mat');
psi50 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_60.mat');
psi60 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_70.mat');
psi70 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_80.mat');
psi80 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_90.mat');
psi90 = abs(data.LS_delta_P);

data = load('_2d_20mpsi_100.mat');
psi100 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_110.mat');
psi110 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_120.mat');
psi120 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_130.mat');
psi130 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_140.mat');
psi140 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_150.mat');
psi150 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_160.mat');
psi160 = abs(data.LS_delta_P);
data = load('_2d_20mpsi_170.mat');
psi170 = abs(data.LS_delta_P);

psi(1,:) = sqrt(sum(psi05.^2));
psi(2,:) = sqrt(sum(psi10.^2));
psi(3,:) = sqrt(sum(psi20.^2));
psi(4,:) = sqrt(sum(psi30.^2));
psi(5,:) = sqrt(sum(psi40.^2));
psi(6,:) = sqrt(sum(psi50.^2));
psi(7,:) = sqrt(sum(psi60.^2));
psi(8,:) = sqrt(sum(psi70.^2));
psi(9,:) = sqrt(sum(psi80.^2));
psi(10,:) = sqrt(sum(psi90.^2));

psi(11,:) = sqrt(sum(psi100.^2));
psi(12,:) = sqrt(sum(psi110.^2));
psi(13,:) = sqrt(sum(psi120.^2));
psi(14,:) = sqrt(sum(psi130.^2));
psi(15,:) = sqrt(sum(psi140.^2));
psi(16,:) = sqrt(sum(psi150.^2));
psi(17,:) = sqrt(sum(psi160.^2));
psi(18,:) = sqrt(sum(psi170.^2));

n = length(psi(1,:));
for i=1:18
    a(i,1)=length(find(psi(i,:)<=0.05))/n; % <0.1
    a(i,2)=length(find(psi(i,:)<=0.1))/n; % <0.2
    a(i,3)=length(find(psi(i,:)<=0.2))/n; % <0.3
    a(i,4)=length(find(psi(i,:)<=0.3))/n; % <0.4
    a(i,5)=length(find(psi(i,:)<=0.4))/n; % <0.5
    a(i,6)=length(find(psi(i,:)<=0.6))/n; % <0.6
    a(i,7)=length(find(psi(i,:)<=0.8))/n; % <0.7
    a(i,8)=length(find(psi(i,:)<=1.0))/n; % <0.8
    a(i,9)=length(find(psi(i,:)<=1.3))/n; % <0.9
    a(i,10)=length(find(psi(i,:)<=1.5))/n; % <1.0
end

t = [0.05, 0.1, 0.2, 0.3, 0.4, 0.6, 0.8, 1.0, 1.3, 1.5];
% figure (1)
% plot(t,a(2,:),'c-o',t,a(3,:),'k:',t,a(4,:),'b-.',t,a(5,:),'g--',t,a(9,:),'r-','linewidth',2);
% xlabel('positioning error (m)','FontSize',14)
% ylabel('Probability','FontSize',14)
% figure (2)
% plot(t,a(17,:),'c-o',t,a(16,:),'k:',t,a(15,:),'b-.',t,a(13,:),'g--',t,a(10,:),'r-','linewidth',2);
% xlabel('positioning error (m)','FontSize',14)
% ylabel('Probability','FontSize',14)

figure (1)
plot(t,a(2,:),'-o',t,a(3,:),'-+',t,a(4,:),'-.',t,a(5,:),t,a(6,:),'m',t,a(7,:),'k',t,a(8,:),'b',t,a(9,:),'g',t,a(10,:),'r','linewidth',2);
xlabel('positioning error (m)','FontSize',14)
ylabel('Probability','FontSize',14)
figure (2)
plot(t,a(17,:),'-o',t,a(16,:),'-+',t,a(15,:),'-.',t,a(14,:),'m',t,a(13,:),'k',t,a(12,:),'b',t,a(11,:),'g',t,a(10,:),'r','linewidth',2);
xlabel('positioning error (m)','FontSize',14)
ylabel('Probability','FontSize',14)



for i=1:18
    b(i,1)=length(find(psi(i,:)<=0.05))/n; % <0.1
    b(i,2)=length(find(psi(i,:)<=0.1))/n; % <0.2
    b(i,3)=length(find(psi(i,:)<=0.15))/n; % <0.3
    b(i,4)=length(find(psi(i,:)<=0.2))/n; % <0.4
    b(i,5)=length(find(psi(i,:)<=0.25))/n; % <0.5
    b(i,6)=length(find(psi(i,:)<=0.3))/n; % <0.6
    b(i,7)=length(find(psi(i,:)<=0.35))/n; % <0.7
    b(i,8)=length(find(psi(i,:)<=0.4))/n; % <0.8
    b(i,9)=length(find(psi(i,:)<=0.45))/n; % <0.9
    b(i,10)=length(find(psi(i,:)<=0.5))/n; % <1.0
end

t2 = [0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5];
figure (3)
plot(t2,b(5,:),t2,b(6,:),t2,b(7,:),t2,b(8,:),'b',t2,b(9,:),'g',t2,b(10,:),'r','linewidth',2);
xlabel('positioning error (m)','FontSize',14)
ylabel('Probability','FontSize',14)
figure (4)
plot(t2,b(14,:),t2,b(13,:),t2,b(12,:),'b',t2,b(11,:),'g',t2,b(10,:),'r','linewidth',2);
xlabel('positioning error (m)','FontSize',14)
ylabel('Probability','FontSize',14)

