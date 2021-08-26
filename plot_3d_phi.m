close all;
clear all;
clc;

data = load('_3d_5mphi_80.mat');
psi10 = abs(data.LS_delta_P);
data = load('_3d_5mphi_70.mat');
psi20 = abs(data.LS_delta_P);
data = load('_3d_5mphi_60.mat');
psi30 = abs(data.LS_delta_P);
data = load('_3d_5mphi_50.mat');
psi40 = abs(data.LS_delta_P);
data = load('_3d_5mphi_40.mat');
psi50 = abs(data.LS_delta_P);
data = load('_3d_5mphi_30.mat');
psi60 = abs(data.LS_delta_P);
data = load('_3d_5mphi_20.mat');
psi70 = abs(data.LS_delta_P);
data = load('_3d_5mphi_10.mat');
psi80 = abs(data.LS_delta_P);
data = load('_3d_5mphi_0.mat');
psi90 = abs(data.LS_delta_P);
data = load('_3d_5mphi_n10.mat');
psi100 = abs(data.LS_delta_P);
data = load('_3d_5mphi_n20.mat');
psi110 = abs(data.LS_delta_P);
data = load('_3d_5mphi_n30.mat');
psi120 = abs(data.LS_delta_P);
data = load('_3d_5mphi_n40.mat');
psi130 = abs(data.LS_delta_P);
data = load('_3d_5mphi_n50.mat');
psi140 = abs(data.LS_delta_P);
data = load('_3d_5mphi_n60.mat');
psi150 = abs(data.LS_delta_P);
data = load('_3d_5mphi_n70.mat');
psi160 = abs(data.LS_delta_P);

psi(1,:) = sqrt(sum(psi10.^2));
psi(2,:) = sqrt(sum(psi20.^2));
psi(3,:) = sqrt(sum(psi30.^2));
psi(4,:) = sqrt(sum(psi40.^2));
psi(5,:) = sqrt(sum(psi50.^2));
psi(6,:) = sqrt(sum(psi60.^2));
psi(7,:) = sqrt(sum(psi70.^2));
psi(8,:) = sqrt(sum(psi80.^2));
psi(9,:) = sqrt(sum(psi90.^2));
psi(10,:) = sqrt(sum(psi100.^2));
psi(11,:) = sqrt(sum(psi110.^2));
psi(12,:) = sqrt(sum(psi120.^2));
psi(13,:) = sqrt(sum(psi130.^2));
psi(14,:) = sqrt(sum(psi140.^2));
psi(15,:) = sqrt(sum(psi150.^2));
psi(16,:) = sqrt(sum(psi160.^2));

m_psi = mean(psi,2);

n = length(psi(1,:));
for i=1:16
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
figure (1)
plot(t,a(1,:),'-o',t,a(2,:),'-+',t,a(3,:),'-.',t,a(4,:),t,a(5,:),'m',t,a(6,:),'k',t,a(7,:),'b',t,a(8,:),'g',t,a(9,:),'r','linewidth',2);
xlabel('positioning error (m)','FontSize',14)
ylabel('Probability','FontSize',14)
figure (2)
plot(t,a(16,:),'-o',t,a(15,:),'-+',t,a(14,:),'-.',t,a(13,:),'m',t,a(12,:),'k',t,a(11,:),'b',t,a(10,:),'g',t,a(9,:),'r','linewidth',2);
xlabel('positioning error (m)','FontSize',14)
ylabel('Probability','FontSize',14)

