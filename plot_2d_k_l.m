close all;
clear all;
clc;

data = load('_3d_k_10.mat');
k10 = abs(data.LS_delta_P);
data = load('_3d_k_20.mat');
k20 = abs(data.LS_delta_P);
data = load('_3d_k_30.mat');
k30 = abs(data.LS_delta_P);
data = load('_3d_k_40.mat');
k40 = abs(data.LS_delta_P);
data = load('_3d_k_50.mat');
k50 = abs(data.LS_delta_P);
data = load('_3d_l_10.mat');
l10 = abs(data.LS_delta_P);
data = load('_3d_l_20.mat');
l20 = abs(data.LS_delta_P);
data = load('_3d_l_30.mat');
l30 = abs(data.LS_delta_P);
data = load('_3d_l_40.mat');
l40 = abs(data.LS_delta_P);
data = load('_3d_m_10.mat');
m10 = abs(data.LS_delta_P);
data = load('_3d_m_20.mat');
m20 = abs(data.LS_delta_P);
data = load('_3d_m_30.mat');
m30 = abs(data.LS_delta_P);
data = load('_3d_m_40.mat');
m40 = abs(data.LS_delta_P);

psi(1,:) = sqrt(sum(k10.^2));
psi(2,:) = sqrt(sum(k20.^2));
psi(3,:) = sqrt(sum(k30.^2));
psi(4,:) = sqrt(sum(k40.^2));
psi(5,:) = sqrt(sum(k50.^2));
psi(6,:) = sqrt(sum(l10.^2));
psi(7,:) = sqrt(sum(l20.^2));
psi(8,:) = sqrt(sum(l30.^2));
psi(9,:) = sqrt(sum(l40.^2));
psi(10,:) = sqrt(sum(m10.^2));
psi(11,:) = sqrt(sum(m20.^2));
psi(12,:) = sqrt(sum(m30.^2));
psi(13,:) = sqrt(sum(m40.^2));

n = length(psi(1,:));
for i=1:13
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
plot(t,a(1,:),'c-o',t,a(2,:),'k:',t,a(3,:),'b-.',t,a(4,:),'g--',t,a(5,:),'r-','linewidth',2);
xlabel('position error (m)','FontSize',14)
ylabel('Probability','FontSize',14)
figure (2)
plot(t,a(6,:),'c-o',t,a(7,:),'k:',t,a(8,:),'b-.',t,a(9,:),'g--',t,a(5,:),'r-','linewidth',2);
xlabel('position error (m)','FontSize',14)
ylabel('Probability','FontSize',14)
figure (3)
plot(t,a(10,:),'c-o',t,a(11,:),'k:',t,a(12,:),'b-.',t,a(13,:),'g--',t,a(5,:),'r-','linewidth',2);
xlabel('position error (m)','FontSize',14)
ylabel('Probability','FontSize',14)

