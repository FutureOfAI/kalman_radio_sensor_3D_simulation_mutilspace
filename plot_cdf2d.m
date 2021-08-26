close all;
clear all;
clc;

data = load('_3d_distr_01.mat');
dist01 = abs(data.LS_delta_P);
data = load('_3d_distr_02.mat');
dist02 = abs(data.LS_delta_P);
data = load('_3d_distr_03.mat');
dist03 = abs(data.LS_delta_P);

psi(1,:) = sqrt(sum(dist01.^2));
psi(2,:) = sqrt(sum(dist02.^2));
psi(3,:) = sqrt(sum(dist03.^2));

n = length(psi(1,:));
for i=1:3
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
plot(t,a(3,:),'b--',t,a(2,:),'g-.',t,a(1,:),'r','linewidth',2);
xlabel('positioning error (m)','FontSize',14)
ylabel('Probability','FontSize',14)
legend('c','b','a');


