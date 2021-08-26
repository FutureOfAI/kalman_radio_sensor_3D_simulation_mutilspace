%% 
% program name uwb_distribution_2D.m
% evaluate the impact of uwb errors in 2D positioning
% 2021-01-23
close all;
clear all;
clc;

dt = 0.1;
fz = 1*0.005;
T = 4*(1/fz);
t0 = 0:dt:T;
t0_1 = t0';
n = length(t0);

t0 = normrnd(0,0.1,1,n);
t1 = normrnd(0,0.1,1,n);

lambda = 3;
sigma =3;
mu = 0.1;
d  = lambda / sqrt(1 + lambda*lambda);

for i=1:n
    y(i)  = mu + sigma * (d * abs(t0(i)) + t1(i) * sqrt(1 - d * d));
end

% plot noise
figure (1)
plot(y(1:8000),'linewidth',1)
xlabel('samples','FontSize',14)
ylabel('Skew t-distribution noise (m)','FontSize',14)
