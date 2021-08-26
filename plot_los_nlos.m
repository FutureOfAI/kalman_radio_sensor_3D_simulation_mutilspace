close all;
clear all;
clc;

data = load('anchor1_los_distance.mat');
los01 = data.an1_los;
data = load('anchor2_los_distance.mat');
los02 = data.an2_los;
data = load('anchor3_los_distance.mat');
los03 = data.an3_los;
data = load('anchor4_los_distance.mat');
los04 = data.an4_los;
data = load('anchor1_nlos_distance.mat');
nlos01 = data.an1_nlos;
data = load('anchor2_nlos_distance.mat');
nlos02 = data.an2_nlos;
data = load('anchor3_nlos_distance.mat');
nlos03 = data.an3_nlos;
data = load('anchor4_nlos_distance.mat');
nlos04 = data.an4_nlos;

% los_nlos01 = [los01, nlos01+0.6];
% los_nlos02 = [los02, nlos02+0.75];
% los_nlos03 = [los03, nlos03+0.38];
% los_nlos04 = [los04, nlos04+0.4];

t = 1:800;

figure (1)
subplot(411)
plot(t,los01,'linewidth',2)
% ylabel('r_1 (m)','FontSize',12)
axis([0 800 5.5 6.1])
set(gca, 'XTick', 0:200:800, 'FontSize', 12)
subplot(412)
plot(t,los02,'linewidth',2)
% ylabel('r_2 (m)','FontSize',12)
axis([0 800 5.1 5.7])
set(gca, 'XTick', 0:200:800, 'FontSize', 12)
subplot(413)
plot(t,los03,'linewidth',2)
% ylabel('r_3 (m)','FontSize',12)
axis([0 800 3.3 3.9])
set(gca, 'XTick', 0:200:800, 'FontSize', 12)
subplot(414)
plot(t,los04,'linewidth',2)
% ylabel('r_4 (m)','FontSize',12)
axis([0 800 3.7 4.3])
xlabel('samples','FontSize',12)
set(gca, 'XTick', 0:200:800, 'FontSize', 12)

figure (2)
subplot(411)
plot(t,nlos01+0.6,'linewidth',2)
% ylabel('r_1 (m)','FontSize',12)
axis([0 800 5.5 6.1])
set(gca, 'XTick', 0:200:800, 'FontSize', 12)
subplot(412)
plot(t,nlos02+0.75,'linewidth',2)
% ylabel('r_2 (m)','FontSize',12)
axis([0 800 5.1 5.7])
set(gca, 'XTick', 0:200:800, 'FontSize', 12)
subplot(413)
plot(t,nlos03+0.38,'linewidth',2)
% ylabel('r_3 (m)','FontSize',12)
axis([0 800 3.3 3.9])
set(gca, 'XTick', 0:200:800, 'FontSize', 12)
subplot(414)
plot(t,nlos04+0.4,'linewidth',2)
% ylabel('r_4 (m)','FontSize',12)
axis([0 800 3.7 4.3])
set(gca, 'XTick', 0:200:800, 'FontSize', 12)
xlabel('samples','FontSize',12)

figure (3)
subplot(411)
plot(t,los01 - mean(los01))
% ylabel('r_1 (m)','FontSize',12)
axis([0 800 -0.1 0.4])
subplot(412)
plot(t,los02 - mean(los02))
% ylabel('r_2 (m)','FontSize',12)
axis([0 800 -0.1 0.4])
subplot(413)
plot(t,los03 - mean(los03))
% ylabel('r_3 (m)','FontSize',12)
axis([0 800 -0.1 0.4])
subplot(414)
plot(t,los04 - mean(los04))
% ylabel('r_4 (m)','FontSize',12)
axis([0 800 -0.1 0.4])
xlabel('samples','FontSize',12)

figure (4)
subplot(411)
plot(t,nlos01 - mean(nlos01))
% ylabel('r_1 (m)','FontSize',12)
axis([0 800 -0.1 0.4])
subplot(412)
plot(t,nlos02 - mean(nlos02))
% ylabel('r_2 (m)','FontSize',12)
axis([0 800 -0.1 0.4])
subplot(413)
plot(t,nlos03 - mean(nlos03))
% ylabel('r_3 (m)','FontSize',12)
axis([0 800 -0.1 0.4])
subplot(414)
plot(t,nlos04 - mean(nlos04))
% ylabel('r_4 (m)','FontSize',12)
axis([0 800 -0.1 0.4])
xlabel('samples','FontSize',12)




