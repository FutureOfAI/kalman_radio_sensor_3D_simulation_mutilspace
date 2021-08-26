close all;
clear all;
clc

data = load('RangingWithCIRLOS.mat');
data1 = data.RangingWithCIRLOS;
A1 = cell2table(struct2cell(data1));
B1 = table2array(A1);
range_los = double(cell2mat(B1(2,:)));
dist_los = double(cell2mat(B1(3,:)));
data_los = cell2mat(B1(1,:));
data_los(isinf(data_los)) = 0;
MAX_los = max(data_los);
VAR_los = var(data_los);

t = 1:1016;
sigma = 0.2;
for i=1:3069
    A = data_los(t,i);
    y = smooth(A)';
    indx1 = find(y==max(y(:)));
    indx_array = find(y<(sigma*max(y(:))+median(y)));
    indx2 = indx_array(find(indx1(1)<indx_array(:)));
    DTI_los(i) = indx2(1);
    dt = indx2(1) - indx1(1);
    dD = y(indx1(1)) - y(indx2(1));
    PDE_los(i) = dD/dt;
    ENG_los(i) = sum(y(indx1(1):indx2(1)));
    AVG_los(i) = mean(y(indx1(1):indx2(1)));
end

data = load('RangingWithCIRNLOS.mat');
data2 = data.RangingWithCIRNLOS;
A2 = cell2table(struct2cell(data2));
B2 = table2array(A2);
range_nlos = double(cell2mat(B2(2,:)));
dist_nlos = double(cell2mat(B2(3,:)));
data_nlos = cell2mat(B2(1,:));
data_nlos(isinf(data_nlos)) = 0;
% ENG_nlos = sum(data_nlos);
MAX_nlos = max(data_nlos);
VAR_nlos = var(data_nlos);

for i=1:1727
    A = data_nlos(t,i);
    y = smooth(A)';
    indx1 = find(y==max(y(:)));
    indx_array = find(y<(sigma*max(y(:))+median(y)));
    indx2 = indx_array(find(indx1(1)<indx_array(:)));
    DTI_nlos(i) = indx2(1);
    dt = indx2(1) - indx1(1);
    dD = y(indx1(1)) - y(indx2(1));
    PDE_nlos(i) = dD/dt;
    ENG_nlos(i) = sum(y(indx1(1):indx2(1)));
    AVG_nlos(i) = mean(y(indx1(1):indx2(1)));
end


 %% Put the data in one matrix, and make a vector of classifications
% t1 = 1:1000;
% t2 = 1:1000;
% data1 = [t1',ENG_los(t1)'];
% data2 = [t2',ENG_nlos(t2)'];
% data3 = [data1;data2];
% theclass = ones(2000,1);
% theclass(1:1000) = -1;

%% Train an SVM classifier with KernelFunction set to 'rbf' and BoxConstraint set to Inf
% %Train the SVM Classifier
% cl = fitcsvm(data3,theclass,'KernelFunction','rbf',...
%     'BoxConstraint',Inf,'ClassNames',[-1,1],'Cost',[0,2;1,0]);
% 
% % Predict scores over the grid
% d = 0.1;
% [x1Grid,x2Grid] = meshgrid(min(data3(:,1)):d:max(data3(:,1)),...
%     min(data3(:,2)):d:max(data3(:,2)));
% xGrid = [x1Grid(:),x2Grid(:)];
% [~,scores] = predict(cl,xGrid);
% 
% tt1 = 1000:1300;
% tt2 = 1000:1300;
% data1a = [tt1',ENG_los(tt1)'];
% data2a = [tt2',ENG_nlos(tt2)'];
% data3a = [data1a;data2a];
% newX = data3a;
% [label,score] = predict(cl,newX);
% % find the percentage
% los_percent = length(find(label(1:300)==-1));
% nlos_percent = length(find(label(300:600)==1));
% 
% % Plot the data and the decision boundary
% figure;
% h(1:2) = gscatter(data3(:,1),data3(:,2),theclass,'rb','.');
% hold on
% % ezpolar(@(x)1);
% % h(3) = plot(data3(cl.IsSupportVector,1),data3(cl.IsSupportVector,2),'ko');
% contour(x1Grid,x2Grid,reshape(scores(:,2),size(x1Grid)),[0 0],'k');
% legend(h,{'-1','+1'});
% % axis equal
% hold off
% xlabel('samples','FontSize',14)
% ylabel('ENG','FontSize',14)


%% plot feature
t3 = 1:3069;
t4 = 1:1727;
% t3 = range_los(t2);
% t4 = range_nlos(t2);
% t3 = dist_los(t2);
% t4 = dist_nlos(t2);
% t3 = abs(range_los(t2)-dist_los(t2));
% t4 = abs(range_nlos(t2)-dist_nlos(t2));
% figure;
% plot(t3,ENG_los,'r.','MarkerSize',15);
% hold on
% plot(t4,ENG_nlos,'b.','MarkerSize',15);
% legend('LOS','NLOS');
% xlabel('samples','FontSize',14)
% ylabel('ENG','FontSize',14)
% 
% figure;
% plot(t3,MAX_los,'r.','MarkerSize',15);
% hold on
% plot(t4,MAX_nlos,'b.','MarkerSize',15);
% legend('LOS','NLOS');
% xlabel('samples','FontSize',14)
% ylabel('MAX','FontSize',14)
% 
% 
% figure;
% plot(t3,DTI_los,'r.',t4,DTI_nlos,'b.','MarkerSize',15);
% legend('LOS','NLOS');
% xlabel('samples','FontSize',14)
% ylabel('DTI','FontSize',14)
% 
% figure;
% plot(t3,PDE_los,'r.',t4,PDE_nlos,'b.','MarkerSize',15);
% legend('LOS','NLOS');
% xlabel('samples','FontSize',14)
% ylabel('PDE','FontSize',14)
% 
% figure;
% plot(t3,VAR_los,'r.',t4,VAR_nlos,'b.','MarkerSize',15);
% legend('LOS','NLOS');
% xlabel('samples','FontSize',14)
% ylabel('variance','FontSize',14)
% 
% figure;
% plot(t3,AVG_los,'r.',t4,AVG_nlos,'b.','MarkerSize',15);
% legend('LOS','NLOS');
% xlabel('samples','FontSize',14)
% ylabel('Average','FontSize',14)

