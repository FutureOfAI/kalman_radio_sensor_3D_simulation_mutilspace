close all;
clear all;
clc

losdata = load('losdata.mat').losdata_v;
nlosboxdata = load('nlosboxdata.mat').nlosboxdata_v;
nlosdoordata = load('nlosdoordata.mat').nlosdoordata_v;
nloswalldata = load('nloswalldata.mat').nloswalldata_v;
nlosnewboxdata = load('nlosnewboxdata.mat').nlosnewboxdata_v;
nlostvdata = load('nlostvdata.mat').nlostvdata_v;
load('TrainedSVMModel.mat');

feature_table = table();
window_length =5;
window_overlap = 0;
overlap_length = window_length * window_overlap/100;
step_length = window_length - overlap_length;

feature_table_all = table();
labelMap = containers.Map('KeyType','int32','ValueType','char');
keySet = {-1, 1};
valueSet = {'Normal', 'Abnormal'};
labelMap = containers.Map(keySet, valueSet);

fs = 2;
sigma = 0.2;
winsize = 50;
data3 = [losdata,nlosdoordata,nloswalldata,nlosnewboxdata];
for iwin=1:120
    current_signal = data3(:,iwin);

    indx = find(current_signal == max(current_signal));
    if indx(1)>winsize
        indx1 = indx(1)-winsize;
    else
        indx1 = 1;
    end
    if indx(1) < 1016-winsize
        indx2 = indx(1)+winsize;
    else
        indx2 = 1016;
    end
%     indx_array = find(current_signal < (sigma*max(current_signal)+median(current_signal)));
%     indx2 = indx_array(find(indx1(1)<indx_array(:)));
%     DTI_los(iwin) = indx2(1);
%     dt = indx2(1) - indx1(1);
%     dD = current_signal(indx1(1)) - current_signal(indx2(1));    
    
%     feature_table.DTI(iwin, 1) = indx(1);
    feature_table.maxValue(iwin, 1) = max(current_signal(indx1:indx2));
%     feature_table.engValue(iwin, 1) = sum(current_signal(indx1(1):indx2(1)));
    feature_table.meanValue(iwin, 1) = mean(current_signal(indx1:indx2));
    feature_table.medianValue(iwin, 1) = median(current_signal(indx1:indx2));
    feature_table.standardValue(iwin, 1) = std(current_signal(indx1:indx2));
%     feature_table.meanAbsoluteDeviation(iwin, 1) = mad(current_signal(indx1:indx2));
%     feature_table.quantile25(iwin, 1) = quantile(current_signal, 0.25);
%     feature_table.quantile75(iwin, 1) = quantile(current_signal, 0.75);
%     feature_table.signalIQR(iwin, 1) = iqr(current_signal(indx1:indx2));
    feature_table.sampleSkewness(iwin, 1) = skewness(current_signal(indx1:indx2));
%     feature_table.sampleKurtosis(iwin, 1) = kurtosis(current_signal(indx1:indx2));
%     feature_table.signalEntropy(iwin, 1) = signal_entropy(current_signal(indx1:indx2)');
%     feature_table.spectralEntropy(iwin, 1) = spectral_entropy(current_signal(indx1:indx2), fs, 256);
    % Assign class label to the observation
    if iwin<=30
        current_class = -1;
    else
        current_class = 1;
    end    
    if iwin == 1
        feature_table.class = {labelMap(current_class)};
    else
        feature_table.class{iwin, :} = labelMap(current_class);
    end    
end

% %%
% [training_set, testing_set] = splitDataSets(feature_table, 0.3);
% 
% % Assign higher cost for misclassification of abnormal heart sounds
% C = [0, 19; 19, 0];        
% 
% % Create a random sub sample (to speed up training) of 1/4 of the training set
% %subsample = randi([1 height(training_set)], round(height(training_set)/4), 1);
% % OR train on the whole training set
% subsample = 1:height(training_set);
% 
% % Create a 5-fold cross-validation set from training data
% cvp = cvpartition(length(subsample),'KFold',5);
% 
% % Step 2: train the model with hyperparameter tuning (unless you simply
% % load an existing pre-trained model)
% 
% if ~exist('myTrainedSVMModel.mat')
%     % bayesian optimization parameters (stop after 15 iterations)
%     opts = struct('Optimizer','bayesopt','ShowPlots',true,'CVPartition',cvp,...
%                 'AcquisitionFunctionName','expected-improvement-plus','MaxObjectiveEvaluations',15); 
%     rng(1);
%     trained_model = fitcsvm(training_set(subsample,:),'class','KernelFunction','rbf','Cost',C,...
%             'OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',opts);
%     save('myTrainedSVMModel','trained_model');
% else
%     load('myTrainedSVMModel.mat');
% end

%% Step 3: evaluate accuracy on held-out test set

% Predict class labels for the validation set using trained model
% NOTE: if training ensemble without optimization, need to use trained_model.Trained{idx} to predict
predicted_class = predict(trained_model, feature_table);

conf_mat = confusionmat(feature_table.class, predicted_class);

conf_mat_per = conf_mat*100./sum(conf_mat, 2);

% Visualize model performance in heatmap
labels = {'Abnormal', 'Normal'};
heatmap(labels, labels, conf_mat_per, 'Colormap', winter, 'ColorbarVisible','off');

