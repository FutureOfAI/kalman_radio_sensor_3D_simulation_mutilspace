close all;
clear all;
clc

warning off
% load LOS dataset
data = load('RangingWithCIRLOS.mat');
data1 = data.RangingWithCIRLOS;
A1 = cell2table(struct2cell(data1));
B1 = table2array(A1);
range_los = double(cell2mat(B1(2,:)));
dist_los = double(cell2mat(B1(3,:)));
data_los = cell2mat(B1(1,:));
data_los(isinf(data_los)) = 0;

% load NLOS dataset
data = load('RangingWithCIRNLOS.mat');
data2 = data.RangingWithCIRNLOS;
A2 = cell2table(struct2cell(data2));
B2 = table2array(A2);
range_nlos = double(cell2mat(B2(2,:)));
dist_nlos = double(cell2mat(B2(3,:)));
data_nlos = cell2mat(B2(1,:));
data_nlos(isinf(data_nlos)) = 0;

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
data3 = [data_los,data_nlos];
for iwin=1:4796
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
%     feature_table.engValue(iwin, 1) = sum(current_signal(indx1:indx2));
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

%     [maxfreq, maxval, maxratio] = dominant_frequency_features(current_signal, fs, 256, 0);
%     feature_table.dominantFrequencyValue(iwin, 1) = maxfreq;
%     feature_table.dominantFrequencyMagnitude(iwin, 1) = maxval;
%     feature_table.dominantFrequencyRatio(iwin, 1) = maxratio;
%     % Extract Mel-frequency cepstral coefficients
%     Tw = window_length*1000;% analysis frame duration (ms)
%     Ts = 10;                % analysis frame shift (ms)
%     alpha = 0.97;           % preemphasis coefficient
%     M = 20;                 % number of filterbank channels 
%     C = 12;                 % number of cepstral coefficients
%     L = 22;                 % cepstral sine lifter parameter
%     LF = 5;                 % lower frequency limit (Hz)
%     HF = 500;               % upper frequency limit (Hz)
% 
%     [MFCCs, ~, ~] = mfcc(current_signal, fs, Tw, Ts, alpha, @hamming, [LF HF], M, C+1, L);
%     feature_table.MFCC1(iwin, 1) = MFCCs(1);
%     feature_table.MFCC2(iwin, 1) = MFCCs(2);
%     feature_table.MFCC3(iwin, 1) = MFCCs(3);
%     feature_table.MFCC4(iwin, 1) = MFCCs(4);
%     feature_table.MFCC5(iwin, 1) = MFCCs(5);
%     feature_table.MFCC6(iwin, 1) = MFCCs(6);
%     feature_table.MFCC7(iwin, 1) = MFCCs(7);
%     feature_table.MFCC8(iwin, 1) = MFCCs(8);
%     feature_table.MFCC9(iwin, 1) = MFCCs(9);
%     feature_table.MFCC10(iwin, 1) = MFCCs(10);
%     feature_table.MFCC11(iwin, 1) = MFCCs(11);
%     feature_table.MFCC12(iwin, 1) = MFCCs(12);
%     feature_table.MFCC13(iwin, 1) = MFCCs(13);
    % Assign class label to the observation
    if iwin<=3069
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

%%
[training_set, testing_set] = splitDataSets(feature_table, 0.3);
% training_set = feature_table;
% testing_set = feature_table;

% Assign higher cost for misclassification of abnormal heart sounds
C = [0, 19; 19, 0];        

% Create a random sub sample (to speed up training) of 1/4 of the training set
%subsample = randi([1 height(training_set)], round(height(training_set)/4), 1);
% OR train on the whole training set
subsample = 1:height(training_set);

% Create a 5-fold cross-validation set from training data
cvp = cvpartition(length(subsample),'KFold',5);

% Step 2: train the model with hyperparameter tuning (unless you simply
% load an existing pre-trained model)

if ~exist('TrainedSVMModel.mat')
    % bayesian optimization parameters (stop after 15 iterations)
    opts = struct('Optimizer','bayesopt','ShowPlots',true,'CVPartition',cvp,...
                'AcquisitionFunctionName','expected-improvement-plus','MaxObjectiveEvaluations',15); 
    rng(1);
    trained_model = fitcsvm(training_set(subsample,:),'class','KernelFunction','rbf','Cost',C,...
            'OptimizeHyperparameters','auto','HyperparameterOptimizationOptions',opts);
    save('TrainedSVMModel','trained_model');
else
    load('TrainedSVMModel.mat');
end

%% Step 3: evaluate accuracy on held-out test set

% Predict class labels for the validation set using trained model
% NOTE: if training ensemble without optimization, need to use trained_model.Trained{idx} to predict
predicted_class = predict(trained_model, testing_set);

conf_mat = confusionmat(testing_set.class, predicted_class);

conf_mat_per = conf_mat*100./sum(conf_mat, 2);

% Visualize model performance in heatmap
labels = {'Abnormal', 'Normal'};
heatmap(labels, labels, conf_mat_per, 'Colormap', winter, 'ColorbarVisible','off');

% %%
% runNCA = true;   % control whether to see NCA running or just load the selected features
%     
% if ~runNCA && exist('SelectedFeatures.mat')%#ok
%     % Load saved array of selected feature indexes
%     load('SelectedFeatures.mat')
% else % Perform feature selection with neighborhood component analysis
%     rng(1);
%     
%     mdl = fscnca(table2array(training_set(:,1:12)), ...
%         table2array(training_set(:,13)), 'Lambda', 0.005, 'Verbose', 0);
%     
%     % Select features with weight above 1
%     selected_feature_indx = find(mdl.FeatureWeights > 0.1);
% 
%     % Plot feature weights
%     stem(mdl.FeatureWeights,'bo');
%     
%     % save for future reference
%     save('SelectedFeatures', 'selected_feature_indx');
%     % Display list of selected features
% %     disp(feature_table.Properties.VariableNames(selected_feature_indx))  
% end
% 
% %
% trainReducedModel = false;    % control whether to re-train this model or load it from a previous run
% 
% if trainReducedModel | ~exist('TrainedEnsembleModel_FeatSel.mat')%#ok
%     % configure key parameters: cross validation, cost, and hyperparameter
%     % tuning
%     rng(1)
%     opts = struct('Optimizer','bayesopt','ShowPlots',true,'CVPartition',cvp,...
%             'AcquisitionFunctionName','expected-improvement-plus','MaxObjectiveEvaluations',10);    
%     
%     % now we are ready to train...
%     trained_model_featsel = fitcsvm(training_set(subsample,selected_feature_indx),training_set.class(subsample),'Cost',C,...
%         'OptimizeHyperparameters','auto', 'HyperparameterOptimizationOptions',opts)
%     % save the model for later reference
%     save('TrainedWaveletModel_FeatSel', 'trained_model_featsel');
% else
%     load('TrainedEnsembleModel_FeatSel.mat')
% end
% 
% % Predict class labels for the validation set using trained model
% predicted_class_featsel = predict(trained_model_featsel, testing_set(:,selected_feature_indx));
% 
% conf_mat_featsel = confusionmat(testing_set.class, predicted_class_featsel);
% 
% conf_mat_per_featsel = conf_mat_featsel*100./sum(conf_mat_featsel, 2);
% 
% labels = {'Abnormal', 'Normal'};
% 
% % Visualize model performance
% % probably custom AE version: heatmap(conf_mat_per_featsel, labels, labels, 1,'Colormap', 'red','ShowAllTicks',1,'UseLogColorMap',false,'Colorbar',true);
% heatmap(labels,labels,conf_mat_per_featsel, 'Colormap', winter, 'ColorbarVisible','off');
% 
