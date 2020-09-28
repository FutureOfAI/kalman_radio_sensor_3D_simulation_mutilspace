close all; %close all figures
clear all;
clc;       %clear the command line
fclose('all'); %close all open files
delete(instrfindall); %Reset Com Port

obj = serial('com7');
obj.BaudRate = 57600;
obj.Parity = 'none';
obj.StopBits = 1;
count = 4000;
fopen(obj);
disp(obj);
data = zeros(32,count);
for i=1:count
    data(:,i) = fread(obj,32);
end

fclose(obj);
delete(obj);

save 2D_demo_data_180817.mat data;