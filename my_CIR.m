close all;
clear all;
clc

filename = 'nlos30.txt';
q = quantizer('fixed', 'nearest', 'saturate', [10 0]);% quantizer object for num2hex function  
FID = fopen(filename);
dataFromfile = textscan(FID, '%s');% %s for reading string values (hexadecimal numbers)
dataFromfile = dataFromfile{1};
decData = hex2num(q, dataFromfile);
decData = cell2mat(decData);
fclose(FID);

testData = zeros(1,200);
newData = [];

for i = 1:(length(decData)-202)
   if  decData(i) == 18 && decData(i+202) == 128
       testData = decData(i+1:i+200);
       newData = [newData;testData];
   end
end

realData = zeros(1,1050);
imgData = zeros(1,1050);
j = 1;
for i = 1:4:(length(newData)-3)
    realData(j) = newData(i+1) + bitshift(newData(i),8);
    imgData(j) = newData(i+3) + bitshift(newData(i+2),8);
    j = j + 1;
end

for k = 1:length(realData)        
    if realData(k) >= 2^15
        realData(k) = realData(k) - 2^16;
    end
    if imgData(k) >= 2^15
        imgData(k) = imgData(k) - 2^16;
    end 
end

for i = 1:length(realData)
%     cirData(i) = sqrt(realData(i)^2 + imgData(i)^2);
    cirData(i) = 10*log10(abs(realData(i).^2));
end

% save('nlosCIRtv30', 'cirData');

