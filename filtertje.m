tic
desiredfreq = 10;
% samplingrate = 20;
samplingrate = 1000/mean(tijdstap);

norm = desiredfreq/samplingrate;

d1 = designfilt('lowpassiir','FilterOrder',12, ...
    'HalfPowerFrequency',norm,'DesignMethod','butter');

M(:,6) = filtfilt(d1,M(:,6));
M(:,7) = filtfilt(d1,M(:,7));
M(:,8) = filtfilt(d1,M(:,8));
M(:,9) = filtfilt(d1,M(:,9));
M(:,10) = filtfilt(d1,M(:,10));
M(:,11) = filtfilt(d1,M(:,11));
M(:,12) = filtfilt(d1,M(:,12));
M(:,13) = filtfilt(d1,M(:,13));
M(:,14) = filtfilt(d1,M(:,14));
M(:,15) = filtfilt(d1,M(:,15));

M(:,21) = filtfilt(d1,M(:,21));
M(:,22) = filtfilt(d1,M(:,22));
M(:,23) = filtfilt(d1,M(:,23));
M(:,24) = filtfilt(d1,M(:,24));
M(:,25) = filtfilt(d1,M(:,25));
M(:,26) = filtfilt(d1,M(:,26));
M(:,27) = filtfilt(d1,M(:,27));
M(:,28) = filtfilt(d1,M(:,28));
M(:,29) = filtfilt(d1,M(:,29));
M(:,30) = filtfilt(d1,M(:,30));
% M(:,31) = filtfilt(d1,M(:,31));
% M(:,32) = filtfilt(d1,M(:,32));
% M(:,33) = filtfilt(d1,M(:,33));
% M(:,34) = filtfilt(d1,M(:,34));

toc