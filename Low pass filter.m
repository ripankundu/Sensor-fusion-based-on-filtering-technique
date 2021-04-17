% Low pass filter for sensor fusion algorithm.m
% Author- Ripan Kumar Kundu
% MSc in Electrical Engineering
% University of Rostock


clear all; clc; close all;

data_ax=csvread('Accelerometer.csv',1,2);
data_ay=csvread('Accelerometer.csv',1,3);
data_az=csvread('Accelerometer.csv',1,4);
% accx=data(:,3);
% accy=data(:,4);
% accz=data(:,5);

data_gx=csvread('Gyroscope.csv',1,2);
data_gy=csvread('Gyroscope.csv',1,3);
data_gz=csvread('Gyroscope.csv',1,4);
% data2=xlsread('Gyroscope.csv');
% gyrox=data(:,1);
% gyroy=data(:,2);
% gyroz=data(:,3);

%calculate the Mean bias value%
meangyrox = sum(data_gx(1:200))/200;
meangyroy = sum(data_gy(1:200))/200;
meangyroz = sum(data_gz(1:200))/200;
% gyroy=data2-meangyroy;
gyroy=data_gy-meangyroy;
Ts=1/100;

%Calculate the angle from the accelerometer data%
for i=1:length(data_ax)
    angle_acc(i) = atan2(data_ax(i),sqrt((data_ay(i)*data_ay(i))+(data_az(i)*data_az(i))))*(180/pi);
end

%Filter coefficient%
%fc=3;
%fc is the cutoff frequency
%fc=1/(2*pi*RC);
%RC=1/(2*pi*fc);
%tau=RC;
%alpha_lp=Ts/(Ts + tau);
alpha_lp=0.02;

%pass the signal through the low pass filter%
angle_lpf(1)= angle_acc(1);
for i=2:length(data_ax)
    angle_lpf(i)= alpha_lp*angle_acc(i) + (1-alpha_lp)*angle_lpf(i-1);
end

%Ploting the angle derived from the gyroscope%
% plot (data_ax,'g-')
hold on; 
plot(angle_acc,'r-')
plot (angle_lpf,'b-')
legend('Angle using raw accelerometer','Angle passed through lpf');
xlabel('sample data')
ylabel('Angles')
title('jitter correction');
    








% %compute drift in a loop%
% angy(1)=0;
% for i=1:length(gyroy)
%     if i < length(gyroy)
%         angy(i+1) = angy(i) + gyroy(i+1)*Ts;% integrating the gyroscope
%     end
% end
% %plotting the angle value from gyroscope%
% plot(angy,'-');
% hold on;
% anggyr= angy;
% %return
% 
% %filter coefficient%
% %fc is basically cutoff frequency 
% % fc = 3;
% % %fc = 1/(2*pi*RC);
% % RC = 1/(2*pi*fc);
% % tau=RC;
% % alphp=tau/(tau+Ts)
% alphp=0.8414;