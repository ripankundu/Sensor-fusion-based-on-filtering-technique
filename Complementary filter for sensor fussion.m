% Complementary filter filter for sensor fusion algorithm.m
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

data_gx=csvread('Gyroscope.csv',1,2)*(-180/pi);
data_gy=csvread('Gyroscope.csv',1,3)*(-180/pi);
data_gz=csvread('Gyroscope.csv',1,4)*(-180/pi);
% data2=xlsread('Gyroscope.csv');
% gyrox=data(:,1);
% gyroy=data(:,2);
% gyroz=data(:,3);

%calculate the Mean bias value%
meangyrox = sum(data_gx(1:200))/200;
meangyroy = sum(data_gy(1:200))/200;
meangyroz = sum(data_gz(1:200))/200;
% gyroy=data2-meangyroy;
gyroy=data_gy;%-meangyroy;
Ts=1/100;

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
%alphp=0.8414;


%Take the initial angle for gyroscope from the accelerometer
angle_acc(1)=atan2(data_ax(1),sqrt((data_ay(1)*data_ay(1))+(data_az(1)*data_az(1))))*(180/pi);
angle_gyro(1)=atan2(data_ax(1),sqrt((data_ay(1)*data_ay(1))+(data_az(1)*data_az(1))))*(180/pi);
angle_cf(1)=atan2(data_ax(1),sqrt((data_ay(1)*data_ay(1))+(data_az(1)*data_az(1))))*(180/pi);

%Complimentary filter without low passing and high passing filter
for i=2:length(data_ax)
    %if the compute angle from gyro alone
    angle_gyro(i)= angle_gyro(i-1) +(gyroy(i))*(Ts);
    angle_acc(i)=atan2(data_ax(i),sqrt((data_ay(i)*data_ay(i))+(data_az(i)*data_az(i))))*180/pi;
    angle_cf(i)= angle_cf(i-1) +(gyroy(i))*(Ts);
   
    
    %Now using the complimentary filter,we combine the both accelerometer
    %and gyro data
    angle_cf(i)=angle_cf(i)*0.98 + 0.02*angle_acc(i); % Equation of the complimentary filter
end

figure;
plot(angle_gyro)
hold on; plot(angle_acc,'g')
hold on; plot(angle_cf,'r')
legend('Angle of Gyro','Angle of acclerometer','CF with out the pre filtering')
xlabel('sample data')
ylabel('Angles')
title('Angle compute from CF');


%Low pass, high pass and then combine to the complimentary filter
% The gyroscope data passed to the high pass filter and acceleration data
% passed to the low pass filter the raw values%

alphp=0.8414;
alpha_lp=0.1586;

%Initialize the values%
accx_lp(1)=data_ax(1);
accy_lp(1)=data_ay(1);
accz_lp(1)=data_az(1);
lpf_x(1)=data_ax(1);
lpf_y(1)=data_ay(1);
lpf_z(1)=data_az(1);
angle_gyro(1)=angle_acc(1);
angle_acc_lp(1)=angle_acc(1);
angle_gyro_hp(1)=angle_acc(1);


for i=2:length(data_ax)
    
    
    angle_acc1(i)=atan2(data_ax(i),sqrt((data_ay(i)*data_ay(i))+(data_az(i)*data_az(i))))*180/pi;
    
    %The Low pass filter for accelerometer data
    lpf_x(i)=(1-alpha_lp)*lpf_x(i-1)+alpha_lp*data_ax(i);
    lpf_y(i)=(1-alpha_lp)*lpf_y(i-1)+alpha_lp*data_ay(i);
    lpf_z(i)=(1-alpha_lp)*lpf_z(i-1)+alpha_lp*data_az(i);

    
    %Now compute the angle from gyroscope data
    
    angle_gyro(i)= angle_gyro(i-1) +gyroy(i)*(Ts);
    
    
    %So apply the High pass filter then compute gyro angle
    angle_gyro_hp(i)= (alphp*angle_gyro_hp(i-1) + alphp*(angle_gyro(i)-angle_gyro(i-1)));
    
    
    %myself%
%     angle_cf_filter(i)= (alphp*angle_cf_filter(i-1) + alphp*(angle_gyro(i)-angle_gyro(i-1)));
   
    
    %combine the complimentary filter now
%     angle_cf_filter(i)=angle_cf_filter(i)*0.98+0.02*angle_acc1(i);
%     
    angle_cf_filter(i)=angle_gyro_hp(i)*0.98+0.02*angle_acc1(i);
end

% %angle_cf_filter Error comparision%
% maxerror_cf=max(angle_cf_filter);
% minerror_cf=min(angle_cf_filter);
% averageerorr_cf=mean(angle_cf_filter);
% standardev_cf=std(angle_cf_filter);
% 
% 
% %angle_gyro Error comparision%
% maxerror_gy=max(angle_gyro_hp);
% minerror_gy=min(angle_gyro_hp);
% averageerorr_gy=mean(angle_gyro_hp);
% standardev_gy=std(angle_gyro_hp);
% 
% 
% %angle_gyro Error comparision%
% maxerror_acc=max(angle_acc1);
% minerror_acc=min(angle_acc1);
% averageerorr_acc=mean(angle_acc1);
% standardev_acc=std(angle_acc1);


figure;
plot(angle_gyro_hp)
hold on; plot(angle_acc1,'g')
hold on; plot(angle_cf_filter,'r')
legend('Angle of Gyro','Angle of acclerometer','Coomplimentary filter with the pre filtering')
    
