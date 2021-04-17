% Complementary with out pre filter data for sensor fusion algorithm.m
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


for i=2:length(data_gx)
    
    
    angle_acc1(i)=atan2(data_ax(i),sqrt((data_ay(i)*data_ay(i))+(data_az(i)*data_az(i))))*180/pi;
    
    %The Low pass filter for accelerometer data
    lpf_x(i)=(1-alpha_lp)*lpf_x(i-1)+alpha_lp*data_ax(i);
    lpf_y(i)=(1-alpha_lp)*lpf_y(i-1)+alpha_lp*data_ay(i);
    lpf_z(i)=(1-alpha_lp)*lpf_z(i-1)+alpha_lp*data_az(i);

    
    %Now compute the angle from gyroscope data
    
    angle_gyro(i)= angle_gyro(i-1) +(gyroy(i))*(Ts);
    
    
    %So apply the High pass filter then compute gyro angle
    angle_gyro_hp(i)= (alphp*angle_gyro_hp(i-1) + alphp*(angle_gyro(i)-angle_gyro(i-1)));
    
    
    %combine the complimentary filter now
    angle_cf_filter=angle_gyro_hp(i)*0.02+0.98*angle_acc1(i);
end

figure;
plot(angle_gyro_hp)
hold on; plot(angle_acc1,'g')
hold on; plot(angle_cf_filter,'r')
legend('Angle of Gyro','Angle of acclerometer','Coomplimentary filter with the pre filtering')
    
