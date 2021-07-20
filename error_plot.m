error_data=load('D:\StudySource\惯性导航原理\作业3-惯导机械编排\mechanization_result.txt');

timestamp=error_data(:,1);

posx=error_data(:,2);
posy=error_data(:,3);
posz=error_data(:,4);

velx=error_data(:,5);
vely=error_data(:,6);
velz=error_data(:,7);

attx=error_data(:,8);
atty=error_data(:,9);
attz=error_data(:,10);

%% 打印位置误差
figure(1);
subplot(3,1,1);
plot(timestamp,posx)
xlabel('time(s)');
ylabel('latitude(deg)');
grid on;

subplot(3,1,2);
plot(timestamp,posy)
xlabel('time(s)');
ylabel('longitude(deg)');
grid on;

subplot(3,1,3);
plot(timestamp,posz)
xlabel('time(s)');
ylabel('height(m)');
grid on;

%% 打印速度误差
figure(2)
subplot(3,1,1);
plot(timestamp,velx)
xlabel('time(s)');
ylabel('velX(m/s)');
grid on;

subplot(3,1,2);
plot(timestamp,vely)
xlabel('time(s)');
ylabel('velY(m/s)');
grid on;

subplot(3,1,3);
plot(timestamp,velz)
xlabel('time(s)');
ylabel('velZ(m/s');
grid on;

%% 打印姿态误差
figure(3);
subplot(3,1,1);
plot(timestamp,attx)
xlabel('time(s)');
ylabel('roll(deg)');
grid on;

subplot(3,1,2);
plot(timestamp,atty)
xlabel('time(s)');
ylabel('pitch(deg)');
grid on;

subplot(3,1,3);
plot(timestamp,attz)
xlabel('time(s)');
ylabel('yaw(deg)');
grid on;