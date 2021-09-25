
newData = importdata('Data.txt', '\t', 2);
data=newData.data;

subplot(2,2,1);plot(data(:,1),data(:,2:4));grid on;xlabel('时间/s');ylabel('加速度/g');title('加速度曲线');
subplot(2,2,2);plot(data(:,1),data(:,5:7));grid on;xlabel('时间/s');ylabel('角速度/°/s');title('角速度曲线');
subplot(2,2,3);plot(data(:,1),data(:,8:10));grid on;xlabel('时间/s');ylabel('角度/°');title('角度曲线');
subplot(2,2,4);plot(data(:,1),data(:,11));grid on;xlabel('时间/s');ylabel('温度/°');title('温度曲线');

