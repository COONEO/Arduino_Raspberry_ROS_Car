
newData = importdata('Data.txt', '\t', 2);
data=newData.data;

subplot(2,2,1);plot(data(:,1),data(:,2:4));grid on;xlabel('ʱ��/s');ylabel('���ٶ�/g');title('���ٶ�����');
subplot(2,2,2);plot(data(:,1),data(:,5:7));grid on;xlabel('ʱ��/s');ylabel('���ٶ�/��/s');title('���ٶ�����');
subplot(2,2,3);plot(data(:,1),data(:,8:10));grid on;xlabel('ʱ��/s');ylabel('�Ƕ�/��');title('�Ƕ�����');
subplot(2,2,4);plot(data(:,1),data(:,11));grid on;xlabel('ʱ��/s');ylabel('�¶�/��');title('�¶�����');

