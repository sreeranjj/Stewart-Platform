%plotter. Done just to get  plots for report

t=(1:(Kfinal+1))*Ts;
%omega
% for i=1:3
%     subplot(3,1,i);plot(t,XkStore(i+6,:));title(i+6);
%     xlabel('time in s')
% end
%XYZ
% ylab=['X';'Y';'Z']
% title('XYZ vs time')
% for i=1:3
%     subplot(3,1,i);plot(t,XkStore(i,:));
%     xlabel('time in s')
%     ylabel(ylab(i))
% end

% t=(1:(Kfinal))*Ts;
% for i=1:4
%     subplot(2,2,i);plot(t,ukStore(i,:));title(i)
%     xlabel('time in s')
%     ylabel('input(i)')
% end

%X pos
t=(1:(Kfinal+1))*Ts;
plot(t,XkStore(1,:));
xlabel('time in s')
ylabel('X position in m')
title('lambda=0.06,p=20,m=7')

% %Yaw angle
% t=(1:(Kfinal+1))*Ts;
% plot(t,180/pi*XkStore(12,:));
% xlabel('time in s')
% ylabel('yaw angle in deg')
% title('Yaw vs time with lambda=0.06')
