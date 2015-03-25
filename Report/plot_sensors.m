load('speed5.mat')
stored_all_sensor_values = stored_all_sensor_values(196:end,:);
plot(stored_all_sensor_values(:,9),stored_all_sensor_values(:,1))
xlabel('Time [s]')
ylabel('Left sensor value (range 0-1000)')
%% Odemetry1
close all
load('odometry1.mat')
stored_all_position_values = stored_all_position_values(2:end,:)
gx = stored_all_position_values(:,1);
gz = stored_all_position_values(:,3);
x = stored_all_position_values(:,4);
z = stored_all_position_values(:,6);
time = stored_all_position_values(:,7);

% % 
hold on
figure(1)
plot(stored_all_position_values(:,1),stored_all_position_values(:,3),'r')
plot(stored_all_position_values(:,4),stored_all_position_values(:,6),'k')


xlabel('x Position [m]')
ylabel('y Position [m]')
stored_all_position_values(:,1),stored_all_position_values(:,3)
legend('GPS, true position','Odometry')
figure(2)
xerror = (gx-x);
zerror = (gz-z);
toterror = sqrt(xerror.^2+zerror.^2);
plot(time,toterror)
xlabel('Time [s]')
ylabel('Positional Error [m]')
