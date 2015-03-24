load('speed5.mat')
stored_all_sensor_values = stored_all_sensor_values(196:end,:);
plot(stored_all_sensor_values(:,9),stored_all_sensor_values(:,1))
xlabel('Time')
ylabel('Left sensor value')