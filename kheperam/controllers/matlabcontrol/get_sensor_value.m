function [ value ] = get_sensor_value(string,sensor_values)
%GET_LEFT_SENSORS Summary of this function goes here
%   Detailed explanation goes here
if strcmp(string,'left')
    value = sensor_values(1);
elseif strcmp(string,'2left')
    value = sensor_values(1)+sensor_values(2);
elseif strcmp(string,'3left')
    value = sensor_values(1)+sensor_values(2)+sensor_values(3);
elseif strcmp(string,'right')
    value = sensor_values(6);
elseif strcmp(string,'3right')
    value = sensor_values(4)+sensor_values(5)+sensor_values(6);
elseif strcmp(string,'frontleft')
    value = sensor_values(3);
elseif strcmp(string,'23')
    value = sensor_values(2) + sensor_values(3);
elseif strcmp(string,'2front')
    value = sensor_values(3) + sensor_values(4);
elseif strcmp(string,'4front')
    value = sensor_values(2) + sensor_values(3) + sensor_values(4) + sensor_values(5);
elseif strcmp(string,'max4front')
    temp = [sensor_values(2),sensor_values(3),sensor_values(4),sensor_values(5)]
    value = max(temp);
else
    value = 0;
end
end

