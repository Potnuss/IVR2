function ret = set_speeds(left_speed,right_speed)
% converts integer speeds to the appropriate command string
% and sends it to the robot or simulator

string = sprintf('D,%d,%d',left_speed,right_speed);
send_command(string);
ret = read_command