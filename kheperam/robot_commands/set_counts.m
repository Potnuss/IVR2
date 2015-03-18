function ret = set_counts(left_count,right_count)
% converts integer speeds to the appropriate command string
% and sends it to the robot or simulator

string = sprintf('G,%d,%d',left_count,right_count);
send_command(string);
ret = read_command;