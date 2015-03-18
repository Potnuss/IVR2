function [left_count, right_count] = read_counts()
% sends a read_counter command to the robot or simulator and 
% converts the answer to integer values

string = 'H';
send_command(string);
reply = read_command;
counts = sscanf(reply,'h,%d,%d');
left_count = counts(1);
right_count = counts(2);
