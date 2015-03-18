function []=send_command(string)
% depending on the status of the global variable ROBOT 
% and whether the appropriate connection is established
% sends a command to the serial port or to the tcpip connection
global ROBOT
global FID
global S

if ROBOT > 0
    fprintf(S,string);
elseif ROBOT < 0
    pnet(FID,'printf',string);
end   
