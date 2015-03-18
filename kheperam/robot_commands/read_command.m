function string=read_command()
% depending on the status of the global variable ROBOT 
% and whether the appropriate connection is established
% reads a line from the serial port or to the tcpip connection
global FID
global S

string = pnet(FID,'readline')
