port = input('What port number is webots using? ');
global FID
global ROBOT

% try opening the connection
FID = pnet('tcpconnect','localhost',port);

% check the opening was successful
if FID <0
    ROBOT = 0;
    error('not opened')
else 
    ROBOT = -1;  
end
