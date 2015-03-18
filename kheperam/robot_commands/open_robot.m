global S
global ROBOT
ROBOT = 1;
S = serial('/dev/ttyS0');
fopen(S)
if strcmp(S.status,'closed')
    ROBOT = 0;
else 
    ROBOT = 1;  
end