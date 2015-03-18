% Very simple MATLAB controller for Webots
% File: matlabcontrol.m
% Date: 21/09/2011          
% Description: This controller will open MATLAB and let you input
%              commands from the keyboard
% Author: Simon Smith (artificialsimon@ed.ac.uk)        
% Modifications: 
% Use: command to set motor speed 1 to left motor, -1 to right motor
%       wb_differential_wheels_set_speed(1, -1)
% After sending a setting command, the controller have to resume by
% sending "return" in the keyboard input
% This will allow a sensor value update as well

TIME_STEP = 64;
N = 8;

% get and enable distance sensors
for i=1:N
  ps(i) = wb_robot_get_device(['ds' int2str(i-1)]);
  wb_distance_sensor_enable(ps(i),TIME_STEP);
end
wb_differential_wheels_enable_encoders(TIME_STEP);

% Calling MATLAB desktop versionwb_differential_wheels_set_speed(1, -1);
desktop;

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the controll to the keyboard
while wb_robot_step(TIME_STEP) ~= -1
%keyboard;
 test;
  end
