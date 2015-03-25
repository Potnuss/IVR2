
%% Init
general_speed = 5; %rad/s
general_fast_speed = 10; %rad/s
output_left_speed=1;
output_right_speed=1;
%Kp = 0.01*general_fast_speed;
Kp = 0.05;
Ki = 0;
Kd = 0;
wb_wheel_base = 0.053;
wb_wheel_radius = 0.008;
wb_encoder_resulution = 100;

%odometric variable declarations
odo_phi = 0;
odo_z = 0;
odo_x = 0;
odo_phi_dot =0;
odo_z_dot=0;
odo_x_dot=0;
odo_encoder_value_left=0;
odo_encoder_value_right=0;
wb_differential_wheels_set_encoders(odo_encoder_value_left,odo_encoder_value_right);

odo_z_home = odo_z;
odo_x_home = odo_x;
odo_minimum_radius = 0.1;
odo_home_radius = 0.0015;
odo_been_away_from_home = 0;

last_time = wb_robot_get_time();

turn_radius=0.026 %meter
turn_anglespeedradpersec = general_speed*wb_wheel_radius/(turn_radius+(wb_wheel_base/2))
turn_outer_speed=(turn_anglespeedradpersec*(turn_radius + wb_wheel_base))/wb_wheel_radius
turn_inner_speed=(turn_anglespeedradpersec*turn_radius)/wb_wheel_radius

stored_odo_phi = 0; %Just for test
stored_odo_phi2 = 0;%Just for test

turned_angle = 0;
turned_angle2 = 0;
timer1 = 0;
edge_detected = 0;
diff_sensor_values1 = 0;
stored_sensor_values1 = 0;
stuck_timer_enabled = 0;
timer2 = 0;

%state declarations
state_finish = 1;
state_obstacle_in_front = 2;
state_obstacle_check_intruder = 3;
state_obstacle_turn_on_spot = 4;
state_lost_wall = 5;
state_follows_wall = 6;
state_movement_detected = 7;
state_default = 8;
state_look_for_wall = 9;
state = 8;
state_initialized = 0;

stored_all_sensor_values = [0,0,0,0,0,0,0,0,last_time]  %initally store all sensor values as 0
%% Main loop
while wb_robot_step(TIME_STEP) ~= -1

%% Update sensors
% Read all distance sensors
  for i=1:N
    sensor_values(i) = wb_distance_sensor_get_value(ps(i));
  end
       
% display all distance sensors values
  sensor_values
  stored_all_sensor_values = [stored_all_sensor_values;sensor_values,last_time]
  
%% Odometry
  odo_dt = wb_robot_get_time() - last_time;
  last_time = wb_robot_get_time();%update time
  
  odo_delta_gamma_left = (wb_differential_wheels_get_left_encoder()-odo_encoder_value_left)/wb_encoder_resulution;
  odo_speed_left_rads = (odo_delta_gamma_left)/odo_dt;%speed sr m/s
  odo_speed_left_ms= odo_speed_left_rads*wb_wheel_radius;%speed sl m/s
  odo_encoder_value_left = wb_differential_wheels_get_left_encoder();%update
  
  odo_delta_gamma_right = (wb_differential_wheels_get_right_encoder()-odo_encoder_value_right)/wb_encoder_resulution;
  odo_speed_right_rads = (odo_delta_gamma_right)/odo_dt;%speed sr m/s
  odo_speed_right_ms = odo_speed_right_rads*wb_wheel_radius;%speed sr m/s
  odo_encoder_value_right = wb_differential_wheels_get_right_encoder();%update
  
  odo_phi_dot = -(odo_speed_left_ms - odo_speed_right_ms)/(wb_wheel_base);
  odo_phi = odo_phi + odo_phi_dot * odo_dt
  
  odo_z_dot = (-(odo_speed_left_ms + odo_speed_right_ms)/(2))*cos(odo_phi);
  odo_z = odo_z + odo_z_dot * odo_dt
  
  odo_x_dot = (-(odo_speed_left_ms + odo_speed_right_ms)/(2))*sin(odo_phi);
  odo_x = odo_x + odo_x_dot * odo_dt
  
  odo_distance_from_home = (odo_x-odo_x_home)^2 + (odo_z-odo_z_home)^2
  
  if (odo_distance_from_home > odo_minimum_radius)
      odo_been_away_from_home = 1;
  end
  
  if (odo_distance_from_home < odo_home_radius)
      odo_is_home = 1;
  else
      odo_is_home = 0;
  end
%% Controlling
%% State Machine
while(true)
%% Init States
    if(state_initialized == 0)
    %Init states

        if (state == state_obstacle_in_front)
        elseif (state == state_obstacle_check_intruder) 
                timer1_start_value = wb_robot_get_time();
                start_listen_for_intruder = 0;

        elseif (state == state_obstacle_turn_on_spot)
                stored_odo_phi = odo_phi;         %store odometry values
                stored_front_sensor_value = sensor_values(3);     %store front sensor value
                edge_detected = 0;

        elseif (state == state_lost_wall)  
            stored_odo_phi2 = odo_phi;     %store odometry values

        elseif (state == state_default)
            stored_odo_phi3 = odo_phi;      %store odometry values
        end
        state_initialized = 1;
        stuck_timer_enabled = 0; %Always reset the stuck timer when enter a new state
    end

    %% Make transition from state or stay in state x
    if (state == state_follows_wall)                  %if currently following a wall
        error_left = get_sensor_value('left',sensor_values) - 600;  %calculate the error
        
        if (odo_been_away_from_home && odo_is_home)   %check if robot has left home radius and returned
            state = state_finish;                       %enter finish state
            state_initialized = 0;
        elseif ((sensor_values(3)  > 570) || (sensor_values(4)  > 570))   %check if the front sensors detect an obstacle
            state = state_obstacle_in_front;                                %enter obstacle in front state
            state_initialized = 0;
        elseif (sensor_values(2) == 0) && (sensor_values(1) == 0)         %check if the leftsensors detect a wall
            state = state_lost_wall;                                        %enter lost wall state
            state_initialized = 0; 
        else
        end
          
    elseif (state == state_obstacle_check_intruder)     %if currently in obstacle_check intruder
        
        timer1 =  wb_robot_get_time() - timer1_start_value      %calculate the timer
        
        stored_sensor_values1 %only for show in console
        odo_speed_right_rads  %only for show in console
        odo_speed_left_rads   %only for show in console
        
        if (((abs(odo_speed_left_rads) + abs(odo_speed_right_rads)) == 0) ...
                && (start_listen_for_intruder == 0))
                
            start_listen_for_intruder = 1;        %enter start_listen
            stored_sensor_values1 = sensor_values;    %store current sensor values
        end
        if(start_listen_for_intruder)             %if currently listening for an intruder
            diff_sensor_values1 = abs(stored_sensor_values1 - sensor_values)    %calculate the difference in sensor values to determine the presence of an intruder
        end
        
        if (odo_been_away_from_home && odo_is_home)   %check if robot has left home radius and returned
            state = state_finish;
            state_initialized = 0;
        elseif (timer1 > 0.5) && (sensor_values(3)  > 570)  %if obstacle is still stationary after .5  seconds
            state = state_obstacle_turn_on_spot;              %enter turning state
            state_initialized = 0;
        elseif(start_listen_for_intruder && (max(diff_sensor_values1) > 10))  %if currently listening for intruders and sensor values change
            state = state_movement_detected;    %enter movement detected state
            state_initialized = 0;
        end
    elseif (state == state_obstacle_turn_on_spot)   %if currently in turning state
        turned_angle = abs(stored_odo_phi - odo_phi);   %calculate the turned angle
        dval = abs(stored_front_sensor_value - sensor_values(3)); %calculate the difference
        if (abs(stored_front_sensor_value - sensor_values(3) > 400))  %check if the front sensors have found an edge
            edge_detected = 1;
        end
        stored_front_sensor_value = sensor_values(3); %store the current sensor values
        
        if (odo_been_away_from_home && odo_is_home) %check if robot has left home radius and returned
            state = state_finish;
            state_initialized = 0;
        elseif ((edge_detected == 0) && ...
            ((turned_angle > (pi/2)) ||...
            (get_sensor_value('23',sensor_values) < 280)))  %check if robot has found a wall
            state = state_follows_wall;                       %enter follow wall state
            state_initialized = 0;
            
        elseif (edge_detected == 1) && (turned_angle > (pi/2)) && (sensor_values(1) > 500) %if the robot loses sight of wall suddenly
            state = state_lost_wall;  %enter lost wall state
            state_initialized = 0;
        elseif (edge_detected == 1) && (turned_angle > (pi/2)*0.9)      %if the robot finds an edge and turns toward it
            state = state_follows_wall;                                   %enter follow wall state
            state_initialized = 0;
        end
        
    elseif (state == state_lost_wall)       %if currently searching for wall
        turned_angle2 = abs(stored_odo_phi2 - odo_phi); %calculate the angle turned
        
        if (odo_been_away_from_home && odo_is_home)     %check if robot has left home radius and returned
            state = state_finish;                             
            state_initialized = 0;
        elseif ((sensor_values(3)  > 570) || (sensor_values(4)  > 570)) %check if front sensors detect an obstacle
            state = state_obstacle_in_front;                              %enter obstacle in front state
            state_initialized = 0;
        elseif (turned_angle2 > (pi/2)) || (get_sensor_value('left',sensor_values) > 600) %check if robot has turned sufficiently to have found a wall on its left  
            state = state_follows_wall;     %enter follow wall state
            state_initialized = 0;

        end
    elseif (state == state_obstacle_in_front) %if currently faced with an obstacle
        state = state_obstacle_check_intruder;  %enter state to check if it is an intruder
        state_initialized = 0;
        
    elseif (state == state_default)                   %if currently in the default state
        turned_angle3 = abs(stored_odo_phi3 - odo_phi)  %store new turned angle
        
        if (odo_been_away_from_home && odo_is_home) %check if robot has left home radius and returned
            state = state_finish;                     %if it has, the robot is done
            state_initialized = 0;
        elseif ((sensor_values(1) < 800)...
                && (sensor_values(1) > 400)...
                && (get_sensor_value('2front',sensor_values) == 0)...
                && (sensor_values(2) < 280))  %if the robot has found a wall
            state = state_follows_wall;         %enter follow wall state
            state_initialized = 0;
        elseif ((turned_angle3 > 2*pi)...
                && (get_sensor_value('4front',sensor_values) == 0)...
                &&  (sensor_values(1) < 950))   %if robot cannot find any wall
            state = state_look_for_wall;          %enter state to look for wall
            state_initialized = 0;
        elseif ((turned_angle3 > 4*pi) && (get_sensor_value('2front',sensor_values) == 0))  %if robot cannot find any wall
            state = state_look_for_wall;                                                     %enter state to look for wall
            state_initialized = 0;
        end
    elseif (state == state_look_for_wall)   %if  currently looking for wall
        if ((get_sensor_value('max4front',sensor_values) > 570))
            state = state_default;  %transition to default state
            state_initialized = 0;
        end
    elseif (state == state_movement_detected)   %if the robot detects motion while it is stopped
        disp('ALARM: INTRUDER!')                  %report the intruder
    elseif (state == state_finish)              %otherwise, the robot has completed its tasks
        disp('state_finish')
    end
%% Stuck timer (overrides all states)
%if the robot becomes stuck, this timer activates
%after a certain period of time the robot moves to become unstuck
    if ((odo_speed_right_rads == 0) && (odo_speed_right_rads == 0) )                              %if the robot is not moving
        if ((output_left_speed ~= 0)  && (output_right_speed ~= 0) && (stuck_timer_enabled == 0)) %but should be moving
            stuck_timer_enabled = 1;                                                              %enable the timer
            timer2_start_value = wb_robot_get_time();
        elseif(stuck_timer_enabled)                             %if the stuck timer is enabled
            timer2 = wb_robot_get_time() - timer2_start_value;  
            if (timer2 > 1)                                     %and the time difference is great than 1
                state = state_default;                          %restore the default state
                state_initialized = 0;
            end
        end
    elseif(stuck_timer_enabled)
        stuck_timer_enabled = 0;
    end   
%%  End of State Machine
if (state_initialized == 1); break; end
end

%% main contol output
%this sets the wheel speeds depending on the current state

if (state == state_finish)      %if currently in finish state
      output_left_speed = 0;      %stop moving
      output_right_speed = 0;
elseif (state == state_obstacle_check_intruder) %if looking for intruder
    output_left_speed=0;                          %stop moving
    output_right_speed=0;
elseif (state == state_obstacle_turn_on_spot)     %if turning on spot
    output_left_speed=general_speed;                %set left and right wheels to  opposite speeds
    output_right_speed=-general_speed;
elseif (state == state_lost_wall)                 %if the wall was lost
    output_right_speed=turn_outer_speed;            %set the calculated turning speeds
    output_left_speed=turn_inner_speed;
elseif (state == state_follows_wall)                        %if following a wall
    output_left_speed=general_fast_speed ;                    %set left wheel to faster speed
    output_right_speed=general_fast_speed - Kp * error_left;  %set right wheel to speed proportional to the error 
                                                              % to stay near the wall
elseif(state == state_movement_detected)          %if movement is detected
    output_left_speed=50;                           %spin in circles
    output_right_speed=-50;
elseif(state == state_default)                    %if in default state
    output_left_speed=general_speed;                %move at general speed
    output_right_speed=-general_speed;
elseif(state == state_look_for_wall)              %if looking for the wall
    output_left_speed=general_speed;                %move at general speed
    output_right_speed=general_speed;
else
    %keep the old values
end



wb_differential_wheels_set_speed(output_left_speed, output_right_speed);
% turned_angle
% edge_detected
% timer1
% diff_sensor_values1

state
state_initialized


end
