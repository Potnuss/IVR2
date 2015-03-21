
%% Init
general_speed = 5; %rad/s
output_left_speed=1;
output_right_speed=1;
Kp = 0.01*general_speed;
Ki = 0;
Kd = 0;
wb_wheel_base = 0.053;
wb_wheel_radius = 0.008;
wb_encoder_resulution = 100;

odo_phi = 0;
odo_z = 0;
odo_x = 0;
odo_z = -.23;
odo_x = -.2;
odo_phi_dot =0;
odo_z_dot=0;
odo_x_dot=0;
odo_encoder_value_left=0;
odo_encoder_value_right=0;
wb_differential_wheels_set_encoders(odo_encoder_value_left,odo_encoder_value_right);

odo_z_home = odo_z;
odo_x_home = odo_x;
odo_minimum_radius = 0.5;%0.1
odo_home_radius = 0.0015;
odo_been_away_from_home = 0;

last_time = wb_robot_get_time();

turn_radius=0.026 %meter
turn_anglespeedradpersec = general_speed*wb_wheel_radius/(turn_radius+(wb_wheel_base/2))
turn_outer_speed=(turn_anglespeedradpersec*(turn_radius + wb_wheel_base))/wb_wheel_radius
turn_inner_speed=(turn_anglespeedradpersec*turn_radius)/wb_wheel_radius

stored_odo_phi = 0; %Just for test
stored_odo_phi2 = 0;%Just for test
edge_detected = 0;
state_obsticle_in_front = 0;
state_lost_wall = 0;
state_follows_wall = 1;
state_finnish = 0;
%% Main loop
while wb_robot_step(TIME_STEP) ~= -1
%% Update sensors
% Read all distance sensors
       for i=1:N
           sensor_values(i) = wb_distance_sensor_get_value(ps(i));
       end
       
% display all distance sensors values
  sensor_values
  
%% Odometry
  odo_dt = wb_robot_get_time() - last_time;
  last_time = wb_robot_get_time();%update time
  
  odo_delta_gamma_left = (wb_differential_wheels_get_left_encoder()-odo_encoder_value_left)/wb_encoder_resulution;
  odo_speed_left = (odo_delta_gamma_left*wb_wheel_radius)/odo_dt;%speed sl
  odo_encoder_value_left = wb_differential_wheels_get_left_encoder();%update
  
  odo_delta_gamma_right = (wb_differential_wheels_get_right_encoder()-odo_encoder_value_right)/wb_encoder_resulution;
  odo_speed_right = (odo_delta_gamma_right*wb_wheel_radius)/odo_dt;%speed sr
  odo_encoder_value_right = wb_differential_wheels_get_right_encoder();%update
  
  odo_phi_dot = -(odo_speed_left - odo_speed_right)/(wb_wheel_base);
  odo_phi = odo_phi + odo_phi_dot * odo_dt
  
  odo_z_dot = (-(odo_speed_left + odo_speed_right)/(2))*cos(odo_phi);
  odo_z = odo_z + odo_z_dot * odo_dt
  
  odo_x_dot = (-(odo_speed_left + odo_speed_right)/(2))*sin(odo_phi);
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

error_left = get_sensor_value('left',sensor_values) - 600;
error_2left = get_sensor_value('2left',sensor_values) - 850;

%sTrigger/init states
if ( (state_obsticle_in_front == 0) && (sensor_values(3)  > 570))
        state_obsticle_in_front = 1;
        stored_odo_phi = odo_phi;
        stored_front_sensor_value = sensor_values(3); 
        edge_detected = 0;
end
sensor_values(2)
sensor_values(1)
if ((state_obsticle_in_front == 0) && ...
    (state_follows_wall == 1) && ...
        (state_lost_wall == 0) && ...
        (sensor_values(2) == 0) && (sensor_values(1) == 0))
    
    state_lost_wall = 1;
    %state_follows_wall = 0 %should be here later TODO maybe?
    stored_odo_phi2 = odo_phi;
end
if (odo_been_away_from_home && odo_is_home)
    state_finnish = 1;
end

%Calculate values
turned_angle = abs(stored_odo_phi - odo_phi);
turned_angle2 = abs(stored_odo_phi2 - odo_phi);

if (state_obsticle_in_front)
    dval = abs(stored_front_sensor_value - sensor_values(3))
    if (abs(stored_front_sensor_value - sensor_values(3) > 400))
        edge_detected = 1;
    end
    stored_front_sensor_value = sensor_values(3); %maybe move this?
end

%Cancell states
if (state_obsticle_in_front)
    if ((edge_detected == 0) && ...
            ((turned_angle > (pi/2)) ||...
            (get_sensor_value('23',sensor_values) < 280)))
        disp('Cancell1 state_obsticle_in_front')
        state_obsticle_in_front = 0;
    end
    if (edge_detected == 1) && (turned_angle > (pi/2))
        disp('Cancell2 state_obsticle_in_front')
        state_obsticle_in_front = 0;
    end
end

if (turned_angle2 > (pi/2)) || (get_sensor_value('left',sensor_values) > 600)
state_lost_wall = 0;
end

%main contol output
if (state_finnish)
      output_left_speed = 0;
      output_right_speed = 0;
else
    if ( state_obsticle_in_front )
            output_left_speed=general_speed;
            output_right_speed=-general_speed;
    else
        if (state_lost_wall)
            output_right_speed=turn_outer_speed
            output_left_speed=turn_inner_speed
        else %Follow wall as usual
            output_left_speed=general_speed + Kp * error_left;
            output_right_speed=general_speed;
        end
    end
end

wb_differential_wheels_set_speed(output_left_speed, output_right_speed);
% turned_angle
% edge_detected
% 
% state_finnish
% state_obsticle_in_front
% state_lost_wall
% state_follows_wall


end
