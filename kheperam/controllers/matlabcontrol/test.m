%% Init
speed = 1;
left_speed=1;
right_speed=1;
Kp = 0.01*speed;
Ki = 0;
Kd = 0;
wheel_base = 0.053;
wheel_radius = 0.008;
encoder_resulution = 100;

phi = 0;
z = 0;
x = 0;
z = -.23;
x = -.2;
phi_dot =0;
z_dot=0;
x_dot=0;
encoder_value_left=0;
encoder_value_right=0;
wb_differential_wheels_set_encoders(encoder_value_left,encoder_value_right);

z_home = z;
x_home = x;
minimum_radius = 0.1;
home_radius = 0.0015;
been_away_from_home = 0;

time = wb_robot_get_time();

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
  dt = wb_robot_get_time() - time;
  time = wb_robot_get_time();%update time
  
  delta_gamma_left = (wb_differential_wheels_get_left_encoder()-encoder_value_left)/encoder_resulution;
  speed_left = (delta_gamma_left*wheel_radius)/dt;%speed sl
  encoder_value_left = wb_differential_wheels_get_left_encoder();%update
  
  delta_gamma_right = (wb_differential_wheels_get_right_encoder()-encoder_value_right)/encoder_resulution;
  speed_right = (delta_gamma_right*wheel_radius)/dt;%speed sr
  encoder_value_right = wb_differential_wheels_get_right_encoder();%update
  
  phi_dot = -(speed_left - speed_right)/(wheel_base);
  phi = phi + phi_dot * dt
  
  z_dot = (-(speed_left + speed_right)/(2))*cos(phi);
  z = z + z_dot * dt
  
  x_dot = (-(speed_left + speed_right)/(2))*sin(phi);
  x = x + x_dot * dt
  
  distance_from_home = (x-x_home)^2 + (z-z_home)^2
  
  if (distance_from_home > minimum_radius)
      been_away_from_home = 1;
  end
  
  if (distance_from_home < home_radius)
      is_home = 1;
  else
      is_home = 0;
  end
%% Controlling

%error_left = get_sensor_value('left',sensor_values) - 600;


if (been_away_from_home && is_home)
     left_speed = 0;
     right_speed = 0;
 else
    if ( (sensor_values(2) > 600) || (sensor_values(3)  > 600))
        left_speed=speed;
        right_speed=-speed;
    else
        if ((sensor_values(2) == 0) && (sensor_values(1) ~= 0))
            error_left = get_sensor_value('left',sensor_values) - 600;
            left_speed=speed + Kp * error_left;
            right_speed=speed;
        elseif ((sensor_values(2) == 0) && (sensor_values(1) == 0))
            left_speed=0*speed;
            right_speed=1*speed;
        else
            error_left = get_sensor_value('2left',sensor_values) - 850;
            left_speed=speed + Kp * error_left;
            right_speed=speed;
        end
    end
end
 
wb_differential_wheels_set_speed(left_speed, right_speed);


end
