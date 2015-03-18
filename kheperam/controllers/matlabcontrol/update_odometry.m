function [x,z,phi,x_dot,z_dot,phi_dot,encoder_value_left,encoder_value_right,time] = update_odometry(x,z,phi,x_dot,z_dot,phi_dot,encoder_value_left,encoder_value_right,time)
wheel_base = 0.053;
wheel_radius = 0.008;
encoder_resulution = 100;

dt = wb_robot_get_time() - time
time = wb_robot_get_time();

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
end