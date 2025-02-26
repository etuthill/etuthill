hold off
clear
syms t
u = 0.1 * t;

x = 0.3960*cos(2.65*(u + 1.4));
y = -0.99*sin(u + 1.4);

t_vals = linspace(0, 32, 1000);
x_vals = double(subs(x, t, t_vals));
y_vals = double(subs(y, t, t_vals));

figure()
h1 = plot(x_vals, y_vals, "b", "LineWidth", 1.5);
axis([-1.5, 1.5, -1.5, 1.5])
hold on


v_x = diff(x);
v_y = diff(y);
v_t = sqrt(v_x^2 + v_y^2);
T_x = v_x / v_t;
T_y = v_y / v_t;

t_vals_tan = [0, 5, 10, 15, 20, 25, 30];

scale = 0.3;

for i = 1:length(t_vals_tan)
    x_pos = double(subs(x, t, t_vals_tan(i)));
    y_pos = double(subs(y, t, t_vals_tan(i)));
    T_x_val = double(subs(T_x, t, t_vals_tan(i)));
    T_y_val = double(subs(T_y, t, t_vals_tan(i)));
    h2 = plot([x_pos, x_pos + T_x_val*scale], [y_pos, y_pos + T_y_val*scale], 'r-', "LineWidth", 1.5);
end


q_x = diff(T_x);
q_y = diff(T_y);
q_norm = sqrt(q_x^2 + q_y^2);
N_x = q_x / q_norm;
N_y = q_y / q_norm;

for i = 1:length(t_vals_tan)
    x_pos = double(subs(x, t, t_vals_tan(i)));
    y_pos = double(subs(y, t, t_vals_tan(i)));
    N_x_val = double(subs(N_x, t, t_vals_tan(i)));
    N_y_val = double(subs(N_y, t, t_vals_tan(i)));
    h3 = plot([x_pos, x_pos + N_x_val*scale], [y_pos, y_pos + N_y_val*scale], 'c-', "LineWidth", 1.5);    
end
legend([h1, h2, h3], "Curve", "Tangent", "Normal")
title("Parametric Curve")
xlabel("X Position (meters)")
ylabel("Y Position (meters)")

%%
%Linear speed and Angular velocity

my_path = 'C:\Users\etuthill\OneDrive - Olin College of Engineering\QEA2\Week 4\';
fname = 'Vid_1_data.mat';
neato_data = load([my_path,fname],'recorded_data').recorded_data;


time = neato_data(:,1); 
xl = neato_data(:,2); 
xr = neato_data(:,3); 

tlist = neato_data(:,1);
left_wheel_encoder_list = neato_data(:,2);
right_wheel_encoder_list = neato_data(:,3);
left_wheel_vel_input_list = neato_data(:,4);
right_wheel_vel_input_list = neato_data(:,5);

d = 0.24; 
dt = diff(tlist); 

v_L = diff(left_wheel_encoder_list) ./ dt;
v_R = diff(right_wheel_encoder_list) ./ dt;

linear_speed_actual = 0.5 * (v_L + v_R);  
angular_velocity_actual = (v_R - v_L) / d;

t_actual = tlist(2:end);


planned_angular_velocity = T_x * q_y - T_y * q_x;

v_1 = diff(x);
v_2 = diff(y);
planned_linear_speed = simplify(sqrt(v_1^2 + v_2^2));

figure()
subplot(2, 1, 1)
fplot(planned_angular_velocity, [0, 32], "r-");
title("Angular Velocity")
hold on
subplot(2, 1, 1)
plot(t_actual, angular_velocity_actual, "c--")
legend("Predicted", "Actual", "Location", "southwest")
xlabel("Time (seconds)")
ylabel("Angular Velocity (rad/s)")


subplot(2, 1, 2)
fplot(planned_linear_speed, [0, 32], "r-");
title("Linear Speed")
hold on
subplot(2, 1, 2)
plot(t_actual, linear_speed_actual, "c--")
legend("Predicted", "Actual", "Location", "northwest")
xlabel("Time (seconds)")
ylabel("Linear Speed (m/s)")
%%
%Wheel speeds



d = 0.24;
t = linspace(0, 32, 200);
vl = planned_linear_speed - 0.5 * d * planned_angular_velocity;
vr = planned_linear_speed + 0.5 * d * planned_angular_velocity;
vl_vals = double(subs(vl, t));
vr_vals = double(subs(vr, t));

dt = diff(tlist);
delta_left_encoder = diff(left_wheel_encoder_list); 
delta_right_encoder = diff(right_wheel_encoder_list); 

v_L_actual = delta_left_encoder ./ dt; 
v_R_actual = delta_right_encoder ./ dt;


figure()
plot(t, vl_vals, 'b', 'linewidth', 2);
hold on
plot(t, vr_vals, 'r', 'linewidth', 2);
plot(t_actual, v_L_actual, "b--")
plot(t_actual, v_R_actual, "r--")
legend("Left Wheel Predicted", "Right Wheel Predicted", "Left Wheel Actual", "Right Wheel Actual", "Location", "north");
title("Wheel Velocity");
xlabel("Time (seconds)")
ylabel("Velocity (m/s)")
hold off;

%%
% 
% Neato Programming
% 
% neatov3.connect('192.168.16.91');
% 
% t_vals = 0:0.05:32;
% 
% vl_range = double(subs(vl,t_vals));
% vr_range = double(subs(vr,t_vals));
% 
% num_samples = length(t_vals);
% recorded_data = zeros(num_samples, 5);
% index = 1; 
% tic;
% while toc<=32
% 
%     t_in = toc;
%     vl_out = interp1(t_vals,vl_range,t_in);
%     vr_out = interp1(t_vals,vr_range,t_in);
%     neatov3.setVelocities(vl_out, vr_out)
%     sensors = neatov3.receive();
%     if length(sensors.encoders)>=2
%     xl = sensors.encoders(1);
%     xr = sensors.encoders(2);
%     recorded_data(index, :) = [t_in, xl, xr, vl_out, vr_out];
%     index = index + 1;
%     end
% end
% 
% recorded_data = recorded_data(1:index-1, :);
% 
% writematrix(recorded_data, 'neato_data.csv');
% save('neato_data.mat', 'recorded_data');
% 
% neatov3.setVelocities(0,0);
% 
% sensors = neatov3.receive();
% neatov3.disconnect();


%%


syms t
u = 0.1 * t;

x = 0.3960*cos(2.65*(u + 1.4));
y = -0.99*sin(u + 1.4);

t_vals = linspace(0, 32, 1000);
x_vals = double(subs(x, t, t_vals));
y_vals = double(subs(y, t, t_vals));

figure()
plot(x_vals, y_vals, "r", "LineWidth", 2)
axis([-2, 2, -2, 2])
hold on

my_path = 'C:\Users\etuthill\OneDrive - Olin College of Engineering\QEA2\Week 4\';
fname = 'Vid_1_data.mat';
neato_data = load([my_path,fname],'recorded_data').recorded_data;

tlist = neato_data(:,1);
left_wheel_encoder_list = neato_data(:,2);
right_wheel_encoder_list = neato_data(:,3);
left_wheel_vel_input_list = neato_data(:,4);
right_wheel_vel_input_list = neato_data(:,5);

vl_approx = diff(left_wheel_encoder_list)./diff(tlist);
vr_approx = diff(right_wheel_encoder_list)./diff(tlist);

d = .24;
omega = (vr_approx-vl_approx)/d;

theta = pi/2+[0;cumsum(omega.*diff(tlist))];
signed_speed_approx = (vl_approx+vr_approx)/2;
vx_approx = cos(theta(1:end-1)).*signed_speed_approx;
vy_approx = sin(theta(1:end-1)).*signed_speed_approx;


px_approx = [0;-cumsum(vx_approx.*diff(tlist))];
py_approx = [0;cumsum(vy_approx.*diff(tlist))];

theta_r = -13 * pi / 180;
px_approx = cos(theta_r) * px_approx + sin(theta_r) * py_approx;
py_approx = -sin(theta_r) * px_approx + cos(theta_r) * py_approx;


hold on;
axis equal;
axis square;
plot(py_approx-0.2,px_approx-1.02,'b--', "LineWidth", 2);
xlabel('X Position (m)'); 
ylabel('Y Position (m)'); 
title('Predicted v. Actual Neato Path')
axis([-2, 2, -2, 2])
hold on
legend("Expected Path", "Actual Path")
