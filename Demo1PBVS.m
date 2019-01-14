clear;
clc;
close all;
deg2rad = pi/180;

%% Initial Camera Pose at A wrt to the fixed frame R
% s-pace
u_AR = [ 1; 0; 0];          % u (orientation axis)
theta_AR = -110*deg2rad;    % theta (orientation angle)
t_AR = [ -0.1; -0.1; 0.9 ]; % position

% dq-space
dq_AR = uthetat2dq( u_AR, theta_AR, t_AR );  % dual quaternion

% C-space
[ u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt( dq_AR );
X_AR = [ R_AR, t_AR ; 0 0 0 1];  % Cartesian space pose

%% Desired Camera Pose at B wrt to the fixed frame R
% s-pace
u_BR = [ 1; 0; 0];         % u (orientation axis)
theta_BR = -90*deg2rad;    % theta (orientation angle)
t_BR = [ 0.3; 0.2; 0.8 ];  % position

% dq-space   
dq_BR = uthetat2dq( u_BR, theta_BR, t_BR );  % dual quaternion

% C-space
[ u_BR, theta_BR, R_BR, t_BR ] = dualq2uthetaRt( dq_BR );
X_BR = [ R_BR, t_BR ; 0 0 0 1];  % Cartesian space pose

%% Something
traces = [];

time = 0;  % current time
tf = 1; % final time 
dt = 0.005; % control sampling time

figure;
v_all = [];
w_all = [];
err   = [];
time_intervals = [];

while( time < tf )
    %% Error
    % e = inv( pose_desired )*pose;
    error_dq_AB = muldualpq( conjdualqsimple( dq_BR ),  dq_AR ); 
    [ u_AB, theta_AB, R_AB, t_AB ] = dualq2uthetaRt( error_dq_AB );
    
    err = [err; error_dq_AB'];

    %% Control Law
    lambda = 5;
    v = -lambda * R_AB' * t_AB;
    w = -lambda * theta_AB * u_AB;
    control_law_AB = [v; w];

    %% Convert Control Law 
    T_BR = skew(t_BR);

    control_law_AR = [R_BR, T_BR*R_BR; zeros(3,3), R_BR] * control_law_AB;

    %% Move Camera 
    v = control_law_AR(1:3); v_all = [v_all; v'];
    w = control_law_AR(4:6); w_all = [w_all; w'];
    
    theta = norm(w);
    if( theta == 0 ) u = [0;0;1]; else u = w/norm(w); end

    update_dq_AR = uthetat2dq( u, dt*theta, dt*v );
    dq_AR = muldualpq( update_dq_AR, dq_AR );

    [ u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt( dq_AR );
    X_AR = [R_AR, t_AR; 0 0 0 1];

    %% Plot Scene 
    time = time + dt;
    time_intervals = [time_intervals; time];
    traces = [ traces, t_AR ]; % the trajectory of the camera frame
    
    clf; hold on; grid on;
    plot3( traces(1,:), traces(2,:), traces(3,:), 'k' );

    plot_pose( X_AR, 'k'  );  plot_camera( X_AR, 'b' );
    plot_pose( X_BR, 'k'  );  plot_camera( X_BR, 'r' );

    axis([-1.2 1.2 -1.2 1.2 0 1.4]);
    view( 64, 34 ); 
    title( num2str(time) );
    
    drawnow;
end

% %% Result
% figure(2); 
% subplot(1,2,1); title('Control Law (velocity)'); xlabel('time'); ylabel('velocity'); xlim([0 1]);
% hold on;
% plot(time_intervals, v_all(1:end,1), 'r');
% plot(time_intervals, v_all(1:end,2), 'g');
% plot(time_intervals, v_all(1:end,3), 'b');
% hold off;
% 
% subplot(1,2,2); title('Control Law (orientation)'); xlabel('time'); ylabel('orientation'); xlim([0 1]);
% hold on;
% plot(time_intervals, w_all(1:end,1), 'r');
% plot(time_intervals, w_all(1:end,2), 'g');
% plot(time_intervals, w_all(1:end,3), 'b');
% hold off;

%% Result
figure(2); 
title('Control Law'); xlabel('time'); ylabel('velocity and orientation');
hold on; grid on;
% velocity
plot(time_intervals, v_all(1:end,1), 'r', 'LineWidth', 2);
plot(time_intervals, v_all(1:end,2), 'g', 'LineWidth', 2);
plot(time_intervals, v_all(1:end,3), 'b', 'LineWidth', 2);
% orientation
plot(time_intervals, w_all(1:end,1), 'm--', 'LineWidth', 2);
plot(time_intervals, w_all(1:end,2), 'c--', 'LineWidth', 2);
plot(time_intervals, w_all(1:end,3), 'y--', 'LineWidth', 2);
legend('vx', 'vy', 'vz', 'wx', 'wy', 'wz');
hold off;

figure(3);
title('Error'); xlabel('time'); ylabel('x');
hold on; grid on;
plot(time_intervals, err(1:end,1), 'r-', 'LineWidth', 2);
plot(time_intervals, err(1:end,3), 'g-', 'LineWidth', 2);
plot(time_intervals, err(1:end,5), 'b-', 'LineWidth', 2);
plot(time_intervals, err(1:end,7), 'k-', 'LineWidth', 2);
plot(time_intervals, err(1:end,2), 'm--', 'LineWidth', 2);
plot(time_intervals, err(1:end,4), 'c--', 'LineWidth', 2);
plot(time_intervals, err(1:end,6), 'y--', 'LineWidth', 2);
plot(time_intervals, err(1:end,8), 'r--', 'LineWidth', 2);
legend('r1', 'r2', 'r3', 'r4', 'd1', 'd2', 'd3', 'd4');
hold off;


