clear; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
deg2rad = pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Camera Intrinsic Matrix
fx = 800; % focal length (mm) / pixel_width_x (mm) = pixels 
fy = 800; % focal length (mm) / pixel_height_y (mm) = pixels
uo = 512; % image center x-coordinate
vo = 512; % image center y-coordinate

K = [ fx,  0, uo;
       0, fy, vo;
       0,  0,  1 ];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Current Camera location at A wrt to the fixed reference frame at R
u_AR = [ 1; 0; 0];         % u (orientation axis)
theta_AR = -110*deg2rad;   % theta (orientation angle)
t_AR = [ 0.0; 0.0; 0.9 ];  % position

dq_AR = uthetat2dq( u_AR, theta_AR, t_AR ); % Dual quaternion pose
[ u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt( dq_AR );

X_AR = [ R_AR, t_AR; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired Camera location at B wrt to the fixed reference frame at R
u_BR = [ 1; 0; 0];         % u (orientation axis)
theta_BR = -90*deg2rad;    % theta (orientation angle)
t_BR = [ 0.3; 0.2; 0.8 ]; % position

dq_BR = uthetat2dq( u_BR, theta_BR, t_BR ); % Dual quaternion pose
[ u_BR, theta_BR, R_BR, t_BR ] = dualq2uthetaRt( dq_BR );

X_BR = [ R_BR, t_BR; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Pattern location at C wrt to the fixed reference frame at R
u_CR = [ 1; 0; 0];        % u (orientation axis)
theta_CR = 90*deg2rad;    % theta (orientation angle)
t_CR = [ 0.2; 0.9; 0.7 ]; % position

dq_CR = uthetat2dq( u_CR, theta_CR, t_CR ); % Dual quaternion pose 
[ u_CR, theta_CR, R_CR, t_CR ] = dualq2uthetaRt( dq_CR );

X_CR = [ R_CR, t_CR; 0 0 0 1];

pattern_points_CR = put_pattern( X_CR ); % 3D pattern points

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired Image of the Pattern from location B ?

% Take the points from pattern_points_CR (first 4 points)
P1 = pattern_points_CR(:,1);
P2 = pattern_points_CR(:,2);
P3 = pattern_points_CR(:,3);
P4 = pattern_points_CR(:,4);

% Scene points in the camera frame
P_B = inv(X_BR)*[ P1, P2, P3, P4; 1 1 1 1 ];
P1B = P_B(1:3,1);
P2B = P_B(1:3,2);
P3B = P_B(1:3,3);
P4B = P_B(1:3,4);

% camera frame to image plane
ZpB = K*[P1B,P2B,P3B,P4B];

% normalize
p1B = ZpB(1:2,1)/ZpB(3,1);
p2B = ZpB(1:2,2)/ZpB(3,2);
p3B = ZpB(1:2,3)/ZpB(3,3);
p4B = ZpB(1:2,4)/ZpB(3,4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired image point features vector s_B from location B in metric units ?
s_B = inv(K) * [p1B, p2B, p3B, p4B ; 1 1 1 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LOOP SHOULD START FROM HERE
dt = 1/50;
tf = 2;
Hz = 50;

% let's store everything
v_all = [];
w_all = [];
err   = [];
time_intervals = [];

traces = [];
traces_point = [];
 
for t=0:dt:tf 
    %% Take an image from location A ?

    % Scene points in the camera frame
    P_A = inv(X_AR)*[ P1, P2, P3, P4; 1 1 1 1 ];
    P1A = P_A(1:3,1);
    P2A = P_A(1:3,2);
    P3A = P_A(1:3,3);
    P4A = P_A(1:3,4);

    % camera frame to image plane
    ZpA = K*[P1A,P2A,P3A,P4A];

    % normalize
    p1A = ZpA(1:2,1)/ZpA(3,1);
    p2A = ZpA(1:2,2)/ZpA(3,2);
    p3A = ZpA(1:2,3)/ZpA(3,3);
    p4A = ZpA(1:2,4)/ZpA(3,4);
    
    traces_point = [ traces_point; p1A' p2A' p3A' p4A'];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Build the point features vector s_A in metric units ?
    s_A  = inv(K) * [p1A, p2A, p3A, p4A ; 1 1 1 1];

    %% Calculate interaction matrix ?
    Z = mean(P_B(3,:));
    Lx = [];

    for i=1:4
        Lx_temp = [ -1/Z, 0,    s_A(1,i)/Z, s_A(1,i)*s_A(2,i), -( 1 + s_A(1,i)^2), s_A(2,i);
                    0,    -1/Z, s_A(2,i)/Z, 1 + s_A(2,i)^2,      -s_A(1,i)*s_A(2,i), -s_A(1,i)];

        Lx = [Lx; Lx_temp];
    end

    %% Error ?
    error = s_A - s_B;
    error = reshape(error(1:2,:),8,1);    
    err = [err; error'];

    %% Compute Control Law ?
    lambda = 5;
    L_psudoinverse = inv(Lx'*Lx)*Lx';
    controllaw_AA = -lambda*L_psudoinverse*error;

    %% Convert Control Law ?
    controllaw_AR = [ R_AR, skew(t_AR)*R_AR; zeros(3,3), R_AR] * controllaw_AA;

    %% Move Camera ?
    v = controllaw_AR(1:3); v_all = [v_all; v'];
    w = controllaw_AR(4:6); w_all = [w_all; w'];
    
    time_intervals = [time_intervals; t];
    
    theta = norm(w);
    if( theta == 0 ) u = [0;0;1]; else u = w/norm(w); end

    update_dq_AR = uthetat2dq( u, dt*theta, dt*v );
    dq_AR = muldualpq( update_dq_AR, dq_AR );

    [ u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt( dq_AR );
    X_AR = [R_AR, t_AR; 0 0 0 1];
    
    traces = [ traces, t_AR ]; % the trajectory of the camera frame

    %% Plot the 3D scene wrt to the reference frame at R  
    clf;
    
    figure(1); 
    subplot(1,2,1); hold on;
    plot3( traces(1,:), traces(2,:), traces(3,:), 'k' );
    plot_pattern( pattern_points_CR );
    plot_camera( X_BR, 'r' );
    plot_camera( X_AR, 'b' );
    axis([-1.2 1.2 -1.2 1.2 0 1.4]);
    view( 50, 40 ); 

    %% Plot the image of the camera from locations A and B  
    subplot(1,2,2); hold on; box on;
    plot(traces_point(:,1), traces_point(:,2), 'r');
    plot(traces_point(:,3), traces_point(:,4), 'g');
    plot(traces_point(:,5), traces_point(:,6), 'b');
    plot(traces_point(:,7), traces_point(:,8), 'k');
    plot_image([p1B, p2B, p3B, p4B], 0);
    plot_image([p1A, p2A, p3A, p4A], 1);
    axis ij; axis image; xlabel('i'); ylabel('j');
    axis([ 0 1024 0 1024 ]);
    title('Camera View');
    
    drawnow;
end

%% Result
figure(2); 
subplot(2,2,1); title('Control Law (velocity)'); xlabel('time'); ylabel('velocity');
hold on;
plot(time_intervals, v_all(1:end,1), 'r');
plot(time_intervals, v_all(1:end,2), 'g');
plot(time_intervals, v_all(1:end,3), 'b');
hold off;

subplot(2,2,2); title('Control Law (orientation)'); xlabel('time'); ylabel('orientation');
hold on;
plot(time_intervals, w_all(1:end,1), 'r');
plot(time_intervals, w_all(1:end,2), 'g');
plot(time_intervals, w_all(1:end,3), 'b');
hold off;

subplot(2,2,3); title('Error (x)'); xlabel('time'); ylabel('x');
hold on;
plot(time_intervals, err(1:end,1), 'r');
plot(time_intervals, err(1:end,3), 'g');
plot(time_intervals, err(1:end,5), 'b');
plot(time_intervals, err(1:end,7), 'k');
hold off;

subplot(2,2,4); title('Error (y)'); xlabel('time'); ylabel('y');
hold on;
plot(time_intervals, err(1:end,2), 'r');
plot(time_intervals, err(1:end,4), 'g');
plot(time_intervals, err(1:end,6), 'b');
plot(time_intervals, err(1:end,8), 'k');
hold off;
