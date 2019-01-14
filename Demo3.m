clear; clc;
clear all;
close all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%   losing 2 feature (2 points) %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

iPrexix="Demo3";

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
%u_AR = [ 1; 0; 0];                 % normal condition
%u_AR = [ 1; 0; 0.3];               % losing 1 feature (3 points)
u_AR = [ 1; 0; 0];                 % losing 2 feature (2 points)
%u_AR = [ 0.0000; 0.7071; 0.7071];  % coplanar problem
%u_AR = [-0.8348; -0.3893; 0.3893]; % local minima problem
%u_AR = [-0.9351; -0.2506; 0.2506]; % divergence problem

%theta_AR = -110*deg2rad;   % normal condition
%theta_AR = -125*deg2rad;   % losing 1 feature (3 points)
theta_AR = -130*deg2rad;   % losing 2 feature (2 points)
%theta_AR = pi;             % coplanar problem
%theta_AR = 1.7504;         % local minima problem
%theta_AR = 1.6378;         % divergence

%t_AR = [ 0.0; 0.0; 0.9 ];
t_AR = [ 0.0; 0.0; 0.8 ];  % losing 1 feature (3 points)
%t_AR = [ 0.3; 0.2; 0.8 ];  % coplanar problem
%t_AR = [ 1.0; -.10; 1.0];  % local minima problem
%t_AR = [ 1.0; -.10; 1.0];  % divergence

dq_AR = uthetat2dq( u_AR, theta_AR, t_AR ); % Dual quaternion pose
[ u_AR, theta_AR, R_AR, t_AR ] = dualq2uthetaRt( dq_AR );

X_AR = [ R_AR, t_AR; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Desired Camera location at B wrt to the fixed reference frame at R
u_BR = [ 1; 0; 0];                 % normal condition
%u_BR = [ 1; 0; 0];                 % coplanar problem
%u_BR = [-0.8348; 0.3893; -0.3893]; % local minima problem
%u_BR = [-0.8348; 0.3893; -0.3893]; % divergence problem

theta_BR = -90*deg2rad; % normal condition
%theta_BR = -pi/2;        % coplanar problem
%theta_BR = 1.7504;      % local minima problem
%theta_BR = 1.7504;      % divergence problem

t_BR = [ 0.3; 0.2; 0.8 ]; % normal condition
%t_BR = [ 0.3; 0.0; 0.8 ]; % coplanar problem
%t_BR = [ -0.5; 0.1; 1.0]; % local minima problem
%t_BR = [ -0.5; 0.1; 1.0]; % divergence problem

dq_BR = uthetat2dq( u_BR, theta_BR, t_BR ); % Dual quaternion pose
[ u_BR, theta_BR, R_BR, t_BR ] = dualq2uthetaRt( dq_BR );

X_BR = [ R_BR, t_BR; 0 0 0 1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Pattern location at C wrt to the fixed reference frame at R
u_CR = [ 1; 0; 0];        % normal condition and coplanar problem
%u_CR = [ 1; 0; 0];        % local minima problem
%u_CR = [ 1; 0; 0];        % divergence problem

theta_CR = 90*deg2rad;    % normal condition and coplanar problem
%theta_CR = 90*deg2rad;    % local minima problem
%theta_CR = 90*deg2rad;    % divergence problem

t_CR = [ 0.2; 0.9; 0.7 ]; % normal condition and coplanar problem
%t_CR = [ 0.2; 0.9; 0.9 ]; % local minima problem
%t_CR = [ 0.2; 0.9; 0.7 ]; % divergence problem

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
lambda = 5;

% let's store everything
v_all = [];
w_all = [];
err   = [];
eigen = [];
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
        Lx_temp = [ -1/Z, 0,    s_A(1,i)/Z, s_A(1,i)*s_A(2,i),        -( 1 + s_A(1,i)^2), s_A(2,i);
                    0,        -1/Z, s_A(2,i)/Z, 1 + s_A(2,i)^2,      -s_A(1,i)*s_A(2,i), -s_A(1,i)];

        Lx = [Lx; Lx_temp];
    end

    %% Error ?
    error = s_A - s_B;
    error = reshape(error(1:2,:),8,1);    
    err = [err; error'];

    %% Compute Control Law ?
    eigen = [eigen; eig(Lx'*Lx)'];
    
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
    
    fig1= figure(1);
    set(gca,'LooseInset',get(gca,'TightInset'));
    
    subplot(2,2,[1 3]); hold on; grid on;
    plot3( traces(1,:), traces(2,:), traces(3,:), 'k' );
    plot_pattern( pattern_points_CR );
    plot_camera( X_BR, 'r' );
    plot_camera( X_AR, 'b' );
    axis([-1.2 1.2 -1.2 1.2 0 1.4]);
    view( 50, 40 ); 

    %% Plot the image of the camera from locations A and B  
    subplot(2,2,4); hold on; grid on; box on;
    plot(traces_point(:,1), traces_point(:,2), 'r');
    plot(traces_point(:,3), traces_point(:,4), 'g');
    plot(traces_point(:,5), traces_point(:,6), 'b');
    plot(traces_point(:,7), traces_point(:,8), 'k');
    plot_image([p1B, p2B, p3B, p4B], 0);
    plot_image([p1A, p2A, p3A, p4A], 1);
    axis ij; axis image; xlabel('i'); ylabel('j');
    axis([ 0 1024 0 1024 ]);
    title('Camera View');
    
    %% Plot the Initial image of the camera from locations A and B
    if t == dt
       p1A_t0 = p1A;
       p2A_t0 = p2A;
       p3A_t0 = p3A;
       p4A_t0 = p4A;
    end
    if t > dt
        subplot(2,2,2); hold on; grid on; box on;
        plot_image([p1A_t0, p2A_t0, p3A_t0, p4A_t0], 1);
        axis ij; axis image; xlabel('i'); ylabel('j');
        axis([ 0 1024 0 1024 ]);
        title('Initial Camera View');       
    end
    
    drawnow;
end

%% Result
fig2 = figure(2);
set(gca,'LooseInset',get(gca,'TightInset'));

title('Control Law'); xlabel('time'); ylabel('velocity and orientation');
hold on;
grid on;
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

fig3 = figure(3);
set(gca,'LooseInset',get(gca,'TightInset'));

title('Eigenvalues of Interaction Matrix'); xlabel('time'); ylabel('velocity and orientation');
hold on;
grid on;
% velocity
plot(time_intervals, eigen(1:end,1), 'r', 'LineWidth', 2);
plot(time_intervals, eigen(1:end,2), 'g', 'LineWidth', 2);
plot(time_intervals, eigen(1:end,3), 'b', 'LineWidth', 2);
% orientation
plot(time_intervals, eigen(1:end,4), 'm--', 'LineWidth', 2);
plot(time_intervals, eigen(1:end,5), 'c--', 'LineWidth', 2);
plot(time_intervals, eigen(1:end,6), 'y--', 'LineWidth', 2);
legend('vx', 'vy', 'vz', 'wx', 'wy', 'wz');
hold off;

fig4 = figure(4);
set(gca,'LooseInset',get(gca,'TightInset'));

title('Error'); xlabel('time'); ylabel('x');
hold on;
grid on;
% error x
plot(time_intervals, err(1:end,1), 'r-', 'LineWidth', 2);
plot(time_intervals, err(1:end,3), 'g-', 'LineWidth', 2);
plot(time_intervals, err(1:end,5), 'b-', 'LineWidth', 2);
plot(time_intervals, err(1:end,7), 'k-', 'LineWidth', 2);
% error y
plot(time_intervals, err(1:end,2), 'm--', 'LineWidth', 2);
plot(time_intervals, err(1:end,4), 'c--', 'LineWidth', 2);
plot(time_intervals, err(1:end,6), 'y--', 'LineWidth', 2);
plot(time_intervals, err(1:end,8), 'r--', 'LineWidth', 2);
legend('x1', 'x2', 'x3', 'x4', 'y1', 'y2', 'y3', 'y4');
hold off;

%% Save Images
saveas(fig1,'results/'+iPrexix+'-simulation.png');
saveas(fig2,'results/'+iPrexix+'-control-law.png');
saveas(fig3,'results/'+iPrexix+'-eignen.png');
saveas(fig4,'results/'+iPrexix+'-error.png');

