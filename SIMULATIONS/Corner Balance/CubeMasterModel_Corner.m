%% Choice Parameters

g = 9.81;

% Core Parameters
M_core = 70; % g including peripheral PCBs, fasteners, and cables
COM_core = [51.684 51.705 51.556]; % mm
I_core = [5.719 5.717 5.713]*10^5; % g*mm^2
P_core = [-1.625 -1.628 -1.628]*10^5; % g*mm^2

% Battery Parameters
M_battery = 100; % g

% Frame Parameters
M_frame = 6*6 + 2.5*4; % g including fasteners and corner joints

% Motor and Reaction Wheel Parameters
Tmax = 0.25; % Max Torque Nm
Kd = [0,0,0];
KV = 195;
Km = 0.05; % Nm/A
Cw = 0.05*10^-3; % Damping coefficient for the wheel
TorqueRate = 1/0.005; 
Motor_Resistance = (0.54+0.535+0.535)/3;

M_motor = 110;
M_wheel = 145;

% PCB Parameters
M_PCB = 106.6; % g

% Mass Verification
M_total = M_PCB + 1000*Macctuator*3 + M_frame + M_battery + M_core;

% Moment of Inertia
J_rw = 4.721e-4; % kgm^2
J_motor = 7.4e-8; % kgm^2
J_acctuator = J_rw+J_motor; % kgm^2

% Noise
Encoder_np = 2e-10;
Encoder_st = 9e-5;
IMU_acc_np = 1.73e-7;
IMU_gyro_np = 3.81e-9;
IMU_st = 0.005;

TorqueRate = 0.1/0.005;

% Discretisation 
Ts = 0.005;
alpha = 0;

%% Initialisation
K = zeros(3,8);
A0 = 0; B0 = atan(-1/sqrt(2)); Y0 = pi/4; % Angles for equilibrium
T = zeros(9);

%% Simscape Parameters 
I = out.Ib(1:3,1:3); % Cube Inertia Tensor = Icube - Mcube * Rcube^2 + sum(Iwi - Mw * Rwi^2)
Iw1 = out.Iw1(9); Iw2 = out.Iw2(9); Iw3 = out.Iw3(9); % Wheel Inertia Tensors
Iw = diag([Iw1, Iw2, Iw3]); % Wheel Useful Inertia Tensors
Ihat = I - Iw;
M = out.mb(1)*skewSym(out.rb(1:3,1)) + out.mw1(1)*skewSym(out.rw1(1:3,1)) + out.mw2(1)*skewSym(out.rw2(1:3,1)) + out.mw3(1)*skewSym(out.rw3(1:3,1)); % Moments

%% Linearised Dynamics
pg = [0, cos(B0), 0;
    0, sin(B0)*sin(Y0), -cos(B0)*cos(Y0);
    0, sin(B0)*cos(Y0), cos(B0)*sin(Y0)];

F = [0, sin(Y0)/cos(B0), cos(Y0)/cos(B0);
    0, cos(Y0), -sin(Y0);
    1, sin(Y0)*sin(B0)/cos(B0), cos(Y0)*sin(B0)/cos(B0)];

%% xdot = Ax + Bu
% State Vector: x = [phi_hat; v_hat; w_hat]

Ahat = [zeros(3,3), F, zeros(3,3);
    inv(Ihat)*M*pg, zeros(3,3), Cw*inv(Ihat);
    -inv(Ihat)*M*pg, zeros(3,3), -Cw*(inv(Ihat)+inv(Iw))];

Bhat = [zeros(3,3);
    -inv(Ihat)*Km;
    (inv(Ihat)+inv(Iw))*Km];

%% Poles
poles = eig(Ahat);
Tconstant = 1/poles(2);

%% Controller
% Cannonical coordinate transformation
C_dummy = eye(9);
[Abar, Bbar, Cbar, T, k] = ctrbf(Ahat, Bhat, C_dummy);
n_c = sum(k);
A_c = Abar(2:9, 2:9);
B_c = Bbar(2:9, 1:3);
rank_check = rank(ctrb(A_c,B_c));

n = 8; m = 3;
rankCohat = rank(ctrb(Ahat,Bhat));

%% Discretisation
sys_c = ss(A_c, B_c, eye(8), 0);

sys_d = c2d(sys_c, Ts, 'zoh');

Ad = sys_d.A;
Bd = sys_d.B;

%% Bryson's Rule
Q_angle = 1/(0.5*pi/180)^2;
Q_velocity = 1/(2)^2;
Q_wheel = 1/(500*2*pi/60)^2;
Q_b = diag([1e-6, Q_angle, Q_angle, Q_velocity, Q_velocity, Q_velocity, Q_wheel, Q_wheel, Q_wheel]);
Q_br = T*Q_b*T';
Q_bryson = Q_br(2:9,2:9);
R_bryson = eye(3)*1/(0.5)^2;

%% LQR 
Qu = diag([0.000001, 100, 100, 10, 10, 10, 0.1, 0.1, 0.1]); % yaw, pitch, roll, vx, vy, vz, w1, w2, w3
Qc = T*Qu*T';
Q = Qc(2:9,2:9);
R = eye(3)*0.1;

[K, S, P] = dlqr(Ad, Bd, Q_bryson, R_bryson);

%% Plots
close all;

% 1. GLOBAL DEFAULTS (Supervisor-friendly fonts & White background)
set(groot, 'defaultFigureColor', [1 1 1]);
set(groot, 'defaultAxesColor', [1 1 1]);
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultAxesFontSize', 14); 

% 2. DATA EXTRACTION & DIMENSION ALIGNMENT
pos_data = squeeze(out.Position.Data);
if size(pos_data, 1) == 3, pos_data = pos_data'; end
time_pos = out.Position.Time;

wheel_data = squeeze(out.Wheel_Velocity.Data);
if size(wheel_data, 1) == 3, wheel_data = wheel_data'; end
time_wheel = out.Wheel_Velocity.Time;

cube_vel_data = squeeze(out.Cube_Velocity.Data);
if size(cube_vel_data, 1) == 3, cube_vel_data = cube_vel_data'; end
time_cube_vel = out.Cube_Velocity.Time;

torque_data = squeeze(out.Torque.Data);
if size(torque_data, 1) == 3, torque_data = torque_data'; end
time_torque = out.Torque.Time;

% 3. FIGURE CREATION
fig = figure('Name', '3D Corner Balancing Dynamics', ...
             'Units', 'centimeters', ...
             'Position', [2, 2, 22, 16], ... % Slightly wider for large fonts/legends
             'InvertHardcopy', 'off', ...
             'Color', 'w');

t = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% --- PROFESSIONAL COLOUR PALETTE (Publication Standard) ---
c1 = [0.00, 0.45, 0.74]; % Dark Blue
c2 = [0.85, 0.33, 0.10]; % Deep Orange
c3 = [0.40, 0.40, 0.40]; % Slate Grey (Clearer than green in B&W)

% --- ACCESSIBILITY LINE STYLES ---
s1 = '-';   % Solid
s2 = '--';  % Dashed
s3 = ':';   % Dotted

% -------------------------------------------------------------------------
% TILE 1: Cube Orientation (Angular States)
% -------------------------------------------------------------------------
ax1 = nexttile;
hold on;
plot(time_pos, pos_data(:,1), 'LineStyle', s1, 'Color', c1, 'LineWidth', 1.5);
plot(time_pos, pos_data(:,2), 'LineStyle', s2, 'Color', c2, 'LineWidth', 1.5);
plot(time_pos, pos_data(:,3), 'LineStyle', s3, 'Color', c3, 'LineWidth', 2.0);
hold off;
grid on; box on;
title('A: Cube Orientation $\phi$', 'FontSize', 16);
ylabel('Angle (rad)');
legend('Yaw $\alpha$', 'Pitch $\beta$', 'Roll $\gamma$', 'Location', 'northeast');

% -------------------------------------------------------------------------
% TILE 2: Reaction Wheel Velocities (Actuator States)
% -------------------------------------------------------------------------
ax2 = nexttile;
hold on;
plot(time_wheel, wheel_data(:,1), 'LineStyle', s1, 'Color', c1, 'LineWidth', 1.5);
plot(time_wheel, wheel_data(:,2), 'LineStyle', s2, 'Color', c2, 'LineWidth', 1.5);
plot(time_wheel, wheel_data(:,3), 'LineStyle', s3, 'Color', c3, 'LineWidth', 2.0);
hold off;
grid on; box on;
title('B: Reaction Wheel Velocities $w$', 'FontSize', 16);
ylabel('Velocity (rad/s)');
legend('Wheel 1', 'Wheel 2', 'Wheel 3', 'Location', 'northeast');

% -------------------------------------------------------------------------
% TILE 3: Cube Angular Velocity (Derivative States)
% -------------------------------------------------------------------------
ax3 = nexttile;
hold on;
plot(time_cube_vel, cube_vel_data(:,1), 'LineStyle', s1, 'Color', c1, 'LineWidth', 1.5);
plot(time_cube_vel, cube_vel_data(:,2), 'LineStyle', s2, 'Color', c2, 'LineWidth', 1.5);
plot(time_cube_vel, cube_vel_data(:,3), 'LineStyle', s3, 'Color', c3, 'LineWidth', 2.0);
hold off;
grid on; box on;
title('C: Cube Body Velocity $\dot{\phi}$', 'FontSize', 16);
ylabel('Velocity (rad/s)');
xlabel('Time (s)');
legend('Yaw $\dot{\alpha}$', 'Pitch $\dot{\beta}$', 'Roll $\dot{\gamma}$', 'Location', 'northeast');

% -------------------------------------------------------------------------
% TILE 4: Control Torques (MIMO Control Output)
% -------------------------------------------------------------------------
ax4 = nexttile;
hold on;
plot(time_torque, torque_data(:,1), 'LineStyle', s1, 'Color', c1, 'LineWidth', 1.5);
plot(time_torque, torque_data(:,2), 'LineStyle', s2, 'Color', c2, 'LineWidth', 1.5);
plot(time_torque, torque_data(:,3), 'LineStyle', s3, 'Color', c3, 'LineWidth', 2.0);
hold off;
grid on; box on;
title('D: Control Torques $u$', 'FontSize', 16);
ylabel('Torque (N$\cdot$m)');
xlabel('Time (s)');
legend('Motor 1', 'Motor 2', 'Motor 3', 'Location', 'northeast');

% -------------------------------------------------------------------------
% LINK AXES & EXPORT
% -------------------------------------------------------------------------
linkaxes([ax1, ax2, ax3, ax4], 'x');

% Export to vector PDF
exportgraphics(fig, '3D_Corner_Balance_Results.pdf', 'ContentType', 'vector');
fprintf('Visualization Complete. "3D_Corner_Balance_Results.pdf" has been generated.\n');

%% Skew-Symmetric Matrix Function
function S = skewSym(v)
    S = [0, -v(3), v(2);
         v(3), 0, -v(1);
         -v(2), v(1), 0];
end