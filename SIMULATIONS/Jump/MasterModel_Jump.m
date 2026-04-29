% Initialisation
g = 9.81;
Cw = 0.05*10^-3;

% Motor Torque
Tmax = 0.75;
wmax = 3120*2*pi/60;
TorqueRate = 1/0.005;

OutputTorque = [Tmax,Tmax,Tmax,Tmax,Tmax,0.72,0.64,0.56,0.48,0.4,0.32,0.24,0.16,0.08,0]';
RPM = [-2100,-1000,-0.1,0,1000,1050,1200,1300,1500,1700,2000,2100,2400,2600,3120]'*2*pi/60;

% Start and End Velocities
wstart = -2000*2*pi/60;
wend = 500*2*pi/60;

% Masses
M_core = 70.3;
M_battery = 100.7;
M_wheel = 143;
M_motor = 112;
M_PCB = 106.6;
M_frame = 46;

M_total = M_core + Macctuator*3 + M_battery + M_PCB + M_frame;

% Moment of Inertia
J_rw = 4.721e-4; % kgm^2
J_motor = 7.4e-8; % kgm^2
J_acctuator = J_rw+J_motor; % kgm^2

%% Plot of Motor Torque Speed

% 1. Define your system constants
Kv = 195; % RPM/V
R_phase = 0.5; % Ohms (Replace with your actual measured resistance)
I_target = 15; % Amps
Tmax = 0.75; % Nm
T_cont = 0.25; % Continuous Torque Limit (Nm)

% Calculate the exact transition point for plug braking
RPM_plug_val = I_target * R_phase * Kv; 

% 2. Define your Exact Data Vectors
RPM_raw = [-3120, -2100, -1000, -0.1, 0, 1000, 1050, 1200, 1300, 1500, 1700, 2000, 2100, 2400, 2600, 3120]';
OutputTorque = [Tmax, Tmax, Tmax, Tmax, Tmax, Tmax, 0.72, 0.64, 0.56, 0.48, 0.4, 0.32, 0.24, 0.16, 0.08, 0]';

% 3. Smooth the Curve using PCHIP Interpolation
% This generates 500 points for a perfectly smooth curve without overshooting
RPM_smooth = linspace(min(RPM_raw), max(RPM_raw), 500);
Torque_smooth = interp1(RPM_raw, OutputTorque, RPM_smooth, 'pchip');

% 4. Create the Figure
figure('Color', 'w');
hold on;

% 5. Add the Intermittent Operating Zones (Shaded Regions)
% High Speed Intermittent Zones (Vertical)
patch([-3120 -2100 -2100 -3120], [0 0 1 1], 'r', 'FaceAlpha', 0.05, 'EdgeColor', 'none', 'HandleVisibility', 'off');
patch([2100 3120 3120 2100], [0 0 1 1], 'r', 'FaceAlpha', 0.05, 'EdgeColor', 'none', 'HandleVisibility', 'off');

% High Torque Intermittent Zone (Horizontal, > 0.25 Nm)
patch([-3120 3120 3120 -3120], [T_cont T_cont 1 1], 'r', 'FaceAlpha', 0.05, 'EdgeColor', 'none', 'HandleVisibility', 'off');

% Add Zone Labels (with white backgrounds to prevent line overlap)
text(-2700, 0.5, 'Intermittent Speed', 'Interpreter', 'latex', 'FontSize', 14, 'Rotation', 90, 'Color', [0.6 0 0], 'HorizontalAlignment', 'center', 'Margin', 1);
text(2700, 0.5, 'Intermittent Speed', 'Interpreter', 'latex', 'FontSize', 14, 'Rotation', 90, 'Color', [0.6 0 0], 'HorizontalAlignment', 'center', 'Margin', 1);
text(-1700, 0.22, 'Intermittent Torque ($>0.25$ Nm)', 'Interpreter', 'latex', 'FontSize', 14, 'Color', [0.6 0 0], 'HorizontalAlignment', 'center', 'Margin', 1);

text(1000, 0.12, 'Continuous Safe Zone', 'Interpreter', 'latex', 'FontSize', 14, 'Color', [0.2 0.6 0.2], 'HorizontalAlignment', 'center', 'Margin', 1);

% 6. Plot the SMOOTHED operating envelope
plot(RPM_smooth, Torque_smooth, '-', 'Color', '#0072BD', 'LineWidth', 2.5, 'DisplayName', 'GL40 Dynamic Envelope');

% 7. Annotate the Electrical Transition Zones
% Draw the vertical line exactly at the calculated plug braking RPM
xline(-RPM_plug_val, '--k', 'LineWidth', 1.5, 'HandleVisibility', 'off');

% Add the Equation exactly on the line
text(-RPM_plug_val + 120, 0.35, '$\omega_{plug} \approx I_{max} R_{phase} K_v$', 'Interpreter', 'latex', 'FontSize', 14, 'Rotation', 90);

% Position text for the braking regions
text(-2300, 0.5, 'Regen \& Chopper Active', 'Interpreter', 'latex', 'FontSize', 14, 'Rotation', 90, 'HorizontalAlignment', 'center', 'Margin', 2);
text(-RPM_plug_val/2, 0.5, 'Plug \& Regen', 'Interpreter', 'latex', 'FontSize', 14, 'HorizontalAlignment', 'center', 'Rotation', 90, 'Margin', 2);

text(-700, 0.05, 'Braking Side', 'Interpreter', 'latex', 'FontSize', 14, 'HorizontalAlignment', 'center', 'Margin', 2);
text(700, 0.05, 'Driving Side', 'Interpreter', 'latex', 'FontSize', 14, 'HorizontalAlignment', 'center', 'Margin', 2);

% 8. Add the Peak Continuous and Impulse Labels
yline(Tmax, '--r', 'Peak Impulse Rating (0.75 Nm)', ...
    'LineWidth', 1.5, 'LabelHorizontalAlignment', 'right', ...
    'LabelVerticalAlignment', 'top', 'Interpreter', 'latex', ...
    'FontSize', 14, 'HandleVisibility', 'off');

yline(T_cont, '--', ...
    'Color', [0.6 0 0], 'LineWidth', 1.5, 'LabelHorizontalAlignment', 'left', ...
    'LabelVerticalAlignment', 'bottom', 'Interpreter', 'latex', ...
    'FontSize', 12, 'HandleVisibility', 'off');

% 9. Format Axes and Grid
grid on; grid minor;
ax = gca;
ax.TickLabelInterpreter = 'latex';
ax.FontSize = 12;
xline(0, 'k', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(0, 'k', 'LineWidth', 1, 'HandleVisibility', 'off');

% 10. Labels and Limits
xlabel('Wheel Velocity, $\omega$ (RPM)', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('Available Torque, $\tau$ (Nm)', 'Interpreter', 'latex', 'FontSize', 14);
title('CubeMars GL40 Dynamic Operating Envelope', 'Interpreter', 'latex', 'FontSize', 14);
xlim([-3120 3120]);
ylim([0 0.95]);

% 11. Legend
leg = legend('Location', 'northeast');
leg.Interpreter = 'latex';
leg.FontSize = 14;


%% Plot of Jump

% 1. DATA EXTRACTION
TimeArr  = out.tout;
TauData  = out.Torque.Data;        
VelData  = out.Cube_Velocity.Data; 
WheelVel = out.Wheel_Velocity.Data;
Angle    = out.Position.Data*180/pi -45; % Replace with your actual Angle variable

% Define the Jump Timing
t_start = 2.0;
t_end   = 2.171; % Example: 171ms
dt_ms   = round((t_end - t_start) * 1000); % Convert to integer for clean display

idx_cross = find(TimeArr >= t_start & Angle >= 0, 1);
t_cross = TimeArr(idx_cross);
dt_rise_ms = round((t_cross - t_start) * 1000);

% 2. CREATE THE WIDE FIGURE
figure('Color', 'w', 'Position', [100, 100, 1000, 420]); 

% ==========================================
% LEFT PANEL: CHASSIS KINEMATICS (Angle & Velocity)
% ==========================================
ax1 = subplot(1, 2, 1);
hold on; grid on; grid minor;

% Left Y-Axis: Cube Angle (SOLID BLACK)
yyaxis left;
plot(TimeArr, Angle, 'k-', 'LineWidth', 2, 'DisplayName', 'Angle $\phi$');
ylabel('Pitch Angle, $\phi$ ($^\circ$)', 'Interpreter', 'latex', 'FontSize', 14);
yline(0, '--k', 'Equilibrium', 'Interpreter', 'latex', 'LabelHorizontalAlignment', 'left', 'Alpha', 0.5, 'FontSize', 14);
ylim([-50 50]);
ax1.YColor = 'k';

plot([t_start, t_cross], [15, 15], 'k-', 'LineWidth', 1);
plot([t_start, t_start], [12, 18], 'k-', 'LineWidth', 1);
plot([t_cross, t_cross], [12, 18], 'k-', 'LineWidth', 1);
text(mean([t_start, t_cross]), 18, sprintf('$t_{rise} = %d$ ms', dt_rise_ms), ...
    'Interpreter', 'latex', 'FontSize', 14, 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'bottom', 'Margin', 1);

% Right Y-Axis: Cube Velocity (DASHED GREY)
yyaxis right;
plot(TimeArr, VelData, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5, 'DisplayName', 'Velocity $\dot{\phi}$');
ylabel('Angular Velocity, $\dot{\phi}$ (rad/s)', 'Interpreter', 'latex', 'FontSize', 14);
ax1.YColor = [0.4 0.4 0.4];

xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14);
title('Chassis Kinematics', 'Interpreter', 'latex', 'FontSize', 14);
xlim([1.5 3]); 
ax1.TickLabelInterpreter = 'latex';


% ==========================================
% RIGHT PANEL: ACTUATOR KINETICS (Torque & Wheel Vel)
% ==========================================
ax2 = subplot(1, 2, 2);
hold on; grid on; grid minor;

% Left Y-Axis: Motor Torque (SOLID BLACK)
yyaxis left;
plot(TimeArr, TauData, 'k-', 'LineWidth', 2); % Changed to solid black
ylabel('Motor Torque, $\tau$ (Nm)', 'Interpreter', 'latex', 'FontSize', 14);
ylim([-0.1 1.05]); 
ax2.YColor = 'k'; % Changed axis color to black

% Draw the Dimension Line for Jump Duration
plot([t_start, t_end], [0.92, 0.92], 'k-', 'LineWidth', 1, 'HandleVisibility', 'off');
plot([t_start, t_start], [0.89, 0.95], 'k-', 'LineWidth', 1, 'HandleVisibility', 'off');
plot([t_end, t_end], [0.89, 0.95], 'k-', 'LineWidth', 1, 'HandleVisibility', 'off');
text(mean([t_start, t_end]), 0.95, sprintf('$\\Delta t = %d$ ms', dt_ms), ...
    'Interpreter', 'latex', 'FontSize', 14, 'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'bottom', 'Margin', 1);

% Right Y-Axis: Wheel Velocity (DASHED GREY)
yyaxis right;
plot(TimeArr, WheelVel, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 2); % Changed to dashed grey
ylabel('Wheel Velocity, $\omega_w$ (rad/s)', 'Interpreter', 'latex', 'FontSize', 14);
ax2.YColor = [0.4 0.4 0.4]; % Changed axis color to grey to match line

xlabel('Time (s)', 'Interpreter', 'latex', 'FontSize', 14);
title('Actuator Kinetics', 'Interpreter', 'latex', 'FontSize', 14);
xlim([1.5 3]); 
ax2.TickLabelInterpreter = 'latex';

linkaxes([ax1, ax2], 'x');
sgtitle('Open-Loop Jump Momentum Transfer', 'Interpreter', 'latex', 'FontSize', 15);