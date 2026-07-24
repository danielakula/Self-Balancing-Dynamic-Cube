%% Initialisation
g = 9.81;
Cb = 0; % Damping coefficient for the body
l = sqrt((0.15/2)^2*2);
alphad = 0;

Mbody = 0.25;

% Core Parameters
M_core = 70; % g
COM_core = [51.684 51.705 51.556]; % mm
I_core = [5.719 5.717 5.713]*10^5; % g*mm^2
P_core = [-1.625 -1.628 -1.628]*10^5; % g*mm^2

% Battery Parameters
M_battery = 100; % g

% Frame Parameters
M_frame = 6*6 + 2.5*4; % g

% Motor and Reaction Wheel Parameters
Tmax = 0.25; % Max Torque Nm
Kd = [0,0,0,0];
KV = 195;
Kt = 0.05; % Nm/A
Cw = 0.05*10^-3; % Damping coefficient for the wheel
TorqueRate = 1/0.005; 

Mmotor = 0.11;
Mwheel = 0.145;
Macctuator = Mmotor + Mwheel; % kg

% PCB Parameters
M_PCB = 106.6; % g

% Discretisation 
Ts = 0.005;

%% Parameters
rb = out.rb(3);
Ib = out.Ib(5);
mw = out.mw1(1); % Mass of the wheel
mb = out.mb(1); % Mass of Body excl. wheel and motor
Iw = out.Iw1(9); % Moment of inertia of the wheel and motor

%% State Space Form
% x = [theta_b; theta_dot_b; theta_dot_w];

A = [0, 1, 0;
    (mb*rb+mw*l)*g/(Ib+mw*l^2), -Cb/(Ib+mw*l^2), Cw/(Ib+mw*l^2);
    -(mb*rb+mw*l)*g/(Ib+mw*l^2), Cb/(Ib+mw*l^2), -Cw*(Ib+Iw+mw*l^2)/(Iw*(Ib+mw*l^2))];

B = [0;
    -Kt/(Ib+mw*l^2);
    Kt*(Ib+Iw+mw*l^2)/(Iw*(Ib+mw*l^2))];

n = 3; m = 1;

Co = ctrb(A,B);
rankCo = rank(Co);

%% Poles
poles = eig(A);
Tconstant = 1/poles(1);
SF_Ts = Tconstant/ Ts;

% Low Pass Filter
alpha_poles = poles(1)/10; % cutoff frequency rad/s - poles/10

wc = 0.1;
alpha = wc*Ts/(1+wc*Ts);

alphad = alpha * Ts;

%% Augmented Matricies
% Row 4 represents: z_dot = 0*theta + 0*theta_dot + alpha*wheel_dot - alpha*z
A_aug = [A, zeros(3,1); 0, 0, alpha, -alpha];
B_aug = [B; 0];

%% Discretisation
sys_c = ss(A_aug, B_aug, eye(4), 0);

sys_d = c2d(sys_c, Ts, 'zoh');

Ad = sys_d.A;
Bd = sys_d.B;

%% Bryson's Rule
Q11 = 1/(3*pi/180)^2; % Max displacement
Q22 = 1/(2)^2; % Max cube velocity
Q33 = 1/(500*(2*pi)/60); % Max wheel velocity
Q44 = 1/(0.1)^2; % Max ideal wheel velocity

Q_bryson = diag([Q11, Q22, Q33, Q44]);
R_bryson = 1/0.05^2; % Maximum current/voltage

%% Manual Weights
Q = diag([10000, 10, 1, 1]); % theta_b; theta_dot_b; theta_dot_w, alpha
R = 10;

%% Discrete LQR
[Kd, S, P] = dlqr(Ad, Bd, Q_bryson, R_bryson);

%% 
wc_vals = logspace(-1, 1.301, 100); % 100 points between 0.1 and 20
phase_margins = zeros(length(wc_vals), 1);

% 4. RUN THE SWEEP
for i = 1:length(wc_vals)
    wc = wc_vals(i);
    
    % Build the 4x4 Augmented A Matrix
    % Filter dynamics: dot_vf = wc * v_wheel - wc * v_f
    A_aug = [ A(1,1)  A(1,2)  A(1,3)   0;
              A(2,1)  A(2,2)  A(2,3)   0;
              A(3,1)  A(3,2)  A(3,3)   0;
              0       0       wc      -wc ];
          
    % Build the Augmented B Matrix
    B_aug = [ B(1); B(2); B(3); 0 ];
    
    % Calculate the rigorous LQR gains for this specific wc
    K_aug = lqr(A_aug, B_aug, Q_bryson, R_bryson);
    
    % Create the Loop Transfer Function: L(s) = K * (sI - A)^-1 * B
    % We use MATLAB's 'ss' (state-space) command to break the loop
    sys_open_loop = ss(A_aug, B_aug, K_aug, 0);
    
    % Extract the Phase Margin
    [~, PM, ~, ~] = margin(sys_open_loop);
    
    % Store the result
    phase_margins(i) = PM;
end

% 5. PLOT THE RESULTS
figure('Name', 'Rigorous Filter Design');
semilogx(wc_vals, phase_margins, 'LineWidth', 2.5, 'Color', '#0072BD');
hold on; grid on;

% Draw the 45-degree danger line
yline(45, 'r--', 'Minimum Safe Phase Margin (45°)', 'LineWidth', 2, 'LabelHorizontalAlignment', 'left');

title('Phase Margin vs. LPF Cutoff Frequency (\omega_c)', 'FontSize', 14);
xlabel('Filter Cutoff \omega_c (rad/s)', 'FontSize', 12);
ylabel('Phase Margin (Degrees)', 'FontSize', 12);
xlim([0.1 20]);
ylim([20 100]);