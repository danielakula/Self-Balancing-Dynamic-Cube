%% Initialisation
g = 9.81;
Tmax = 0.75; % Max Torque Nm
K = [0,0,0];
KV = 210;
Km = 1/(KV)*sqrt(5.7); % Motor constant
Cw = 0.05*10^-3; % Damping coefficient for the wheel
Cb = 0; % Damping coefficient for the body
l = sqrt((0.15/2)^2*2);
Mbody = 0.4;
Mmotor = 0.11;
Mwheel = 0.14;
Macctuator = Mmotor + Mwheel;

%% Parameters
rb = out.rb(3);
Ib = out.Ib(5);
mw = out.mw1(1,1); % Mass of the wheel
mb = out.mb(1);
Iw = out.Iw1(9); % Moment of inertia of the wheel

%% State Space Form
% x = [theta_b; theta_dot_b; theta_dot_w];

A = [0, 1, 0;
    (mb*rb+mw*l)*g/(Ib+mw*l^2), -Cb/(Ib+mw*l^2), Cw/(Ib+mw*l^2);
    -(mb*rb+mw*l)*g/(Ib+mw*l^2), Cb/(Ib+mw*l^2), -Cw*(Ib+Iw+mw*l^2)/(Iw*(Ib+mw*l^2))];

B = [0;
    -Km/(Ib+mw*l^2);
    Km*(Ib+Iw+mw*l^2)/(Iw*(Ib+mw*l^2))];

n = 3; m = 1;

Co = ctrb(A,B);
rankCo = rank(Co);

%% Bryson's Rule
Q11 = 1/(7*pi/180)^2;
Q22 = 1/(0.79)^2;
Q33 = 1/(100)^2 + 10;
R_bryson = 1/(Tmax)^2;

%% LQR
Q = diag([100, 1, 1]); % theta_b; theta_dot_b; theta_dot_w
Q_bryson = diag([Q11, Q22, Q33]);
R = 0.1;

[K, S, P] = lqr(A, B, Q, R);