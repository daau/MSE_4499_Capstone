%% Velocity & Acceleration Calculation

cph = 3600; % Containers per hour
disp_time = 5;  % Dispensing time (s)
row_dist = 0.1; % Distance between rows (m)
a_time = 0.025;  % Acceleration time (s)


rph = cph/10;   % Rows per hour
spr = 60^2/rph; % Second per row
v = row_dist / (spr-disp_time) % Velocity (m/s)
a = v/a_time % Acceleration

%% RPM and Torque Calculation



m = 7; % Total Load (kg)
g = 9.81;
theta = 0; % Incline angle (rad)
d = 0.06; % Wheel diameter (mm)
N = 2; % Number of Wheels
e = .65; % System efficiency (65% recommended average) 

rpm = v * 60 / (3.1415 * d) % rpm 

% T = Ffriction (f) * d/2
% Summing X axis Forces: Fx = m * a = f - m * g * sin(theta)
% M * a = 2T/d - mgx
T = 1/e * ((a+g*sin(theta))*m*d/2)/N;   % Torque (Nm)
T = T * 141.612 % Torque (ozf-in)


%% Pump Calculations

vpc = 0.005;    % Volume per container (L)
vpr = vpc * 10; % Volume per row (L)
disp_time = 1;  % Dispensing Time (from Velocity Calculation)

Q = vpr/disp_time; % Flow rate (L/s)
Q = Q * 1000 % Flow rate (ml/s)


%% Bernoulli's Equation for Manifold Calculation

dh = 0.025; % hydraulic diameter - diameter of pipe (m)
u=8.9*10^-4;    % dynamics viscosity [kg/(m.s)]
p1=104000;  % Inlet Pressure. built in 241317Pa 
p2=101325;  % Atmospheric Pressure [Pa]
pp=1000;    % Density of water [kg/m^3]
Q = 5e-6;   % Flow Rate[m^3/s]
A=pi*dh;    % Flow Area[m^2]
v1 = Q/A;   % Flow velocity[m/s]
Re=pp*v1*dh/u;  % Reynolds number
f=64/(Re);  % Flow coefficient 
w = 0.6;    % Width between arrays (m)
L=0.1;  % Length of pipe (m)



for i = 1:5 
    d(i) = 1000 * sqrt((4*Q)/(pi*sqrt((p1-p2)/pp+(v1^2/2)*(1-(f*(0.30 + L*i)/dh)))));
end

d   % Outlet diameters (mm)















