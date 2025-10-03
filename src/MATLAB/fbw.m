function [ X_dot ] = fbw(X, U)
% Fly-by-Wire dynamics for aircraft
% X : State vector [u v w p q r phi theta psi]
% U : Control vector [Aileron, Elevator, Rudder, Throttle1, Throttle2]

% Extract states
x1 = X(1); x2 = X(2); x3 = X(3); % Body velocities
x4 = X(4); x5 = X(5); x6 = X(6); % Angular rates
x7 = X(7); x8 = X(8); x9 = X(9); % Euler angles

% Extract controls
u1 = U(1); u2 = U(2); u3 = U(3); u4 = U(4); u5 = U(5);

% Aircraft constants
m = 120000; cbar = 6.6; lt = 24.8;
S = 260; St = 64; Xcg = 0.23*cbar; Ycg = 0; Zcg = 0.1*cbar;
Xac = 0.12*cbar; Yac = 0; Zac = 0;

% Engine positions
Xapt1 = 0; Yapt1 = -7.94; Zapt1 = -1.9;
Xapt2 = 0; Yapt2 = 7.94; Zapt2 = -1.9;

% Environment constants
rho = 1.225; g = 9.8; d = 0.25;

% Aerodynamic coefficients
alpha_l0 = -11.5*pi/180; n_slope = 5.5;
a3 = -768.5; a2 = 609.2; a1 = -155.2; a0 = 15.212;
alpha_switch = 14.5*pi/180;

% Control limits
u_min = [-25*pi/180; -25*pi/180; -30*pi/180; 0.5*pi/180; 0.5*pi/180];
u_max = [25*pi/180; 10*pi/180; 30*pi/180; 10*pi/180; 10*pi/180];

% Compute airspeed, angle of attack, sideslip, dynamic pressure
Va = sqrt(x1^2 + x2^2 + x3^2);
alpha = atan2(x3, x1);
l = x2 / Va; beta = asin(l);
Q = 0.5 * rho * Va^2;

% Angular velocity & translational velocity vectors
Wbe_b = [x4; x5; x6];
V_b = [x1; x2; x3];

% Aerodynamic forces
if alpha <= alpha_switch
    Cl_wb = n_slope * (alpha - alpha_l0);
else
    Cl_wb = a3*(alpha^3) + a2*(alpha^2) + a1*alpha + a0;
end
epsilon = d * (alpha - alpha_l0);
alpha_t = alpha - epsilon + u2 + (1.3*x5*(lt/Va));
Cl_t = 3.1*(St/S)*alpha_t;
Cl = Cl_wb + Cl_t;
Cd = 0.13 + 0.07*((5.5*alpha + 0.654)^2);
Cy = -1.6*beta + 0.24*u3;

% Transform to body frame
Fa_s = [-Cd*Q*S; Cy*Q*S; -Cl*Q*S];
C_bs = [cos(alpha),0,-sin(alpha);0,1,0;sin(alpha),0,cos(alpha)];
Fa_b = C_bs*Fa_s;

% Moments calculations
n1 = -1.4*beta; n2 = -0.59 - 3.1*(St*lt/(S*cbar))*(alpha-epsilon); n3 = (1-alpha*(180/(15*pi)))*beta;
n_vec = [n1; n2; n3];
dCm_dx = (cbar/Va)*[-11,0,5;0,-4.03*((St*lt^2)/(S*cbar^2)),0;1.7,0,-11.5];
dCm_du = [-0.6,0,0.22;0,-3.1*((St*lt)/(S*cbar)),0;0,0,-0.63];
Cmac_b = n_vec + dCm_dx*Wbe_b + dCm_du*[u1; u2; u3];
Mac_b = Cmac_b*Q*S*cbar;

% Propulsion forces
F1 = u4*m*g; F2 = u5*m*g;
Fe_b = [F1+F2; 0; 0];
mu1 = [Xcg-Xapt1; Yapt1-Ycg; Zcg-Zapt1]; mu2 = [Xcg-Xapt2; Yapt2-Ycg; Zcg-Zapt2];
Me_cg = cross(mu1,[F1;0;0]) + cross(mu2,[F2;0;0]);

% Gravity forces
g_b = [-g*sin(x8); g*cos(x8)*sin(x7); g*cos(x8)*cos(x7)];
Fg_b = m*g_b;

% Inertia
Ib = m*[40.07,0,-2.0923;0,64,0;-2.0923,0,99.92];
inv_Ib = (1/m)*[0.0249836,0,0.000523151;0,0.015625,0;0.000523151,0,0.010019];

% Total forces and moments
F_b = Fg_b + Fe_b + Fa_b;
xdot1_3 = (1/m)*F_b - cross(Wbe_b,V_b);
xdot4_6 = inv_Ib*(Me_cg + Mac_b - cross(Wbe_b,Ib*Wbe_b));

h_dot = [1, sin(x7)*tan(x8), cos(x7)*tan(x8);
         0, cos(x7), -sin(x8);
         0, sin(x7)/cos(x8), cos(x7)/cos(x8)];
xdot7_9 = h_dot*Wbe_b;

% Assign output
if all(U==0)
    X_dot = zeros(9,1);
else
    X_dot = [xdot1_3; xdot4_6; xdot7_9];
end
end
