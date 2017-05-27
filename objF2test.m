% MAE 538 Design of Smart Material Systems - Course Project
% Active vibration control of a beam using smart materials
% Vinod Kumar Singla 4/22/2017
% Note: X = [x11 x12 x21 x22 x31 x32 . . . xm1 xm2]
% # of Piezoelectric patches = m, # of modes considered = 4

X = [.0264,0.1264,0.302,0.402,0.1659];

%% Beam properties
rho_b = 1190; % density, kg/^3
E_b = 3.1028e9; % young's modulus, Pa
v_b = 0.3; % poisson's ratio, dimensionless
t_b = 1.6e-3; % thickness, m
L_b = 0.5; % length, m
b = 0.01; % width, m
J_b = b*t_b^3/12; % Area Moment of Inertia
A_b = b*t_b; % cross sectional area of the beam
zeta = diag([0.01,0.01,0.01,0.01]); % damping ratio, dimensionless

%% Piezoelectric patch properties
rho = 1800; % density, kg/^3
E = 2e9; % young's modulus, Pa
d31 = 2.3e-11; %piezoelectric constant, m/V
h31 = 4.32e8; %piezoelectric constant, V/m
v = 0.3; % poisson's ratio, dimensionless
t = 4e-5; % thickness, m

%% Derived constants
Ka = b*((t_b+t)/2)*d31*E;
Ks1 = - t*h31*((t_b+t)/2)/(X(2)-X(1));
Ks2 = - t*h31*((t_b+t)/2)/(X(4)-X(3));

%% Natural frequencies for first 4 modes
wj = (pi/L_b)^2*sqrt(E_b*J_b/(rho_b*A_b));
W = wj.*(diag([1,2^2,3^2,4^2]));

%% Mode shape derivative differences for all elements
% Element 1
U1diff21 = (sqrt(2/(rho_b*L_b*A_b)))*(1*pi/L_b)*(cos((1*pi*X(2))/L_b)-cos((1*pi*X(1))/L_b));
U2diff21 = (sqrt(2/(rho_b*L_b*A_b)))*(2*pi/L_b)*(cos((2*pi*X(2))/L_b)-cos((2*pi*X(1))/L_b));
U3diff21 = (sqrt(2/(rho_b*L_b*A_b)))*(3*pi/L_b)*(cos((3*pi*X(2))/L_b)-cos((3*pi*X(1))/L_b));
U4diff21 = (sqrt(2/(rho_b*L_b*A_b)))*(4*pi/L_b)*(cos((4*pi*X(2))/L_b)-cos((4*pi*X(1))/L_b));

% Element 2
U1diff43 = (sqrt(2/(rho_b*L_b*A_b)))*(1*pi/L_b)*(cos((1*pi*X(4))/L_b)-cos((1*pi*X(3))/L_b));
U2diff43 = (sqrt(2/(rho_b*L_b*A_b)))*(2*pi/L_b)*(cos((2*pi*X(4))/L_b)-cos((2*pi*X(3))/L_b));
U3diff43 = (sqrt(2/(rho_b*L_b*A_b)))*(3*pi/L_b)*(cos((3*pi*X(4))/L_b)-cos((3*pi*X(3))/L_b));
U4diff43 = (sqrt(2/(rho_b*L_b*A_b)))*(4*pi/L_b)*(cos((4*pi*X(4))/L_b)-cos((4*pi*X(3))/L_b));

%% Open loop matrices
B_ = Ka.*[U1diff21,U1diff43;...
    U2diff21,U2diff43;...
    U3diff21,U3diff43;...
    U4diff21,U4diff43];

C_ = [Ks1.*[U1diff21,U2diff21,U3diff21,U4diff21];...
    Ks2.*[U1diff43,U2diff43,U3diff43,U4diff43]];
    
A = vertcat([zeros(4),eye(4)],[-W*W,-2*zeta*W]);
B = [zeros(4,2);B_];
C = [C_,zeros(2,4)];
Q = vertcat([W*W,zeros(4)],[zeros(4),eye(4)]);

%% Close loop matrices
G = [.4,0.4;0,0.4];
Ac = double(vertcat([zeros(4),eye(4)],[-W*W,B_*G*C_-2.*zeta*W]));

%% Initial conditions
n0 = [0,0,0,0]; n0_d = [0.2,0.4,0.6,0.8];n = [n0,n0_d];
P = lyap(double(Ac)',-double(Q)); % The inbuilt function is used to boost speed
f = max(-double(n*P*n'),0);

%% IC response & plots
sys =ss(Ac,[],eye(8),[])
sys_ol = ss(A,[],eye(8),[])
[y,t,x] = initial(sys,n,1.5);
[y_ol,t_ol,x_ol] = initial(sys_ol,n,1);
% plot(t,y(:,4));hold on;
% plot(t_ol,y_ol(:,1));
% figure;
% plot(t,y(:,2));hold on;
% plot(t_ol,y_ol(:,2));figure;
% plot(t,y(:,3));hold on;
% plot(t_ol,y_ol(:,3));figure;
% plot(t,y(:,4));hold on;
% plot(t_ol,y_ol(:,4));hold off;