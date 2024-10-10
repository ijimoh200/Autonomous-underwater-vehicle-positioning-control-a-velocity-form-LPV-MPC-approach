function [xidot,M, Jk, Ck, Dk, gk] = Naminow_AUV(xi,ui,nu_c,dw)
% [xidot,M, Jk, Ck, Dk, gk] = Naminow_AUV(xi,ui,nu_c,dw) returns the 
% time derivatives of the state vector 
% x=[x y z phi theta psi u v w p q r] of the Naminow-D AUV 
% (Uchihori et al., 2021) and the matrices (M, Jk, Ck, Dk) and vector (gk)
% used to describe the vehicle dynamics.

% Earth-fixed coordinates
% x = Position in x-direction           [m]
% y = Position in y-direction           [m]
% z = Position in z-direction           [m]
% phi = Roll angle                      [rad]
% theta = Pitch angle                   [rad]
% psi = Yaw angle                       [rad]

% Body-referenced Coordinates
% u = Surge velocity                    [m/sec]
% v = Sway velocity                     [m/sec]
% w = Heave velocity                    [m/sec]
% p = Roll rate                         [rad/sec]
% q = Pitch rate                        [rad/sec]
% r = Yaw rate                          [rad/sec]

% Input vector
% ui = [tau_u, tau_v, tau_w, tau_p, tau_q, tau_r]

% Check for dimensional accuracy of inputs state and control to the function
if (length(xi) ~= 12), error('State vector must have a dimension of 12!');
end

 if (length(ui) ~= 6), error('Input vector must have a dimension of 6!');
 end

% State and input variables assignment
%State variables:
x = xi(1);              y = xi(2);                  z = xi(3);
phi = xi(4);            theta = xi(5);              psi = xi(6);
u = xi(7);              v =xi(8);                   w = xi(9);   
p = xi(10);             q = xi(11) ;                r = xi(12);

% Compute velocity vectors
v1 = [u v w]'; 
v2 = [p q r]';  
nu = [v1; v2];          % AUV Velocity 

% AUV relative velocity w.r.t. ocean currents
nu_r = nu - nu_c;       % AUV relative velocity
v1_r = nu_r(1:3);       v2_r = nu_r(4:6);
% Input variables                           % Units
tau_u     = ui(1);                    % N
tau_v     = ui(2);                    % N
tau_w     = ui(3);                    % N
tau_p     = ui(4);                    % Nm
tau_q     = ui(5);                    % Nm
tau_r     = ui(6);                    % Nm

% Vehicle's parameter
W       = 1940;                     % N
B       = 1999;                     % N
L       = 3.00;                     % m
Ixx     = 5.8;                      % kg.m^2
Iyy     = 114;                      % kg.m^2
Izz     = 114;                      % kg.m^2
xg      = -1.378;                   % m
yg      = 0;                        % m
zg      = 0.00;                     % m
xb      = xg;                       % m
yb      = yg;                       % m
zb      = zg;                       % m
b       = 0.324;                    % m
rho     = 1024;                     % kg/m^3
g       = 9.8;                      % m/s
m       = W/g;                      % kg

% Hydrodynamic damping coefficients
Xuu     = -12.7;                    % kg/m
Yvv     = -574;                     % kg/m
Zww     = -574;                     % kg/m
Yrr     = 12.3;                     % kg.m/rad^2
Zqq     = 12.3;                     % kg.m/rad^2
Mww     = 27.4;                     % kg
Mqq     = -4127;                    % kg.m^2/rad^2
Nvv     = -27.4;                    % kg
Nrr     = -4127;                    % kg.m^2/rad^2
Kpp     = -0.63;                    % kg.m^2/rad^2

% Added mass coefficients
Xud     = -6;                       % kg
Yvd     = -230;                     % kg
Zwd     = -230;                     % kg
Kpd     = -1.31;                    % kg.m^2/rad            
Mqd     = -161;                     % kg.m^2/rad
Nrd     = -161;                     % kg.m^2/rad
Yrd     = 28.3;                     % kg.m^2/rad
Zqd     = -28.3;                    % kg.m^2/rad
Mwd     = -28.3;                    % kg.m
Nvd     = 28.3;                     % kg.m

% Kinematic model

% The vehicle's kinematics given by eta = [x y z phi theta psi]'; is
% derived using the transformation matrix J through the relationship: 
% eta = J(eta)V.

% Coordinate transform term definition
cos1 = cos(phi); cos2 = cos(theta); cos3 = cos(psi); 
sin1 = sin(phi); sin2 = sin(theta); sin3 = sin(psi); 
tan2 = tan(theta);

% Rotation matrix
J1 = [cos2*cos3 -sin3*cos1+cos3*sin2*sin1 cos3*cos1*sin2+sin3*sin1;
sin3*cos2 cos3*cos1+sin1*sin2*sin3 sin2*sin3*cos1-cos3*sin1
-sin2 cos2*sin1 cos2*cos1];

J2 = [1 sin1*tan2 cos1*tan2;
0 cos1 -sin1;
0 (sin1/cos2) (cos1/cos2)];

Jk = [J1 zeros(size(J1,1),size(J2,2));
    zeros(size(J2,1),size(J1,2)) J2 ];

% Dynamic model

% Added mass matrix
I0 = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];
rgb = [xg yg zg];

M11 = m*eye(3);
M12 = -m*SS(rgb);
M21 = m*SS(rgb);
M22 = I0;

MRB = [M11 M12; M21 M22];

A11 = [Xud 0 0;
    0 Yvd 0;
    0 0 Zwd];

A12 = [0 0 0;
    0 0 Yrd;
    0 Zqd 0];

A21 = [0 0 0;
    0 0 Mwd;
    0 Nvd 0];

A22 = [Kpd 0 0;
    0 Mqd 0;
    0 0 Nrd];

MA  = [A11 A12; A21 A22];

M = MRB + MA;

% Coriolis and centripetal matrix
CRB = [zeros(3) -SS(M11*v1 + M12*v2);
    -SS(M11*v1 + M12*v2) -SS(M21*v1 + M22*v2)];

CAM = [zeros(3) -SS(A11*v1_r + A12*v2_r);
    -SS(A11*v1_r + A12*v2_r) -SS(A21*v1_r + A22*v2_r)];

Ck = CRB + CAM;

% Hydrodynamic Damping
Dk = -[Xuu*abs(nu_r(1)) 0 0 0 0 0;
    0 Yvv*abs(nu_r(2)) 0 0 0 Yrr*abs(nu_r(6));
    0 0 Zww*abs(nu_r(3)) 0 Zqq*abs(nu_r(5)) 0;
    0 0 0 Kpp*abs(nu_r(5)) 0 0;
    0 0 Mww*abs(nu_r(3)) 0 Mqq*abs(nu_r(5)) 0;
    0 Nvv*abs(nu_r(2)) 0 0 0 Nrr*abs(nu_r(6))];

% Buoyancy forces and moments
gk = [(W-B)*sin2;
    -(W-B)*cos2*sin1;
    -(W-B)*cos2*sin1;
    -(yg*W-yb*B)*cos2*sin1 + (zg*W-zb*B)*cos2*cos1;
    (zg*W-zb*B)*sin2 + (xg*W-xb*B)*cos2*cos1;
    -(xg*W-xb*B)*cos2*sin1-(yg*W-yb*B)*sin2];

% Input forces and moments
H = [tau_u;tau_v;tau_w;tau_p; tau_q;tau_r];
% Compute state vector
g_xi = [Jk*nu_r; -inv(M)*(Ck*nu_r+Dk*nu_r+gk)];
h_xi_u = [zeros(6,1); inv(M)*H];
xidot = g_xi + h_xi_u + dw; 

end

