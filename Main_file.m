%% POSITIONING CONTROL DURING DOCKING BASED ON LPV-MPC1

% Initial values of the state vector
x   = 0;            y       = 0;          z     = 0;
phi = 0;            theta   = 0;          psi   = 0;
u   = 0;            v       = 0;          w     = 0;
p   = 0;            q       = 0;          r     = 0;
eta_ini = [x y z phi theta psi]';
nv_ini  = [u v w p q r]'; % Note nv is used to denote \nu

% Sampling and simulation parameters
Ts      = 0.1;
Tf      = 30;
NoS     = round(Tf/Ts);

%% Generate reference trajectory and noise signals
noise   = zeros(12,NoS);    % Measurement noise
yref    = zeros(6,NoS);     % Reference signal
nu_c = zeros(6,NoS);        % Unknown time-varying current disturbance
for k = 1:NoS
    % Reference signal:
    if k <= round(NoS/4)
        yref(:,k) = [0; 0; 0; 0; 0; 0];
    else
        yref(:,k) = [0.5; 0.5; 0.0; 0; 0; -0.];
    end
    % Generate time-varying ocean current
    if k <= round(NoS/2)
        nu_c(:,k)  = [0; 0; 0; 0; 0; 0];
    else
        nu_c(:,k)  = [0.2*1; 0.2*1; 0.1; 0; 0; 0] ...
            +[0.1*cos(3/20*pi*k*Ts)*cos(0.002*k*Ts); 0.1*sin(3/20*pi*k*Ts); 0.1*sin(0.002*k*Ts)*cos(3/20*pi*k*Ts); 0; 0; 0];
    end
end

%% Simulate Controller
%Proposed LPV-MPC1 in the conference paper:
%Jimoh, I.A., Yue, H. and Küçükdemiral, I.B., 2023. Autonomous underwater
% vehicle positioning control-a velocity form LPV-MPC approach.
% IFAC-PapersOnLine, 56(2), pp.4388-4393.
[tau1, x1, y1] = proposed(eta_ini,nv_ini,yref,nu_c,Ts,Tf);

%% Plot Results
t = 0:Ts:Tf-Ts;
figure
subplot(621)
stairs(t, yref(1, :),'-.k', 'linewidth', 1.2)
hold on
stairs(t, y1(1, 1:size(t,2)),'-r', 'linewidth', 1.2)
ylabel('$x$ [m]', 'interpreter','latex')
ylim([-0.1 0.6])
grid on

subplot(623)
stairs(t, yref(2, 1:size(t,2)),'-.k', 'linewidth', 1.2)
hold on
stairs(t, y1(2, 1:size(t,2)),'r', 'linewidth', 1.2)
ylabel('$y$ [m]', 'interpreter','latex')
ylim([-0.1 0.6])
grid on

subplot(625)
stairs(t, yref(3, 1:size(t,2)),'-.k', 'linewidth', 1.2)
hold on
stairs(t, y1(3, 1:size(t,2)),'r', 'linewidth', 1.2)
ylim([-0.05 0.03])
ylabel('$z$ [m]', 'interpreter','latex')
grid on

subplot(627)
stairs(t, yref(4, 1:size(t,2)),'-.k', 'linewidth', 1.2)
hold on
stairs(t, y1(4, 1:size(t,2)),'r', 'linewidth', 1.2)
ylabel('$\phi$ [rad]', 'interpreter','latex')
ylim([-0.04 0.02])
grid on

subplot(629)
stairs(t, yref(5, 1:size(t,2)),'-.k', 'linewidth', 1.2)
hold on
stairs(t, y1(5, 1:size(t,2)),'r', 'linewidth', 1.2)
ylabel('$\theta$ [rad]', 'interpreter','latex')
grid on
ylim([-0.05 0.05])

subplot(6,2,11)
stairs(t, yref(6, 1:size(t,2)),'-.k', 'linewidth', 1.2)
hold on
stairs(t, y1(6, 1:size(t,2)),'r', 'linewidth', 1.2)
ylabel('$\psi$ [rad]', 'interpreter','latex')
legend('Target','LPCMPC1')
xlabel ('Time [s]', 'interpreter','latex')
ylim([-0.15 0.03])
grid on

subplot(622)
stairs(t, tau1(1, 1:size(t,2)),'r', 'linewidth', 1.2)
ylabel('X [N]', 'interpreter','latex')
ylim([-800 800])
grid on

subplot(624)
stairs(t, tau1(2, 1:size(t,2)),'r', 'linewidth', 1.2)
ylabel('Y [N]', 'interpreter','latex')
ylim([-800 800])
grid on

subplot(626)
stairs(t, tau1(3, 1:size(t,2)),'r', 'linewidth', 1.2)
ylabel('Z [N]', 'interpreter','latex')
ylim([-200 200])
grid on

subplot(628)
stairs(t, tau1(4, 1:size(t,2)),'r', 'linewidth', 1.2)
ylabel('K [Nm]', 'interpreter','latex')
ylim([-200 200])
grid on


subplot(6,2,10)
stairs(t, tau1(5, 1:size(t,2)),'r', 'linewidth', 1.2)
ylabel('M [Nm]', 'interpreter','latex')
ylim([-800 800])
grid on

subplot(6,2,12)
stairs(t, tau1(6, 1:size(t,2)),'r', 'linewidth', 1.2)
xlabel ('Time [s]', 'interpreter','latex')
ylabel('N [Nm]', 'interpreter','latex')
ylim([-800 800])
grid on


