function [ui, xi, yi] = proposed(eta_ini,nv_ini,yref,nu_c,Ts,Tf)
% This function implements the velocity form LPV-MPC1 cconsidering ocean
% currents and measurement noise.
% Cite: Jimoh, I.A., Yue, H. and Küçükdemiral, I.B., 2023. Autonomous underwater
% vehicle positioning control-a velocity form LPV-MPC approach.
% IFAC-PapersOnLine, 56(2), pp.4388-4393.

% Simulation and MPC parameters
NoS = round(Tf/Ts);
nx = 6;                     % number of velocity variables
nu = 6;                     % number of position variables
ny = 6;                     % number of position (output) variables
Q = blkdiag(1, 1, 1, 1, 1, 1)*1000;
R = blkdiag(1, 1, 1, 1,1, 1)*2e-3;
N = 12;
Nu = 2;

% Memory allocations
x        = zeros(nx,NoS);   % System states defined as the AUV velocities
x(:,1)   = nv_ini;
y        = zeros(ny,NoS);   % System output defined as the AUV position
y(:,1)   = eta_ini;
tau      = zeros(nu,NoS);   % Input foeces and mements
Dtau     = zeros(nu,NoS);
tau_     = zeros(nu, 1);    % Control input at time step k-1
xtilde = zeros(nx+ny,NoS);  % Augemented state
Dx        = zeros(nx,NoS);  % State increment Dx = x(k) - x(k-1)
Dy        = zeros(ny,NoS);  % Output increment Dy = y(k) - y(k-1)
tau_wave = 0;               % Defined as zero since ocean wave disturbances
                            % are not considered

% Main simulation loop
for k = 1:NoS-1
    yref_p = kron(ones(N,1), yref(:,k));    % Trajectory prediction

    % Obtain update AUV matrices based on current state
    [~,M, Jk, Ck, Dk,~] = Naminow_AUV([y(:,k);x(:,k)],tau_,0*nu_c(:,k),tau_wave);
    % Note: current is multiplied by zero because it is unknown to the controller
    Ak = eye(6)-inv(M)*(Ck+Dk)*Ts;
    Bk =  inv(M)*Ts;
    Hk = Jk*Ts;
    Atilde = [Ak zeros(nx,ny); Hk eye(ny)];
    Btilde = [Bk; zeros(ny, nu)];
    Gtilde = [zeros(nx) eye(ny)];
    Bdtilde = [zeros(nx); eye(nx)];
    [X,~,~,~] = dare(Ak,Bk, Q,R,[],[]); % Computes terminal weight based...
    % on discrete-time algebraic Riccati Eqn
    S  = 1*X; % This may also be taken as Q

    % Contruction of prediction model matrices
    A_p = predA(Gtilde,Atilde,N);
    B_p = predB(Gtilde,Atilde, Btilde, N, Nu);
    Bd_p = predB(Gtilde,Atilde, Bdtilde, N, N);
    Q_p = predQ(Q,S,N);
    R_p =  kron(eye(Nu), R);
    Dy_p = kron(ones(N,1), Dy(:,k));
    H = B_p'*Q_p*B_p + R_p;
    f = B_p'*Q_p*(A_p*xtilde(:,k) + Bd_p*Dy_p - yref_p);

    % Output constraint
    ymax =  [inf; inf; inf; inf; 2*pi/5; inf]*1;
    ymin = -[inf; inf; inf; inf; 2*pi/5; inf]*1;
    Ymax = kron(ones(N,1),ymax);
    Ymin = kron(ones(N,1),ymin);
    Aqp = [B_p; -B_p];
    bqp = [Ymax-(A_p*xtilde(:,k)-Bd_p*Dy_p); -Ymin+(A_p*xtilde(:,k)-Bd_p*Dy_p)];

    % Solve quadratic program
    options = optimset('Display', 'off');
    H=(H+H')/2; % To impose symmetry in the Hessian matrix
    V = quadprog(H, f, Aqp,bqp,[],[],[],[],[],options);
    Dtau(:,k) = V(1:nu);
    tau(:,k) = Dtau(:,k) + tau_;

    % Apply input forces to the AUV dynamics and add white noise
    x_next  = awgn(solver_RK([y(:,k);x(:,k)],Ts,tau(:,k),nu_c(:,k),tau_wave),80);

    % Extract the position and velocity vectors
    x(:,k+1) = x_next(7:12) ; %Ak*(x(:,k)-nu_c) + Bk*tau(:,k) + dnk;
    y(:,k+1) = x_next(1:6);
    Dy(:,k+1) = y(:,k+1) - y(:,k);
    Dx(:,k+1) = x(:,k+1) - x(:,k);
    xtilde(:,k+1) = [Dx(:,k+1); y(:,k+1)];

    % Updates the previous control input, position and velocities
    tau_ = tau(:,k);

    %sprintf('Iteration number = %d of %d',k, NoS)
end

xi = [y;x];

yi = y;

ui = tau;

end




