import numpy as np
import cvxopt
from cvxopt import matrix, solvers
import matplotlib.pyplot as plt

# Initial values of the state vector
x = 0.2
y = 0
z = -0.8
phi = 0
theta = 0
psi = 0
u = 0
v = 0
w = 0
p = 0
q = 0
r = 0

eta_ini = np.array([x, y, z, phi, theta, psi])
nv_ini = np.array([u, v, w, p, q, r])

# Sampling and simulation parameters
Ts = 0.1
Tf = 30
NoS = round(Tf / Ts)
t = np.arange(0, Tf, Ts)

# Generate reference trajectory and noise signals
noise = np.zeros((12, NoS))
yref = np.zeros((6, NoS))

nu_c = np.zeros((6, NoS))

for k in range(NoS):

    # yref[:,k] = [10*np.sin(0.03*k*Ts), 10*np.cos(0.03*k*Ts), -0.05*k*Ts, 0, 0, 0]

    if k <= round(NoS / 4):
        yref[:, k] = np.array([0.2, 0, -1, 0, 0, 0])
    else:
        yref[:, k] = np.array([0.5, 0.5, -1, 0, 0, -0.1])

    linearposition_noise = 0.005 * np.random.rand(3)
    angularposition_noise = (5.42 * 10 ** (-5)) * np.random.rand(3)
    linearvelocity_noise = 0.1 * np.random.rand(3)
    angularvelocity_noise = (5.42 * 10 ** (-3)) * np.random.rand(3)
    
    noise[:, k] = np.concatenate((linearposition_noise, angularposition_noise, 
                                  linearvelocity_noise, angularvelocity_noise))

    nu_c[:, k] = np.array([0.2, 0.2, 0.1, 0, 0, 0]) + np.array([0, 0, 0, 0, 0, 0])


# Helper functions for matrix manipulation

def awgn(signal, snr_db):
    snr = 10 ** (snr_db / 10)
    power_signal = np.mean(np.abs(signal) ** 2)
    power_noise = power_signal / snr
    noise = np.sqrt(power_noise) * np.random.randn(*signal.shape)
    return signal + noise
import numpy as np

def block_diag(values, scalar):
    """   
    Parameters:
    values (list of float): The values to place on the diagonal.
    scalar (float): The scalar to multiply the block diagonal matrix by.
    """
    # Check that values is a list
    if not isinstance(values, list):
        raise ValueError("Values must be provided as a list.")
    
    # Create the block diagonal matrix
    blocks = [np.eye(1) * value for value in values]
    blk_diag_matrix = np.block([[block if i == j else np.zeros_like(block) 
                                 for j, block in enumerate(blocks)] 
                                for i in range(len(blocks))])
    
    # Multiply the matrix by the scalar
    result = blk_diag_matrix * scalar
    
    return result




# Define predA, predB, predQ, solver_RK, Naminow_AUV, and awgn as helper functions

def predA(G, Aeta, N):
    temp = []
    for i in range(1, N+1):
        temp.append(G @ np.linalg.matrix_power(Aeta, i))
    A_p = np.vstack(temp)
    return A_p

def predA1(Aeta, N):
    temp = []
    for i in range(1, N + 1):
        temp.append(np.linalg.matrix_power(Aeta, i))
    
    A_p = np.vstack(temp)
    return A_p


def predB(C, A, B, N, Nu):
    p, _ = C.shape
    _, m = B.shape
    barC_ = np.zeros((N*p, Nu*m))
    
    # First column of barC_
    first_col = []
    for i in range(N):
        first_col.append(C @ np.linalg.matrix_power(A, i) @ B)
    first_col = np.vstack(first_col)
    barC_[:, :m] = first_col
    
    # Shift columns
    for col in range((Nu-1) * m):
        for row in range((N-1) * p):
            barC_[row + p, col + m] = barC_[row, col]
    
    B_p = barC_
    return B_p

def predB1(A, B, N, Nu):
    n, m = B.shape
    barC_ = np.zeros((N * n, Nu * m))
    
    # Construct the first column
    firstCol = []
    for i in range(N):
        firstCol.append(np.linalg.matrix_power(A, i) @ B)  # A^(i-1)*B
        
    firstCol = np.vstack(firstCol)  # Stack vertically
    barC_[:, :m] = firstCol
    
    # Fill the rest of the matrix based on the specific pattern
    for col in range(1, Nu):
        for row in range(n, N * n):
            barC_[row, col * m:(col + 1) * m] = barC_[row - n, (col - 1) * m:col * m]
    
    return barC_



def predQ(Q_p, S, N):
    # Create identity matrix of size (N-1)
    temp1 = np.eye(N-1)
    
    # Kronecker product of temp1 and Q_p
    temp2 = np.kron(temp1, Q_p)
    
    # Block diagonal matrix with temp2 and S
    Q_p = np.block([[temp2, np.zeros((temp2.shape[0], S.shape[1]))],
                    [np.zeros((S.shape[0], temp2.shape[1])), S]])
    
    return Q_p


def predW(C, N, Nu):
    p, n = C.shape
    barC_ = np.zeros((N * p, Nu * n))
    
    # Construct the first column
    firstCol = []
    for i in range(N):
        firstCol.append(C)
    
    firstCol = np.vstack(firstCol)  # Stack vertically
    barC_[:, :n] = firstCol
    
    # Fill the rest of the matrix based on the specific pattern
    for col in range(n, (Nu * n)):
        for row in range(p, (N * p)):
            barC_[row, col] = barC_[row - p, col - n]
    
    return barC_




def solver_RK(x, Ts, u, nu_c, dw):
    # RK = Runge Kutta    
    k1, _, _, _, _, _ = Naminow_AUV_sim(x, u, nu_c, dw)
    k2, _, _, _, _, _ = Naminow_AUV_sim(x + k1.flatten() * (Ts / 2), u, nu_c, dw)
    k3, _, _, _, _, _ = Naminow_AUV_sim(x + k2.flatten() * (Ts / 2), u, nu_c, dw)
    k4, _, _, _, _, _ = Naminow_AUV_sim(x + k3.flatten() * Ts, u, nu_c, dw)

    xi = x + (Ts / 6) * (k1.flatten() + 2 * k2.flatten() + 2 * k3.flatten() + k4.flatten())
    return xi

def SS(a):
    """
    This function returns the skew-symmetric matrix for a 3-dimensional vector.
    Based on Fossen's definition (2011).
    """
    if len(a) != 3:
        raise ValueError('Input vector must have a dimension of 3!')

    a1, a2, a3 = a

    S_n = np.array([[0, -a3, a2],
                    [a3, 0, -a1],
                    [-a2, a1, 0]])

    return S_n


def Naminow_AUV(xi, ui):
    # Check dimensionality of inputs
    if len(xi) != 12:
        raise ValueError('State vector must have a dimension of 12!')
    if len(ui) != 6:
        raise ValueError('Input vector must have a dimension of 6!')

    # State variables
    x, y, z, phi, theta, psi = xi[:6]
    u, v, w, p, q, r = xi[6:]
    v1 = np.array([u, v, w]).reshape(3, 1)
    v2 = np.array([p, q, r]).reshape(3, 1)
    nu = np.concatenate((v1, v2))

    # Input variables
    tau_u = ui[0]
    tau_v = ui[1]
    tau_w = ui[2]
    tau_p = ui[3]
    tau_q = ui[4]
    tau_r = ui[5]

    # Vehicle parameters
    W = 1940
    B = 1999
    L = 3.00
    Ixx = 5.8
    Iyy = 114
    Izz = 114
    xg = -1.378
    yg = 0
    zg = 0.00
    xb = xg
    yb = yg
    zb = zg
    b = 0.324
    rho = 1024
    g = 9.8
    m = W / g

    # Hydrodynamic damping coefficients
    Xuu = -12.7
    Yvv = -574
    Zww = -574
    Yrr = 12.3
    Zqq = 12.3
    Mww = 27.4
    Mqq = -4127
    Nvv = -27.4
    Nrr = -4127
    Kpp = -0.63

    # Added mass coefficients
    Xud = -6
    Yvd = -230
    Zwd = -230
    Kpd = -1.31
    Mqd = -161
    Nrd = -161
    Yrd = 28.3
    Zqd = -28.3
    Mwd = -28.3
    Nvd = 28.3

    # Rotation matrix
    cos1, cos2, cos3 = np.cos([phi, theta, psi])
    sin1, sin2, sin3 = np.sin([phi, theta, psi])
    tan2 = np.tan(theta)

    J1 = np.array([
        [cos2*cos3, -sin3*cos1 + cos3*sin2*sin1, cos3*cos1*sin2 + sin3*sin1],
        [sin3*cos2, cos3*cos1 + sin1*sin2*sin3, sin2*sin3*cos1 - cos3*sin1],
        [-sin2, cos2*sin1, cos2*cos1]
    ])

    J2 = np.array([
        [1, sin1*tan2, cos1*tan2],
        [0, cos1, -sin1],
        [0, sin1/cos2, cos1/cos2]
    ])

    Jk = np.block([[J1, np.zeros((3, 3))], [np.zeros((3, 3)), J2]])

    # Dynamic model
    I0 = np.diag([Ixx, Iyy, Izz])
    rgb = np.array([xg, yg, zg])

    M11 = m * np.eye(3)
    M12 = -m * SS(rgb)
    M21 = m * SS(rgb)
    M22 = I0

    MRB = np.block([[M11, M12], [M21, M22]])

    A11 = np.diag([Xud, Yvd, Zwd])
    A12 = np.array([[0, 0, 0], [0, 0, Yrd], [0, Zqd, 0]])
    A21 = np.array([[0, 0, 0], [0, 0, Mwd], [0, Nvd, 0]])
    A22 = np.diag([Kpd, Mqd, Nrd])

    MA = np.block([[A11, A12], [A21, A22]])
    M = MRB + MA
    
    # Coriolis and centripetal matrix
    CRB = np.block([[np.zeros((3, 3)), -SS((np.array(M11 @ v1 + M12 @ v2)).flatten())],
                    [-SS((np.array(M11 @ v1 + M12 @ v2)).flatten()), -SS((np.array(M21 @ v1 + M22 @ v2)).flatten())]])

    CAM = np.block([[np.zeros((3, 3)), -SS((np.array(A11 @ v1 + A12 @ v2)).flatten())],
                    [-SS((np.array(A11 @ v1 + A12 @ v2)).flatten()), -SS((np.array(A21 @ v1 + A22 @ v2)).flatten())]])

    Ck = CRB + CAM
    
    # Hydrodynamic damping
    Dk = -np.array([
        [Xuu * abs(u), 0, 0, 0, 0, 0],
        [0, Yvv * abs(v), 0, 0, 0, Yrr * abs(r)],
        [0, 0, Zww * abs(w), 0, Zqq * abs(q), 0],
        [0, 0, 0, Kpp * abs(p), 0, 0],
        [0, 0, Mww * abs(w), 0, Mqq * abs(q), 0],
        [0, Nvv * abs(v), 0, 0, 0, Nrr * abs(r)]
    ])

    # Buoyancy forces and moments
    gk = np.array([
        [(W - B) * sin2],
        [-(W - B) * cos2 * sin1],
        [-(W - B) * cos2 * sin1],
        [-(yg * W - yb * B) * cos2 * sin1 + (zg * W - zb * B) * cos2 * cos1],
        [(zg * W - zb * B) * sin2 + (xg * W - xb * B) * cos2 * cos1],
        [-(xg * W - xb * B) * cos2 * sin1 - (yg * W - yb * B) * sin2]
    ])
    
    # Input forces and moments
    H = np.array([tau_u, tau_v, tau_w, tau_p, tau_q, tau_r])
    # Compute state vector derivative
    term1 = Jk @ nu 
    term2 = -np.linalg.inv(M) @ (Ck @ nu + Dk @ nu + gk)
    g_xi = np.vstack([term1, term2])
        
    h_xi_u = np.vstack([np.zeros((6, 1)), np.linalg.inv(M) @ H])
    xidot = g_xi + h_xi_u

    return xidot, M, Jk, Ck, Dk, gk


#AUV model for dynamic evolution considering disturbance

def Naminow_AUV_sim(xi, ui, nu_c, dw):
    # State variables assignment
    x = xi[0]; y = xi[1]; z = xi[2]
    phi = xi[3]; theta = xi[4]; psi = xi[5]
    u = xi[6]; v = xi[7]; w = xi[8]
    p = xi[9]; q = xi[10]; r = xi[11]
    
    #x, y, z, phi, theta, psi = xi[:6]
    #u, v, w, p, q, r = xi[6:]
    
    v1 = np.array([u, v, w]).reshape(3, 1)
    v2 = np.array([p, q, r]).reshape(3, 1)
    nu = np.concatenate((v1, v2))
    

    # Compute relative velocity
    nu_r = nu - nu_c.reshape(6,1)
    v1_r = nu_r[:3]
    v2_r = nu_r[3:]

    # Input variables
    tau_u, tau_v, tau_w, tau_p, tau_q, tau_r = ui

    # Vehicle's parameters
    W = 1940; B = 1999
    L = 3.00; Ixx = 5.8; Iyy = 114; Izz = 114
    xg = -1.378; yg = 0; zg = 0
    xb = xg; yb = yg; zb = zg
    rho = 1024; g = 9.8; m = W/g

    # Hydrodynamic damping coefficients
    Xuu = -12.7; Yvv = -574; Zww = -574
    Yrr = 12.3; Zqq = 12.3; Mww = 27.4
    Mqq = -4127; Nvv = -27.4; Nrr = -4127
    Kpp = -0.63

    # Added mass coefficients
    Xud = -6; Yvd = -230; Zwd = -230
    Kpd = -1.31; Mqd = -161; Nrd = -161
    Yrd = 28.3; Zqd = -28.3
    Mwd = -28.3; Nvd = 28.3

    # Kinematic model
    cos1, cos2, cos3 = np.cos(phi), np.cos(theta), np.cos(psi)
    sin1, sin2, sin3 = np.sin(phi), np.sin(theta), np.sin(psi)
    tan2 = np.tan(theta)

    # Rotation matrices
    J1 = np.array([
        [cos2*cos3, -sin3*cos1 + cos3*sin2*sin1, cos3*cos1*sin2 + sin3*sin1],
        [sin3*cos2, cos3*cos1 + sin1*sin2*sin3, sin2*sin3*cos1 - cos3*sin1],
        [-sin2, cos2*sin1, cos2*cos1]
    ])
    
    J2 = np.array([
        [1, sin1*tan2, cos1*tan2],
        [0, cos1, -sin1],
        [0, sin1/cos2, cos1/cos2]
    ])

    Jk = np.block([[J1, np.zeros((3, 3))], [np.zeros((3, 3)), J2]])
    eta = np.dot(Jk, nu_r)

     # Dynamic model
    I0 = np.diag([Ixx, Iyy, Izz])
    rgb = np.array([xg, yg, zg])

    M11 = m * np.eye(3)
    M12 = -m * SS(rgb)
    M21 = m * SS(rgb)
    M22 = I0

    MRB = np.block([[M11, M12], [M21, M22]])

    A11 = np.diag([Xud, Yvd, Zwd])
    A12 = np.array([[0, 0, 0], [0, 0, Yrd], [0, Zqd, 0]])
    A21 = np.array([[0, 0, 0], [0, 0, Mwd], [0, Nvd, 0]])
    A22 = np.diag([Kpd, Mqd, Nrd])

    MA = np.block([[A11, A12], [A21, A22]])
    M = MRB + MA

    # Coriolis and centripetal matrix
    CRB = np.block([[np.zeros((3, 3)), -SS((np.array(M11 @ v1 + M12 @ v2)).flatten())],
                    [-SS((np.array(M11 @ v1 + M12 @ v2)).flatten()), -SS((np.array(M21 @ v1 + M22 @ v2)).flatten())]
    ])
    
    CAM = np.block([
        [np.zeros((3, 3)), -SS((np.array(A11 @ v1_r + A12 @ v2_r)).flatten())],
        [-SS((np.array(A11 @ v1_r + A12 @ v2_r)).flatten()), -SS((np.array(A21 @ v1_r + A22 @ v2_r)).flatten())]
    ])

    Ck = CRB + CAM
    nu_rD = (np.array(nu_r)).flatten() # strictly used to implenent the damping matrix
    # Hydrodynamic damping
    Dk = -np.array([
        [Xuu * abs(nu_rD[0]), 0, 0, 0, 0, 0],
        [0, Yvv * abs(nu_rD[1]), 0, 0, 0, Yrr * abs(nu_rD[5])],
        [0, 0, Zww * abs(nu_rD[2]), 0, Zqq * abs(nu_rD[4]), 0],
        [0, 0, 0, Kpp * abs(nu_rD[4]), 0, 0],
        [0, 0, Mww * abs(nu_rD[2]), 0, Mqq * abs(nu_rD[4]), 0],
        [0, Nvv * abs(nu_rD[1]), 0, 0, 0, Nrr * abs(nu_rD[5])]
    ])
    
    # Buoyancy forces and moments
    gk = np.array([
        (W - B) * sin2,
        -(W - B) * cos2 * sin1,
        -(W - B) * cos2 * sin1,
        -(yg * W - yb * B) * cos2 * sin1 + (zg * W - zb * B) * cos2 * cos1,
        (zg * W - zb * B) * sin2 + (xg * W - xb * B) * cos2 * cos1,
        -(xg * W - xb * B) * cos2 * sin1 - (yg * W - yb * B) * sin2
    ]).reshape(6,1)
    
    H = (np.array([tau_u, tau_v, tau_w, tau_p, tau_q, tau_r])).reshape(6,1)
    # Compute state vector derivative
    term1 = Jk @ nu_r 
    term2 = -np.linalg.inv(M) @ (Ck @ nu_r + Dk @ nu_r + gk)
    g_xi = np.vstack([term1, term2]) 
            
    h_xi_u = np.vstack([np.zeros((6, 1)), np.linalg.inv(M) @ H])
    xidot = g_xi + h_xi_u

    return xidot, M, Jk, Ck, Dk, gk



# Simulate Controllers with functions below:

def proposed1(eta_ini, nv_ini, yref, nu_c, Ts, Tf, noise):
    NoS = round(Tf / Ts)
    nx = 6
    m = 6
    ny = 6
    value1 = [1, 1, 1, 1, 1, 1]
    scalar1 = 1000
    scalar2 = 0.002
    Q = block_diag(value1,scalar1) 
    R = block_diag(value1,scalar2) 
    S = Q
    N = 16
    Nu = 2

    # Memory locations
    x = np.zeros((nx, NoS))  # AUV velocities
    x[:, 0] = nv_ini
    y = np.zeros((ny, NoS))  # AUV position
    y[:, 0] = eta_ini
    tau = np.zeros((m, NoS))  # Input forces and moments
    Dtau = np.zeros((m, NoS))
    tau_ = np.zeros((m, 1))  # Control input at time step k-1
    xtilde = np.zeros((nx + ny, NoS))  # Augmented state
    Dx = np.zeros((nx, NoS))  # State increment Dx = x(k) - x(k-1)
    Dy = np.zeros((ny, NoS))  # Output increment Dy = y(k) - y(k-1)
    tau_wave = 0  # No ocean wave disturbances

    for k in range(NoS - 1):
        yref_p = np.kron(np.ones(N), yref[:, k])  # Trajectory prediction

        # Obtain AUV matrices based on current state
        _, M, Jk, Ck, Dk, _ = Naminow_AUV(np.concatenate((y[:, k], x[:, k])), tau_)
        Ak = np.eye(6) - np.linalg.inv(M) @ (Ck + Dk) * Ts
        Bk = np.linalg.inv(M) * Ts
        Hk = Jk * Ts
        Atilde = np.block([[Ak, np.zeros((nx, ny))], [Hk, np.eye(ny)]])
        Btilde = np.block([[Bk], [np.zeros((ny, m))]])
        Gtilde = np.block([np.zeros((nx, ny)), np.eye(ny)])
        Bdtilde = np.block([[np.zeros((nx, nx))], [np.eye(nx)]])

        # Construct prediction model matrices
        A_p = predA(Gtilde, Atilde, N)
        B_p = predB(Gtilde, Atilde, Btilde, N, Nu)
        Bd_p = predB(Gtilde, Atilde, Bdtilde, N, N)
        Q_p = predQ(Q, S, N)
        R_p = np.kron(np.eye(Nu), R)
        Dy_p = np.kron(np.ones(N), Dy[:, k])
        H = B_p.T @ Q_p @ B_p + R_p
        f = B_p.T @ Q_p @ (A_p @ xtilde[:, k] + Bd_p @ Dy_p - yref_p)
       
        # Output constraint
        ymax = np.array([1e12, 1e12, 1e12, 1e12, np.pi / 2, 1e12])
        ymin = -ymax
        Ymax = np.kron(np.ones(N), ymax)
        Ymin = np.kron(np.ones(N), ymin)
        Aqp = np.vstack([B_p, -B_p])
        bqp = np.hstack([Ymax - A_p @ xtilde[:, k] - Bd_p @ Dy_p, 
                         -Ymin + A_p @ xtilde[:, k] + Bd_p @ Dy_p]) # hstack because the concatinated vectors have the same size
                      
        # Solve the quadratic programming problem using cvxopt
         # Convert all inputs to cvxopt.matrix format
        H = matrix(H)
        f = matrix(f)
        Aqp = matrix(Aqp)
        bqp = matrix(bqp)  
        solution = solvers.qp(H, f, Aqp, bqp)
        V = solution['x']
        
        if V is not None:
            V = np.array(V)
            Dtau[:, k] = V[:m].flatten() 
        else:
            print("Optimization failed")

        tau[:, k] = Dtau[:, k] + tau_.flatten()
        
        
        #qq = tau*xtilde
        
        
        # Apply input forces to AUV dynamics
        x_next = solver_RK(np.concatenate((y[:, k], x[:, k])), Ts, tau[:, k], nu_c[:, k], tau_wave)
        x[:, k + 1] = x_next[6:12]
        y[:, k + 1] = x_next[:6]        
        Dy[:, k + 1] = y[:, k + 1] - y[:, k]
        Dx[:, k + 1] = x[:, k + 1] - x[:, k]
        xtilde[:, k + 1] = np.concatenate((Dx[:, k + 1], y[:, k + 1]))
        
        #qq = xtilde*tau
    
        # Update previous control input
        tau_ = tau[:, k].reshape(6,1)

    x1 = np.vstack([y, x])
    y1 = y
    tau1 = tau

    return tau1, x1, y1
   
# Call the functions with respective parameters
tau1, x1, y1 = proposed1(eta_ini, nv_ini, yref, nu_c, Ts, Tf, noise)




# Create figure
plt.figure(figsize=(8, 8))

# Subplot 621
plt.subplot(6, 2, 1)
plt.step(t, yref[0, :], linestyle='-.', color='k', linewidth=1.2, label='yref')
plt.step(t, y1[0, :len(t)], color='r', linewidth=1.2, label='y_1')
plt.ylabel('$x$ [m]')
#plt.ylim([-0.4, 0.6])
plt.xlim(0, 30)
plt.grid(True)

# Subplot 623
plt.subplot(6, 2, 3)
plt.step(t, yref[1, :], linestyle='-.', color='k', linewidth=1.2, label='yref')
plt.step(t, y1[1, :len(t)], color='r', linewidth=1.2, label='y1')
plt.ylabel('$y$ [m]')
#plt.ylim([-0.4, 0.6])
plt.xlim(0, 30)
plt.grid(True)

# Subplot 625
plt.subplot(6, 2, 5)
plt.step(t, yref[2, :], linestyle='-.', color='k', linewidth=1.2, label='yref')
plt.step(t, y1[2, :len(t)], color='r', linewidth=1.2, label='y1')
plt.ylabel('$z$ [m]')
#plt.ylim([-0.15, 0.02])
plt.xlim(0, 30)
plt.grid(True)

# Subplot 627
plt.subplot(6, 2, 7)
plt.step(t, yref[3, :], linestyle='-.', color='k', linewidth=1.2, label='yref')
plt.step(t, y1[3, :len(t)], color='r', linewidth=1.2, label='y1')
plt.ylabel('$\\phi$ [rad]')
#plt.ylim([-0.15, 0.1])
plt.xlim(0, 30)
plt.grid(True)

# Subplot 629
plt.subplot(6, 2, 9)
plt.step(t, yref[4, :], linestyle='-.', color='k', linewidth=1.2, label='yref')
plt.step(t, y1[4, :len(t)], color='r', linewidth=1.2, label='y1')
plt.ylabel('$\\theta$ [rad]')
#plt.ylim([-0.04, 0.04])
plt.xlim(0, 30)
plt.grid(True)

# Subplot 6,2,11
plt.subplot(6, 2, 11)
plt.step(t, yref[5, :], linestyle='-.', color='k', linewidth=1.2, label='yref')
plt.step(t, y1[5, :len(t)], color='r', linewidth=1.2, label='y1')
plt.ylabel('$\\psi$ [rad]')
plt.xlabel('Time [s]')
#plt.ylim([-0.15, 0.15])
plt.xlim(0, 30)
plt.grid(True)

# Subplot 622
plt.subplot(6, 2, 2)
plt.step(t, tau1[0, :len(t)], color='r', linewidth=1.2)
plt.ylabel('$\\tau_X$ [N]')
plt.xlim(0, 30)
plt.grid(True)

# Subplot 624
plt.subplot(6, 2, 4)
plt.step(t, tau1[1, :len(t)], color='r', linewidth=1.2)
plt.ylabel('$\\tau_Y$ [N]')
plt.xlim(0, 30)
plt.grid(True)

# Subplot 626
plt.subplot(6, 2, 6)
plt.step(t, tau1[2, :len(t)], color='r', linewidth=1.2)
plt.ylabel('$\\tau_Z$ [N]')
plt.xlim(0, 30)
plt.grid(True)

# Subplot 628
plt.subplot(6, 2, 8)
plt.step(t, tau1[3, :len(t)], color='r', linewidth=1.2)
plt.ylabel('$\\tau_K$ [Nm]')
plt.xlim(0, 30)
plt.grid(True)

# Subplot 6,2,10
plt.subplot(6, 2, 10)
plt.step(t, tau1[4, :len(t)], color='r', linewidth=1.2)
plt.ylabel('$\\tau_M$ [Nm]')
plt.xlim(0, 30)
plt.grid(True)

# Subplot 6,2,12
plt.subplot(6, 2, 12)
plt.step(t, tau1[5, :len(t)], color='r', linewidth=1.2)
plt.xlabel('Time [s]')
plt.ylabel('$\\tau_N$ [Nm]')
plt.xlim(0, 30)
plt.grid(True)

# Display the figure
plt.tight_layout()
plt.show()