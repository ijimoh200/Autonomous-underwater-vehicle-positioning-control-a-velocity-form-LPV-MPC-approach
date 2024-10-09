function xi = solver_RK(x,Ts,u,nu_c,dw)
% RK = Runge Kutta
k1 = Naminow_AUV(x,u,nu_c,dw);
k2 = Naminow_AUV(x+k1.*(Ts/2),u,nu_c,dw);
k3 = Naminow_AUV(x+k2.*(Ts/2),u,nu_c,dw);
k4 = Naminow_AUV(x+k3.*Ts,u,nu_c,dw);

xi = x + (Ts/6).*(k1+2.*k2+2.*k3 + k4);
end