function A_p = predA(G,Aeta,N)
temp = [];
for i=1:N
 temp = [temp; G*Aeta^i];
end
A_p = temp;
end