function Q_p =predQ(Q_p, S, N)
temp1 = eye(N-1);
temp2 = kron(temp1, Q_p);
Q_p = blkdiag(temp2, S);
end