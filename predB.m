function B_p =predB(C,A,B,N, Nu)
[p,~]=size(C);
[~,m]=size(B);
barC_= zeros(N*p,Nu*m);
firstCol = [];
for i=1:N
firstCol = [firstCol ;
C*A^(i-1)*B];
end
barC_(:,1:m) = firstCol;
 for col=1:(Nu-1)*m
 for row=1:(N-1)*p
barC_(row+p, col+m)=barC_(row,col);
 end
 end
B_p = barC_;
end