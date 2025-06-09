clc;clear all;
A=[0 1; -1 -1];
eig(A)

syms P11 P12 P22;

P=[P11 P12;P12 P22];
H=A'*P+P*A+eye(2);
[a11,a12,a22]=solve(H(1,1),H(1,2),H(2,2));
M_P=eval([a11,a12;a12,a22])
if(M_P>0)
fprintf('P es definida positiva \n');
else
fprintf('P no es definida positiva \n');
end
