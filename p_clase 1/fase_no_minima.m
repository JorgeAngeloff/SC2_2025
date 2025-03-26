P=1;%PM7.4 del DORF
num=[1+P -P];
den=[1 3 6];
G=tf(num,den)
rlocus(G)
step (G,10)
% 
T1=1/2; %Cero
T2=1/5;%Polo
alfa=T2/T1;%Da entre 0 y 1
Kc=1; % K del rlocus(G)
C=tf(Kc*[1 1/T1],[1 1/(T2) ])
rlocus(-C*G)
% step(G*C, 10)

FLC=feedback(-1.12*C*G,1); 
%bode(.3*C2*C*G)
roots(FLC.Den{1})
step(FLC,100)


Kp=1;Ki=1;
C2=tf(Kp*[1 Ki/Kp],[1 0]);
rlocus(-1*C*C2*G)

FLC=feedback(-1*C*C2*G,1); %
bode(.3*C2*C*G)
roots(FLC.Den{1}) % rlocus(-C2*G)
step(FLC,100)