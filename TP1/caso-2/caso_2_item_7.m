clear; clc;

% Parámetros del motor
La = 2.43309;
Ra = 2.43309;
Ki = 0.411;
Km = 0.64803;
Jm = 0.00258956;
Bm = 0.0594551;

% Matriz A del sistema
A = [ -Ra/La, -Km/La, 0;
       Ki/Jm, -Bm/Jm, 0;
       0,     1,      0];

% Cálculo simbólico del determinante (polinomio característico)
syms s
I = eye(3);
char_poly = simplify(det(s*I - A)) % Polinomio característico
coeffs_char_poly = sym2poly(char_poly) % Coeficientes del denominador

% Encontrar polos
polos = roots(coeffs_char_poly)

%%
% Asumiendo que ya tenés los polos del sistema:
polos = [0; -20.8276; -3.1320];  

lambda_rapido = -20.8276;

% Cálculo de t_R y delta_t
t_R = log(0.95) / lambda_rapido;  % tiempo en que la dinámica rápida cae al 5%
At= t_R / 3




%% 
%Verificación de la EDO
% y1pp+y1p+y2=u1+u2p
% y2p=y1+u1p,
% Condic iniciales nulas, respuestas al escalon.
clear all;clc;close all;

T=10; Kmax=20000; At=T/Kmax;TamanioFuente=12;

u1=zeros(1,Kmax);
y1=u1;
y2=u1;
y1p=y1;
y2p=y2;

t=0:At:T-At;y1pp=0;

u1=[0 2*cos(10*t)];
u1p=[diff(u1)/At];
u2=[0 2*sin(1*t)]; 
u2p=[diff(u2)/At];

for jj=1:Kmax-1
%y1pp+y1p+y2=u1+u2p;
%y2p=y1+u1p;
y1pp=-y1p(jj)-y2(jj)+u1(jj)+u2p(jj);
y2p(jj)=y1(jj)+u1p(jj);

%Integro
y2(jj+1) =y2(jj) +y2p(jj)*At;
y1p(jj+1)=y1p(jj)+y1pp*At;
y1(jj+1) =y1(jj) +y1p(jj)*At;
end

figure;title('Sistema MIMO');
subplot(2,2,1);plot(t,y1,'.k');title('y_1'),hold on
subplot(2,2,2);plot(t,y2,'.k');title('y_2'); hold on
subplot(2,2,3);plot(t,u1(1:numel(t)),'k');title('u_1');xlabel('t [seg.]');
subplot(2,2,4);plot(t,u2(1:numel(t)),'k');title('u_2');xlabel('t [seg.]');


A=[0,1,0;0,-1,-1;1,0,0];
B=[0, 1;0, -1;0,0];
C=[1, 0,0;0,0,1];
D=[0 0;1 0];
Sis_ve=ss(A,B,C,D)
% [num, den]=ss2tf(Sis_ve.a,Sis_ve.b,Sis_ve.c,Sis_ve.d,1);
% sys1=tf(num,den)
% [num, den]=ss2tf(Sis_ve.a,Sis_ve.b,Sis_ve.c,Sis_ve.d,2);
% sys2=tf(num,den)

[num, den] = ss2tf(Sis_ve.a, Sis_ve.b, Sis_ve.c, Sis_ve.d, 1); % salida 1

% num es un array 2xN (porque hay 2 entradas)
sys11 = tf(num(1,:), den) % salida 1 respecto entrada 1 (u1)
sys12 = tf(num(2,:), den) % salida 1 respecto entrada 2 (u2)

[num, den] = ss2tf(Sis_ve.a, Sis_ve.b, Sis_ve.c, Sis_ve.d, 2); % salida 2

sys21 = tf(num(1,:), den) % salida 2 respecto entrada 1
sys22 = tf(num(2,:), den) % salida 2 respecto entrada 2


x=[0;0;0];U=[0;0];
y_1(1)=0;
y_2(1)=0;

for jj=1:Kmax-1

U(1)=u1(jj);
U(2)=u2(jj);

xp=A*x+B*U;
y =C*x+D*U;

x=x+xp*At;

y_1(jj+1)=y(1);
y_2(jj+1)=y(2);

end
subplot(2,2,1);plot(t,y_1,'r');
subplot(2,2,2);plot(t,y_2,'r');legend('EDO','Evolución en VE');legend('boxoff');title('y_2','FontSize',TamanioFuente);

%%
clear all; clc; %close all;

% --- Parámetros físicos del motor ---
La  = 2.43309;
Ra  = 2.43309;
Ki  = 0.411;
Km  = 0.64803;
Bm  = 0.0594551;
Jm  = 0.00258956;

% --- Tiempo de simulación ---
T = 10;           
Kmax = 10000;     
At = T / Kmax;    

% --- Inicialización de variables de estado ---
ia = zeros(1,Kmax);
wr = zeros(1,Kmax);
theta = zeros(1,Kmax);

% --- Derivadas ---
iap = ia;
wrp = wr;
thetap = wr;

% --- Entradas ---
Va = zeros(1,Kmax);
TL = zeros(1,Kmax);
theta_ref = zeros(1,Kmax);

% --- PID: parámetros y discretización ---
Kp = 2;
Ki_ = 0;
Kd = 0.1;
Ts = At;

A1 = ((2*Kp*Ts) + (Ki_*(Ts^2)) + (2*Kd)) / (2*Ts);
B1 = (-2*Kp*Ts + Ki_*(Ts^2) - 4*Kd) / (2*Ts);
C1 = (2*Kd) / (2*Ts);

% --- Buffers de error (PID) ---
e = zeros(1,Kmax);
u = zeros(1,Kmax);  % Control Va generado por PID

% --- Referencia y TL alternados cada 2 s ---
for jj = 1:Kmax
    t = (jj-1)*At;
    if mod(floor(t/2),2) == 0
        theta_ref(jj) = pi/2;
        TL(jj) = 0.1;
    else
        theta_ref(jj) = -pi/2;
        TL(jj) = 0;
    end
end

% --- Simulación con integración de Euler ---
for jj = 3:Kmax-1  % Empieza en 3 para usar e(jj-2)
    
    % --- Calcular error ---
    e(jj) = theta_ref(jj) - theta(jj);
    
    % --- PID discreto (forma recursiva) ---
    u(jj) = u(jj-1) + A1*e(jj) + B1*e(jj-1) + C1*e(jj-2);

    % Saturación del control Va
    Va(jj) = max(min(u(jj), 12), -12);

    % --- Ecuaciones de estado ---
    iap(jj) = (-Ra * ia(jj) - Km * wr(jj) + Va(jj)) / La;
    wrp(jj) = (Ki * ia(jj) - Bm * wr(jj) - TL(jj)) / Jm;
    thetap(jj) = wr(jj);

    % --- Integración de Euler ---
    ia(jj+1)    = ia(jj) + iap(jj) * At;
    wr(jj+1)    = wr(jj) + wrp(jj) * At;
    theta(jj+1) = theta(jj) + thetap(jj) * At;
end

% --- Eje de tiempo ---
t = 0:At:T-At;

% --- Gráficas ---
figure('Name','Simulación Motor CC con PID');
subplot(3,2,1); plot(t, Va(1:end), 'b'); title('Entrada Va (PID)'); xlabel('t [s]');
subplot(3,2,2); plot(t, TL(1:end), 'r'); title('Torque de carga TL'); xlabel('t [s]');
subplot(3,2,3); plot(t, ia, 'k'); title('Corriente del inducido i_a'); xlabel('t [s]');
subplot(3,2,4); plot(t, wr, 'g'); title('Velocidad angular \omega_r'); xlabel('t [s]');
subplot(3,2,5); plot(t, theta, 'm'); hold on;
plot(t, theta_ref, '--k'); title('Posición angular \theta vs referencia'); xlabel('t [s]');
legend('\theta','\theta_{ref}'); legend boxoff
subplot(3,2,6); plot(t, u, 'c'); title('Salida PID antes de saturar'); xlabel('t [s]');
