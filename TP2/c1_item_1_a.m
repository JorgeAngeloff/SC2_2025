%% a
close all; clear; clc;

% ==== PARÁMETROS DEL SISTEMA ====
Ra = 2.258930051299405;
Laa = 0.005026901184834716;
Ki_ = 0.25965987053759737;
Jm = 0.0028472626983113334;
Bm = 0.0014165170369840668;
Km = 0.2500481104997174;

A = [-Ra/Laa, -Km/Laa, 0;
      Ki_/Jm, -Bm/Jm, 0;
      0,       1,     0];

B = [1/Laa; 0; 0];
C = [0, 0, 1];

% ==== TIEMPO DE SIMULACIÓN Y CONTROL ====
Tf = 10;                  % Duración total
Ts = 0.001;               % Tiempo de muestreo (inicial)

%Tint = 0.0004247701268;   % Paso de integración motor
t = 0:Ts:Tf;

% ==== REFERENCIA ====
ref = zeros(size(t));
for k = 1:length(t)
    ref(k) = (-1)^(floor(t(k)/5)) * pi/2;
end

% ==== CONTROLADOR LQI ====
% Qi = 1000;
% Qx = diag([0.01, 0.1, 5]);
% Q = blkdiag(Qx, Qi);
Q = diag([0.01, 0.1, 5,1000]);
R = 1;

Aa = [A, zeros(3,1); -C, 0];
Ba = [B; 0];

%Ka = lqr(Aa, Ba, Q, R);%Directo

%%%Obtener Ka con el resolviendo la ecuacion de Riccati
% Hamiltoniano
H = [Aa   -Ba*(R\Ba');
     -Q   -Aa'];
% Autovalores y vectores
[V,D] = eig(H);
% Vectores asociados a autovalores estables (parte real negativa)
V_estables = V(:, real(diag(D)) < 0);
% Separar componentes
MX1 = V_estables(1:4,:); %M
MX2 = V_estables(5:end,:);%PM
% Matriz de Riccati
P = real(MX2 / MX1);
% Ganancia óptima
Ka = (R \ Ba') * P;
%%%

K = Ka(1:3);
Ki = Ka(4);

% ==== ESTADOS ====
X = zeros(3,1);  % Estado completo medido
ei = 0;

% ==== GUARDADO ====
Y  = zeros(1, length(t));  % Salida
U  = zeros(1, length(t));  % Control
E  = zeros(1, length(t));  % Error
Ei = zeros(1, length(t));  % Estado del integrador
TLplot = zeros(1, length(t)); % Torque de carga
X1 = zeros(1, length(t));  % Corriente
X2 = zeros(1, length(t));  % Velocidad
X3 = zeros(1, length(t));  % Ángulo

% ==== SIMULACIÓN ====
for k = 1:length(t)
    % Error e integrador
    e = ref(k) - X(3);
    ei = ei + e * Ts;

    % Control sin observador
    u = -K*X - Ki*ei;
    u = max(min(u, 5), -5);  % Saturación
    
   if t(k) >= 0.7
    TL = 0.12;  % Carga aplicada desde 0.7s
else
    TL = 0;
end

       % Guardar
    Y(k)  = X(3);
    U(k)  = u;
    E(k)  = e;
    Ei(k) = ei;
    TLplot(k) = TL;
    X1(k) = X(1);
    X2(k) = X(2);
    X3(k) = X(3);

    % Simular planta real no lineal
    X = modmotor(Ts, X, [u; TL]);
end

% ==== GRÁFICAS ====
figure;

subplot(4,1,1);
plot(t, X3, 'b', t, ref, 'k--'); 
ylabel('\theta (rad)');
legend('Salida', 'Referencia');
title('Salida angular \theta vs Referencia');
grid on;

subplot(4,1,2);
plot(t, X1, 'm');
ylabel('Corriente (A)');
title('Corriente del motor');
grid on;

subplot(4,1,3);
plot(t, TLplot, 'g');
ylabel('T_L (Nm)');
title('Torque de carga');
grid on;

subplot(4,1,4);
plot(t, U, 'r');
xlabel('Tiempo (s)');
ylabel('u(t) [V]');
title('Acción de control');
grid on;