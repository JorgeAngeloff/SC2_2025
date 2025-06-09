%% b)
 close all;clear; clc;

% ==== PARÁMETROS DEL SISTEMA (modelo de control) ====
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
D = 0;

% ==== TIEMPO DE SIMULACIÓN ====
Tf = 10;                 % 20 segundos
Ts = 0.001;              % Tiempo de muestreo del controlador
%Tint = 0.0004247701268;  % Paso de integración del modelo real
t = 0:Ts:Tf;

% ==== REFERENCIA (±pi/2 alternando cada 5s) ====
ref = zeros(size(t));
for k = 1:length(t)
    ref(k) = (-1)^(floor(t(k)/5)) * pi/2;
end

% ==== CONTROLADOR LQI ====
Qi = 1000;              % Penaliza error de seguimiento
Qx = diag([0.01, 0.1, 5]);
Q = blkdiag(Qx, Qi);    % Matriz de penalización ampliada
R = 1;

Aa = [A, zeros(3,1); -C, 0];
Ba = [B; 0];
Ka = lqr(Aa, Ba, Q, R);
K = Ka(1:3);
Ki = Ka(4);

% ==== OBSERVADOR ====
% Se mide θ (posición) y ω (velocidad), no ia
Cobs = [0 1 0; 0 0 1];

%Armo el observador con asignacion de polos OPCION1
%L = place(A', Cobs', [-100 -400 -1000])';

% Armo observador con LQE Matrices duales (observador) OPCION2
A_o = A';
B_o = Cobs';

% Pesos del observador
Q_o = diag([1e1, 1e2, 1e1]);  % Penaliza error en estimación
R_o = 1e-3;                % Penaliza ruido de medición (o incertidumbre)

% Hamiltoniano del observador
H_obs = [A_o, -B_o*(1/R_o)*B_o';
         -Q_o, -A_o'];

% Autovalores y autovectores
[V_o, D_o] = eig(H_obs);

% Extraer parte estable
V_estables_o = V_o(:, real(diag(D_o)) < 0);
MX1_o = V_estables_o(1:size(A,1), :);
MX2_o = V_estables_o(size(A,1)+1:end, :);

% Matriz de Riccati
P_o = real(MX2_o / MX1_o);

% Ganancia del observador (dual)
L = (P_o * Cobs') / R_o;


% ==== ESTADOS INICIALES ====
X_est = zeros(3,1);     % Estimado: [ia; wr; θ]
X_real = zeros(3,1);    % Real del modelo no lineal
ei = 0;                 % Integrador del error

% ==== DATOS PARA PLOTEO ====
Y = zeros(1, length(t));      % Salida real (theta)
U = zeros(1, length(t));      % Tensión de control

% Estados reales
X1_real = zeros(1, length(t)); % Corriente real
X2_real = zeros(1, length(t)); % Velocidad real
X3_real = zeros(1, length(t)); % Posición real

% Estados estimados
X1_est = zeros(1, length(t));  % Corriente estimada
X2_est = zeros(1, length(t));  % Velocidad estimada
X3_est = zeros(1, length(t));  % Posición estimada

% Torque de carga
TLplot = zeros(1, length(t));  % Carga aplicada


% ==== SIMULACIÓN ====
for k = 1:length(t)
    % Salidas medidas
    y_medido = [X_real(2); X_real(3)];
    y_est = [X_est(2); X_est(3)];

    % Error e integrador
    e = ref(k) - X_est(3);
    ei = ei + e * Ts;

    % Control
    u = -K * X_est - Ki * ei;
    u = max(min(u, 5), -5);  % saturación ±5V
    %torque
       if t(k) >= 0.7
    TL = 0.12;  % Carga aplicada desde 0.7s
else
    TL = 0;
end

       % Guardar datos
    Y(k) = X_real(3);        % Salida real
    U(k) = u;
    TLplot(k) = TL;

    % Estados reales
    X1_real(k) = X_real(1);
    X2_real(k) = X_real(2);
    X3_real(k) = X_real(3);

    % Estados estimados
    X1_est(k) = X_est(1);
    X2_est(k) = X_est(2);
    X3_est(k) = X_est(3);

    % Actualizar planta real
    X_real = modmotor(Ts, X_real, [u; TL]);

    % Actualizar observador (discreto)
    X_est = X_est + Ts*(A*X_est + B*u + L*(y_medido - y_est));
end

% ==== GRÁFICOS ====
figure;

% Ángulo θ
subplot(4,1,1);
plot(t, X3_real, 'b', t, X3_est, 'b--', t, ref, 'k:');
ylabel('\theta (rad)');
legend('Real', 'Estimado', 'Referencia');
title('Salida angular \theta');
grid on;

% Corriente
subplot(4,1,2);
plot(t, X1_real, 'm', t, X1_est, 'm--');
ylabel('Corriente (A)');
legend('Real', 'Estimada');
title('Corriente del motor');
grid on;

% Torque de carga
subplot(4,1,3);
plot(t, TLplot, 'g');
ylabel('T_L (Nm)');
title('Torque de carga aplicado');
grid on;

% Acción de control
subplot(4,1,4);
plot(t, U, 'r');
xlabel('Tiempo (s)');
ylabel('u(t) [V]');
title('Acción de control');
grid on;