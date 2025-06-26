clc; clear; close all;

%% Parámetros físicos del sistema (caso estable)
m = 0.1;           % masa del péndulo (kg)
M = 1.5;           % masa del carro (kg)
long = 1.6;        % longitud del péndulo (m)
g = 9.8;           % gravedad (m/s^2)
Fricc = 0.1;       % fricción viscosa

%% Matrices del sistema linealizado alrededor de phi = pi (estable)
A = [0 1 0 0;
     0 -Fricc/M -m*g/M 0;
     0 0 0 1;
     0 -Fricc/(long*M) -g*(m+M)/(long*M) 0];
B = [0; 1/M; 0; 1/(long*M)];
C = [1 0 0 0;
     0 0 1 0];   % se miden posición y ángulo

%% Diseño del controlador LQR ampliado con integrador
Aa = [A zeros(4,1);
     -C(1,:) 0];
Ba = [B; 0];
Aa
Ba

Q = diag([8e2, 1e1, 1e4, 1e3, 500]);   % pesos del estado + integrador
R = 1e2;


% Ka = lqr(Aa, Ba, Q, R);%Directo


H = [Aa -Ba*(R\Ba'); -Q -Aa'];
[V,D] = eig(H);
V_estables = V(:, real(diag(D)) < 0);
MX1 = V_estables(1:5,:);%M
MX2 = V_estables(6:end,:);%PM
P = real(MX2 / MX1);
Ka = (R \ Ba') * P;

% K = Ka(1:4);     % ganancia sobre estado
% KI = -Ka(end);   % ganancia sobre el integrador

%% Diseño del observador por dualidad (LQR)
A_o = A'; B_o = C';    % sistema dual
Q_o = diag([10, 1e2, 1, 1e3]);   % ruido de proceso
R_o = diag([1e-2, 1e-1]);        % ruido de medición

H_o = [A_o -B_o*(R_o\B_o'); -Q_o -A_o'];
[V_o,D_o] = eig(H_o);
V_estables_o = V_o(:, real(diag(D_o)) < 0);
MX1_o = V_estables_o(1:4,:);
MX2_o = V_estables_o(5:end,:);
P_o = real(MX2_o / MX1_o);

Ko = P_o * (C' / R_o);  % ganancia del observador

%% Simulación con dinámica no lineal
h = 1e-3; t_simul = 50; N = t_simul/h;
ref = [10*ones(1,N/2), zeros(1,N/2)];  % la primer mitad de la simulacion 10m, la otra mitad 0

% Variables de estado reales
delta = zeros(1,N); delta_p = zeros(1,N);
phi = zeros(1,N); omega = zeros(1,N);

% Variables estimadas y control
x_hat = zeros(4,1);
u = zeros(1,N); psi = zeros(1,N);  % integrador
delta_O = zeros(1,N); phi_O = zeros(1,N);

phi(1) = pi; %el angulo comienza en su equilibrio estable

estado = [delta(1); delta_p(1); phi(1); omega(1)];

for i = 1:N-1
   
    if delta(i) >= 9.99 && m ~= 1.0
    m = 1.0;
    fprintf('>> Cambio de masa a m = %.2f kg en t = %.2f s (x = %.2f m)\n', m, i*h, delta(i));
end

    % Salida medida
    Y = C * estado;
    psi_p = ref(i) - Y(1);
    psi(i+1) = psi(i) + psi_p*h;

    % Control LQR + integrador
    u(i) = -Ka * [x_hat; psi(i+1)];

    % Dinámica no lineal (no linealidad real)
    den = long * (4/3 - m*cos(phi(i))^2/(m+M));
    num = g*sin(phi(i)) - cos(phi(i))*(u(i) + m*long*omega(i)^2*sin(phi(i)) - Fricc*delta_p(i))/(M+m);
    phi_pp = num / den;
    delta_pp = (1/(M+m))*(u(i) - m*long*phi_pp*cos(phi(i)) + m*long*omega(i)^2*sin(phi(i)) - Fricc*delta_p(i));

    % Observador
    Y_o = C * x_hat;
    x_hat_p = A*x_hat + B*u(i) + Ko*(Y - Y_o);
    x_hat = x_hat + h*x_hat_p;

    % Integración del sistema real
    delta_p(i+1) = delta_p(i) + h*delta_pp;
    delta(i+1) = delta(i) + h*delta_p(i);
    omega(i+1) = omega(i) + h*phi_pp;
    phi(i+1) = phi(i) + h*omega(i);
    estado = [delta(i+1); delta_p(i+1); phi(i+1); omega(i+1)];

    delta_O(i+1) = x_hat(1);
    phi_O(i+1) = x_hat(3);
end
t = (0:N-1)*h;
u(N) = -Ka * [x_hat; psi(end)];

%% Gráficas
figure('Name','Estados del Sistema');
subplot(2,2,1); plot(t, delta); ylabel('\delta (m)'); grid on;
subplot(2,2,2); plot(t, phi); ylabel('\phi (rad)'); grid on;
subplot(2,2,3); plot(t, delta_p); ylabel('\deltȧ (m/s)'); xlabel('Tiempo (s)'); grid on;
subplot(2,2,4); plot(t, omega); ylabel('\phi̇ (rad/s)'); xlabel('Tiempo (s)'); grid on;
sgtitle('Evolución de los Estados');

figure('Name','Acción de Control');
plot(t, u); xlabel('Tiempo (s)'); ylabel('u(t) (N)'); grid on;

% figure('Name','Observador');
% subplot(1,2,1); plot(t, delta, t, delta_O, '--'); legend('Real','Estimada'); ylabel('\delta'); grid on;
% subplot(1,2,2); plot(t, phi, t, phi_O, '--'); legend('Real','Estimada'); ylabel('\phi'); grid on;
% sgtitle('Comparación Estado Real vs Estimado');
