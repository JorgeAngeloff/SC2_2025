clc; clear; close all;

%% Parámetros físicos del sistema
m = 0.1;           % masa del péndulo (kg)
M = 0.5;           % masa del carro (kg)
long = 0.6;        % longitud del péndulo (m)
g = 9.8;           % gravedad (m/s^2)
Fricc = 0.1;       % fricción viscosa del carro

%% Matrices del sistema linealizado alrededor del punto vertical (fi=0)
A = [0 1 0 0;
     0 -Fricc/M -m*g/M 0;
     0 0 0 1;
     0 Fricc/(long*M) g*(m+M)/(long*M) 0];
B = [0; 1/M; 0; -1/(long*M)];
C = [1 0 0 0;
     0 0 1 0];   % salidas: posición (delta), ángulo (phi)

%% --------------------------------------------------
% 🔧 Diseño del controlador LQR con integrador
%% --------------------------------------------------

% Ampliación del sistema para incluir integrador de error en posición
Aa = [A zeros(4,1);
     -C(1,:) 0];    % C(1,:) = salida de posición
Ba = [B; 0];

% Pesos del LQR ampliado
Q = diag([1e2, 1e2, 1e5, 1e5, 1]);   % penaliza estados + error integrado
R = 3e3;                             % penaliza acción de control

% Resolución de ecuación de Riccati vía Hamiltoniano
H = [Aa -Ba*(R\Ba'); -Q -Aa'];
[V,D] = eig(H);
V_estables = V(:, real(diag(D)) < 0);
MX1 = V_estables(1:5,:);
MX2 = V_estables(6:end,:);
P = real(MX2/MX1);

Ka = (R \ Ba') * P;
K = Ka(1:end-1);      % Ganancia sobre estados
KI = -Ka(end);        % Ganancia sobre el integrador

disp('Polos del sistema con LQR ampliado:');
disp(eig(Aa - Ba*Ka));

%% --------------------------------------------------
% 👁️ Diseño del Observador Luenberger vía LQR dual
%% --------------------------------------------------

A_o = A'; B_o = C';   % Dualidad observador-controlador
Q_o = diag([1, 1e2, 1e1, 1e5]);     % ruido de proceso
R_o = diag([1e-3, 1e-2]);          % ruido de medición

H_obs = [A_o -B_o*(R_o\B_o'); -Q_o -A_o'];
[V_o,D_o] = eig(H_obs);
V_estables_o = V_o(:, real(diag(D_o)) < 0);
MX1_o = V_estables_o(1:4,:);
MX2_o = V_estables_o(5:end,:);
P_o = real(MX2_o / MX1_o);

Ko = P_o * (C' / R_o);  % Ganancia del observador

disp('Polos del observador:');
disp(eig(A - Ko*C));

%% --------------------------------------------------
% 🔁 Simulación con dinámica no lineal
%% --------------------------------------------------

% Configuración de simulación
h = 1e-3; t_simul = 150; N = t_simul/h;
ref = 10;   % referencia en posición

% Inicialización de variables
fi = zeros(1,N); fi(1) = pi;     % ángulo inicial (vertical invertida)
omega = zeros(1,N);
p = zeros(1,N); p_p = zeros(1,N);
u = zeros(1,N); psi = zeros(1,N);  % integrador
y = zeros(1,N); y_o = zeros(1,N);  % salidas
p_O = zeros(1,N); fi_O = zeros(1,N); % estados estimados
estado = [p(1); p_p(1); fi(1); omega(1)];
x_hat = zeros(4,1);  % estimación inicial

% Bucle de simulación
for i = 1:N-1
    Y = C * estado;
    y(i) = Y(1);  % salida medida (posición)
    psi_p = ref - y(i);
    psi(i+1) = psi(i) + psi_p*h;

    % Controlador de estados + integrador
    u(i) = -Ka * [x_hat; psi(i+1)];

    % Dinámica no lineal del péndulo
    den = long * (4/3 - (m * cos(fi(i))^2) / (M + m));
    num = g * sin(fi(i)) - cos(fi(i)) * (u(i) + m * long * omega(i)^2 * sin(fi(i)) - Fricc * p_p(i)) / (M + m);
    fi_pp = num / den;
    p_pp = (1/(M+m))*(u(i) - m*long*fi_pp*cos(fi(i)) + m*long*omega(i)^2*sin(fi(i)) - Fricc*p_p(i));
    fi_pp = (1/long)*(g*sin(fi(i)) - p_pp*cos(fi(i)));  % recalculado

    % Observador
    Y_o = C * x_hat;
    y_o(i) = Y_o(1);
    x_hat_p = A*x_hat + B*u(i) + Ko*(Y - Y_o);
    x_hat = x_hat + h*x_hat_p;

    % Integración del sistema real
    p_p(i+1) = p_p(i) + h*p_pp;
    p(i+1) = p(i) + h*p_p(i);
    omega(i+1) = omega(i) + h*fi_pp;
    fi(i+1) = fi(i) + h*omega(i);

    estado = [p(i+1); p_p(i+1); fi(i+1); omega(i+1)];
    p_O(i+1) = x_hat(1);
    fi_O(i+1) = x_hat(3);
end

t = (0:N-1)*h;
u(N) = -Ka * [x_hat; psi(end)];

%% --------------------------------------------------
% 📊 Gráficas de resultados
%% --------------------------------------------------

figure('Name','Estados del Sistema','NumberTitle','off');
subplot(2,2,1); plot(t, p, 'LineWidth',1); ylabel('Posición \delta (m)'); grid on;
subplot(2,2,2); plot(t, fi, 'LineWidth',1); ylabel('Ángulo \phi (rad)'); grid on;
subplot(2,2,3); plot(t, p_p, 'LineWidth',1); ylabel('Vel. carro \deltȧ (m/s)'); xlabel('Tiempo (s)'); grid on;
subplot(2,2,4); plot(t, omega, 'LineWidth',1); ylabel('Vel. péndulo \phi̇ (rad/s)'); xlabel('Tiempo (s)'); grid on;
sgtitle('Evolución de los estados');

figure('Name','Acción de Control','NumberTitle','off');
plot(t, u, 'LineWidth',1.2); xlabel('Tiempo (s)'); ylabel('u(t) (N)'); title('Acción de control'); grid on;

figure('Name','Estimación del Observador','NumberTitle','off');
subplot(1,2,1); plot(t, p, t, p_O, '--', 'LineWidth',1.2);
xlabel('Tiempo (s)'); ylabel('Posición \delta'); legend('Real','Estimada'); grid on;
subplot(1,2,2); plot(t, fi, t, fi_O, '--', 'LineWidth',1.2);
xlabel('Tiempo (s)'); ylabel('Ángulo \phi'); legend('Real','Estimada'); grid on;
sgtitle('Comparación entre señales reales y estimadas');