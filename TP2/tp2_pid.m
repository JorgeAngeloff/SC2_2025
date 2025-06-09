%%%%%%%%%%%
clear; clc;

% Parámetros del PID
Kp = 10.0;
Ki = 10.0;
Kd = 0.1;

% Constantes identificadas (para coherencia)
Ra = 2.258930051299405;
Laa = 0.005026901184834716;
Ki_ = 0.25965987053759737;
Jm = 0.0028472626983113334;
Bm = 0.0014165170369840668;
Km = 0.2500481104997174;

% Configuración de simulación
tF = 10;
h = 0.0004247701268049;
t_etapa = 0.001; % Paso grande (como entre filas en Excel)
N = round(tF / t_etapa);

% Cargar datos del archivo Excel
data = readmatrix('Curvas_Medidas_Motor_2025_v.xls');
t_D = data(:,1);
TL_D = data(:,5);
TL_Max = max(TL_D);

% Inicialización de vectores
e = zeros(1, N+2);
u = 0;
x1 = zeros(1, N); % Corriente
x2 = zeros(1, N); % Velocidad
x3 = zeros(1, N); % Ángulo
acc = zeros(1, N); % Va
TL_D1 = zeros(1, N);
titaRef_ = zeros(1, N);

% Coeficientes PID discretos (Euler hacia atrás)
A1 = ((2*Kp*t_etapa) + (Ki*t_etapa^2) + (2*Kd)) / (2*t_etapa);
B1 = (-2*Kp*t_etapa + Ki*t_etapa^2 - 4*Kd) / (2*t_etapa);
C1 = Kd / t_etapa;

X = [0; 0; 0];
k = 3;

for jj = 1:N
    tiempo = (jj-1) * t_etapa;

    % Referencia y torque
    if tiempo > 0.7
        TL_ap = TL_Max;
    else
        TL_ap = 0;
    end

    if tiempo <= 5
        titaRef = pi/2;
    else
        titaRef = -pi/2;
        TL_ap = TL_Max;
    end

    % Simulación del modelo del motor
    X = modmotor(t_etapa, X, [u; TL_ap]);

    e(k) = titaRef - X(3);

    u = u + A1*e(k) + B1*e(k-1) + C1*e(k-2);

    % Almacenamiento
    x1(jj) = X(1);
    x2(jj) = X(2);
    x3(jj) = X(3);
    acc(jj) = u;
    TL_D1(jj) = TL_ap;
    titaRef_(jj) = titaRef;

    k = k + 1;
end

t = linspace(0, tF, N);
%%%%%%%%%%%%%%%%%%%%%%%

figure;

subplot(4,1,1)
plot(t, titaRef_, '--', t, x3, 'LineWidth', 1.5);
title('Control PID. Salida: Ángulo');
ylabel('\theta(t)');
legend('Referencia', '\theta');
grid on;

subplot(4,1,2)
plot(t, x1, 'b');
ylabel('i_a(t)');
grid on;

subplot(4,1,3)
plot(t, TL_D1, 'm');
ylabel('T_L(t)');
grid on;

subplot(4,1,4)
plot(t, acc, 'k');
ylabel('v_a(t)');
xlabel('Tiempo [s]');
grid on;
