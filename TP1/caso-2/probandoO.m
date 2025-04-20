%
%Wr/Va
%
clear; clc; close; 

data = xlsread('Curvas_Medidas_Motor_2025.xls', 'Hoja1');
t = data(201:400, 1);    % Tiempo (desde fila 1001 hasta 5000) que corresponde al primer escalon sin retardo
w = data(201:400, 2);    % Corriente
ia = data(201:400, 3);   % Tensión en el capacitor
v = data(201:400, 4);   % Tensión de entrada
TL = data(201:400, 5);   % Tensión de entrada


StepAmplitude = 2;  % Amplitud del escalón (2V)

% normalizo el t para que el escalón comience en t=0
t = t - t(1);  % Ahora t(1) = 0

% Selección de puntos para el método 
t_inic = 64e-3;%0.065;%5e-3; %0.065;  % Tiempo inicial para primer punto 
[val lugar] = min(abs(t_inic - t)); 
y_t1 = w(lugar);
t_t1 = t(lugar);

[val lugar] = min(abs(2*t_inic - t));
y_2t1 = w(lugar);

[val lugar] = min(abs(3*t_inic - t));
y_3t1 = w(lugar);

% Cálculo de parámetros intermedios 
K =w(end) / StepAmplitude;  % Ganancia estática 7.63273;%
k1 = (y_t1 / StepAmplitude) / K - 1;  % Valores normalizados
k2 = (y_2t1 / StepAmplitude) / K - 1;
k3 = (y_3t1 / StepAmplitude) / K - 1;

% Cálculo de constantes de tiempo para polos distintos
be = 4*k1^3*k3 - 3*k1^2*k2^2 - 4*k2^3 + k3^2 + 6*k1*k2*k3;
alfa1 = (k1*k2 + k3 - sqrt(be)) / (2*(k1^2 + k2));
alfa2 = (k1*k2 + k3 + sqrt(be)) / (2*(k1^2 + k2));
beta = (k1 + alfa2) / (alfa1 - alfa2);

% Conversión a constantes de tiempo físicas
T1_ang = -t_t1 / log(alfa1);
T2_ang = -t_t1 / log(alfa2);
T3_ang = beta * (T1_ang - T2_ang) + T1_ang;

% Función de transferencia estimada
w_va = tf(K * [T3_ang 1], conv([T1_ang 1], [T2_ang 1]))

[y_estimada, t_estimada] = step(StepAmplitude * w_va, t(end));% para poder graficarla

% Verificacion gráfica
figure;
plot(t, w, 'b', 'LineWidth', 1.5); hold on;           % Datos reales en azul
plot(t_estimada, y_estimada, 'r--', 'LineWidth', 1.5); % Modelo estimado rojo punteado
hold off;

title('Comparación: Respuesta Real vs Estimada');
xlabel('Tiempo (s)');
ylabel('w (r/seg)');
legend('Real', 'Estimada', 'Location', 'best');
grid on;

%Ia/Va
%
clear;  close; 

data = xlsread('Curvas_Medidas_Motor_2025.xls', 'Hoja1');
t = data(201:400, 1);    % Tiempo (desde fila 1001 hasta 5000) que corresponde al primer escalon sin retardo
w = data(201:400, 2);    % Corriente
ia = data(201:400, 3);   % Tensión en el capacitor
v = data(201:400, 4);   % Tensión de entrada
TL = data(201:400, 5);   % Tensión de entrada


StepAmplitude = 2;  % Amplitud del escalón (2V)

% normalizo el t para que el escalón comience en t=0
t = t - t(1);  % Ahora t(1) = 0

% Selección de puntos para el método 
t_inic = 55e-3;%5e-3; %0.065;  % Tiempo inicial para primer punto 
[val lugar] = min(abs(t_inic - t)); 
y_t1 = ia(lugar);
t_t1 = t(lugar);

[val lugar] = min(abs(2*t_inic - t));
y_2t1 = ia(lugar);

[val lugar] = min(abs(3*t_inic - t));
y_3t1 = ia(lugar);

% Cálculo de parámetros intermedios 
K =ia(end) / StepAmplitude;  % Ganancia estática 7.63273;%
k1 = (y_t1 / StepAmplitude) / K - 1;  % Valores normalizados
k2 = (y_2t1 / StepAmplitude) / K - 1;
k3 = (y_3t1 / StepAmplitude) / K - 1;

% Cálculo de constantes de tiempo para polos distintos
be = 4*k1^3*k3 - 3*k1^2*k2^2 - 4*k2^3 + k3^2 + 6*k1*k2*k3;
alfa1 = (k1*k2 + k3 - sqrt(be)) / (2*(k1^2 + k2));
alfa2 = (k1*k2 + k3 + sqrt(be)) / (2*(k1^2 + k2));
beta = (k1 + alfa2) / (alfa1 - alfa2);

% Conversión a constantes de tiempo físicas
T1_ang = -t_t1 / log(alfa1);
T2_ang = -t_t1 / log(alfa2);
T3_ang = beta * (T1_ang - T2_ang) + T1_ang;

% Función de transferencia estimada
ia_va = tf(K * [T3_ang 1], conv([T1_ang 1], [T2_ang 1]))

[y_estimada, t_estimada] = step(StepAmplitude * ia_va, t(end));% para poder graficarla

% Verificacion gráfica
figure;
plot(t, ia, 'b', 'LineWidth', 1.5); hold on;           % Datos reales en azul
plot(t_estimada, y_estimada, 'r--', 'LineWidth', 1.5); % Modelo estimado rojo punteado
hold off;

title('Comparación: Respuesta Real vs Estimada');
xlabel('Tiempo (s)');
ylabel('ia(A)');
legend('Real', 'Estimada', 'Location', 'best');
grid on;




%%

% --- FUNCIÓN DE TRANSFERENCIA TEÓRICA ---
% Parámetros físicos conocidos (1ra solución)
La  = 0.002511
Ki  = 3.765
Jm  = 0.02056
Km  = 0.2485
Bm  = 0.026
Ra  = 2.415

% Coeficientes del denominador
a2 = (La * Jm);
a1 = (Ra * Jm) + (La * Bm);
a0 = (Ra * Bm) + (Ki * Km);

% Función de transferencia teórica
w_va_teo = tf(Ki, [a2 a1 a0])

% Simulo la respuesta al escalón para la misma duración
[y_teo, t_teo] = step(StepAmplitude * w_va_teo, t(end));

% --- GRAFICADO ---
figure;
plot(t, w, 'b', 'LineWidth', 1.5); hold on;             % Datos reales
plot(t_estimada, y_estimada, 'r--', 'LineWidth', 1.5);  % Modelo estimado
plot(t_teo, y_teo, 'g:', 'LineWidth', 2);               % Modelo teórico

title('Comparación: Respuesta Real vs Estimada vs Teórica');
xlabel('Tiempo (s)');
ylabel('Velocidad angular \omega (rad/s)');
legend('Real','Estimada','Teórica','Location','best');
grid on;