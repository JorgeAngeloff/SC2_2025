<<<<<<< HEAD
%
=======
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
% Coeficientes del numerador (s*J + B)
b1 = Jm;
b0 = Bm;

% Reutilamos el denominador: a2, a1, a0 (ya definidos)

ia_va_teo = tf([b1 b0], [a2 a1 a0]);

% Simulamos la respuesta al escalón de 2V
[y_teo_ia, t_teo_ia] = step(StepAmplitude * ia_va_teo, t(end));

% --- GRAFICADO COMPARATIVO ---
figure;
plot(t, ia, 'b', 'LineWidth', 1.5); hold on;           % Datos reales de corriente
plot(t_estimada, y_estimada, 'r--', 'LineWidth', 1.5); % Estimada
plot(t_teo_ia, y_teo_ia, 'g:', 'LineWidth', 2);        % Teórica

title('Comparación: Corriente Real vs Estimada vs Teórica');
xlabel('Tiempo (s)');
ylabel('Corriente I_a (A)');
legend('Real', 'Estimada', 'Teórica', 'Location', 'best');
grid on;


%%
>>>>>>> c89398b (actualizando el repo postparcial)
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

<<<<<<< HEAD
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
=======
>>>>>>> c89398b (actualizando el repo postparcial)




<<<<<<< HEAD
%%
=======
%% 
>>>>>>> c89398b (actualizando el repo postparcial)

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
<<<<<<< HEAD
grid on;
=======
grid on;





%% Wr/TL
%
 clear; clc; close; 

data = xlsread('Curvas_Medidas_Motor_2025.xls', 'Hoja1');
t = data(401:700, 1);    % Tiempo (desde fila 1001 hasta 5000) que corresponde al primer escalon sin retardo
w = data(401:700, 2);    % Corriente
ia = data(401:700, 3);   % Tensión en el capacitor
v = data(401:700, 4);   % Tensión de entrada
TL = data(401:700, 5);   % Tensión de entrada

StepAmplitude = 0.12;  % Amplitud del escalón (0.12N.m)

% normalizo el t para que el escalón comience en t=0
t = t - t(1);  % Ahora t(1) = 0

w0 = w(1);          % Primer valor (7.5330)
w_adj = w - w0;     % Resta el primer valor (ahora empieza en 0)
w_inv = w_adj;     % Da vuelta la curva para que suba


% Selección de puntos para el método 
t_inic = 200; %0.065;  % Tiempo inicial para primer punto 
[val lugar] = min(abs(t_inic - t)); 
y_t1 = w_inv(lugar);
t_t1 = t(lugar);

[val lugar] = min(abs(2*t_inic - t));
y_2t1 = w_inv(lugar);

[val lugar] = min(abs(3*t_inic - t));
y_3t1 = w_inv(lugar);

% Cálculo de parámetros intermedios 
K = w_inv(end) / StepAmplitude;  % Ganancia estática 7.63273;%
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
w_tl = tf(K * [T3_ang 1], conv([T1_ang 1], [T2_ang 1]))

[y_estimada, t_estimada] = step(StepAmplitude * (w_tl), t(end));% para poder graficarla

% Verificacion gráfica
figure;
plot(t, (w_inv), 'b', 'LineWidth', 1.5); hold on;           % Datos reales en azul
plot(t_estimada, y_estimada, 'r--', 'LineWidth', 1.5); % Modelo estimado rojo punteado
hold off;

title('Comparación: Respuesta Real vs Estimada');
xlabel('Tiempo (s)');
ylabel('w (r/seg)');
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

% Coeficientes del numerador: -sLa - Ra
c1 = -La;
c0 = -Ra;

w_tl_teo = tf([c1 c0], [a2 a1 a0]);


% Simulo la respuesta al escalón para la misma duración
[y_teo_wtl, t_teo_wtl] = step(StepAmplitude * w_tl_teo, t(end));

% --- GRAFICADO ---
figure;
plot(t, w_inv, 'b', 'LineWidth', 1.5); hold on;                % Datos reales
plot(t_estimada, y_estimada, 'r--', 'LineWidth', 1.5);         % Estimada
plot(t_teo_wtl, y_teo_wtl, 'g:', 'LineWidth', 2);              % Teórica
hold off;

title('Comparación: Respuesta a TL - Real vs Estimada vs Teórica');
xlabel('Tiempo (s)');
ylabel('\omega (rad/s)');
legend('Real', 'Estimación', 'Teórica', 'Location', 'best');
grid on;
%%
% Parámetros físicos
La  = 0.002511;
Ki  = 3.765;
Jm  = 0.02056;
Km  = 0.2485;
Bm  = 0.026;
Ra  = 2.415;

% Modelo MIMO en variables de estado
A = [-Ra/La    -Ki/La;
      Km/Jm   -Bm/Jm];
B = [1/La     0;
     0       -1/Jm];
C = [0 1];   % salida: velocidad angular
D = [0 0];

sys_mimo = ss(A, B, C, D);

% Entradas: escalón en TL durante la simulación
Va = 2 * ones(size(t));          % Entrada de tensión constante
TL = zeros(size(t));             % Torque de carga inicialmente 0

% Definís el instante en que entra el escalón de TL
t_escalon = 0.065;               % segundo
idx = find(t >= t_escalon, 1);   % índice de inicio del escalón

TL(idx:end) = 0.12;              % Escalón en TL a partir de ese instante

U = [Va, TL];                    % Matriz de entradas

% Condiciones iniciales reales (de los datos)
x0 = [ia(1); w(1)];

% Simulación
[y_mimo, t_mimo, x_mimo] = lsim(sys_mimo, U, t, x0);

% Comparación gráfica
figure;
plot(t, w, 'b', 'LineWidth', 1.5); hold on;
plot(t, y_mimo, 'm--', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('\omega (rad/s)');
title('Comparación entre datos reales y modelo MIMO');
legend('Real', 'Simulada MIMO');
grid on;

%% 
%%

clear; clc; close; 

data = xlsread('Curvas_Medidas_Motor_2025_v.xls', 'Hoja1');
t = data(:, 1);    % Tiempo (desde fila 1001 hasta 5000) que corresponde al primer escalon sin retardo
w = data(:, 2);    % Corriente
ia = data(:, 3);   % Tensión en el capacitor
v = data(:, 4);   % Tensión de entrada
TL = data(:, 5);   % Tensión de entrada
%Grafico 
subplot(4,1,1); plot(t,w,'r'); title('w,t');  
subplot(4,1,2); plot(t,ia,'b'); title('ia,t'); 
subplot(4,1,3); plot(t,v,'g');  title('v,t');
subplot(4,1,4); plot(t,TL,'g');  title('TL,t');
% %Configuración de estilo para todos los plots
% set(0, 'DefaultAxesLineWidth', 1.5);  % Grosor de líneas de ejes
% set(0, 'DefaultLineLineWidth', 2.5);   % Grosor de líneas de datos
% 
% figure;  % Crear nueva figura
% 
% % Subplot 1: Velocidad (w) en rojo intenso
% subplot(4,1,1); 
% plot(t, w, 'r', 'Color', [0.8 0.1 0.1], 'LineWidth', 2.5); 
% title('Velocidad (w) vs tiempo', 'FontWeight', 'bold');
% xlabel('Tiempo [s]'); ylabel('w [rad/s]');
% grid on;
% 
% % Subplot 2: Corriente (ia) en azul oscuro
% subplot(4,1,2); 
% plot(t, ia, 'b', 'Color', [0.1 0.2 0.6], 'LineWidth', 2.5);
% title('Corriente (i_a) vs tiempo', 'FontWeight', 'bold'); 
% xlabel('Tiempo [s]'); ylabel('i_a [A]');
% grid on;
% 
% % Subplot 3: Voltaje (v) en verde esmeralda
% subplot(4,1,3); 
% plot(t, v, 'Color', [0.0 0.6 0.3], 'LineWidth', 2.5);
% title('Voltaje (v) vs tiempo', 'FontWeight', 'bold');
% xlabel('Tiempo [s]'); ylabel('v [V]');
% grid on;
% 
% % Subplot 4: Torque de carga (TL) en morado
% subplot(4,1,4);
% plot(t, TL, 'Color', [0.5 0.1 0.7], 'LineWidth', 2.5);
% title('Torque de carga (T_L) vs tiempo', 'FontWeight', 'bold');
% xlabel('Tiempo [s]'); ylabel('T_L [Nm]');
% grid on;
% 
% % Ajustar espacio entre subplots
% h = gcf; 
% set(h, 'Position', [100 100 800 900]);  % Tamaño de figura [ancho alto]
% set(gcf, 'Color', [0.95 0.95 0.95]);   % Fondo gris claro

>>>>>>> c89398b (actualizando el repo postparcial)
