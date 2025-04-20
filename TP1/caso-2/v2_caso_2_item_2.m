clear; clc; close; 

data = xlsread('Curvas_Medidas_Motor_2025_v.xls', 'Hoja1');
t = data(101:700, 1);    % Tiempo (desde fila 1001 hasta 5000) que corresponde al primer escalon sin retardo
w = data(101:700, 2);    % Corriente
ia = data(101:700, 3);   % Tensión en el capacitor
v = data(101:700, 4);   % Tensión de entrada
TL = data(101:700, 5);   % Tensión de entrada


StepAmplitude = 2;  % Amplitud del escalón (2V)

% normalizo el t para que el escalón comience en t=0
t = t - t(1);  % Ahora t(1) = 0

% Selección de puntos para el método 
t_inic = 0.07;    % Tiempo inicial para primer punto 
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
%%
% Parámetros físicos obtenidos
La = 2.43309;
Ki = 0.411;
Jm = 0.00258956;
Km = 0.64803;
Bm = 0.0594551;
Ra = 2 / 0.822;

% Coeficientes del denominador
a2 = Jm * La;
a1 = Jm * Ra + Bm * La;
a0 = Bm * Ra + Ki * Km;

% Numerador
K = Ki;

% Función de transferencia teórica
s = tf('s');
w_va_teorica = K / (a2*s^2 + a1*s + a0);

% Respuesta al escalón
[y_estimada, t_estimada] = step(StepAmplitude * w_va, t);
[y_teorica, t_teorica] = step(StepAmplitude * w_va_teorica, t);

% Gráfico comparativo
figure;
plot(t, w, 'k', 'LineWidth', 1.5); hold on;
plot(t_estimada, y_estimada, 'r--', 'LineWidth', 1.5);
plot(t_teorica, y_teorica, 'b-.', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('\omega (rad/s)');
title('Comparación: Real vs Estimada vs Teórica');
legend('Real', 'Estimada', 'Teórica', 'Location', 'best');
grid on;




%%

clear; clc; close; 

data = xlsread('Curvas_Medidas_Motor_2025.xls', 'Hoja1');
t = data(701:1000, 1);    % Tiempo (desde fila 1001 hasta 5000) que corresponde al primer escalon sin retardo
w = data(701:1000, 2);    % Corriente
ia = data(701:1000, 3);   % Tensión en el capacitor
v = data(701:1000, 4);   % Tensión de entrada
TL = data(701:1000, 5);   % Tensión de entrada

StepAmplitude = 0.12;  % Amplitud del escalón (0.12N.m)

% normalizo el t para que el escalón comience en t=0
t = t - t(1);  % Ahora t(1) = 0

w0 = w(1);          % Primer valor (7.5330)
w_adj = w - w0;     % Resta el primer valor (ahora empieza en 0)
w_inv = -w_adj;     % Da vuelta la curva para que suba


% Selección de puntos para el método 
t_inic = 250; %0.065;  % Tiempo inicial para primer punto 
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

[y_estimada, t_estimada] = step(StepAmplitude * w_tl, t(end));% para poder graficarla

% Verificacion gráfica
figure;
plot(t, w_inv, 'b', 'LineWidth', 1.5); hold on;           % Datos reales en azul
plot(t_estimada, y_estimada, 'r--', 'LineWidth', 1.5); % Modelo estimado rojo punteado
hold off;

title('Comparación: Respuesta Real vs Estimada');
xlabel('Tiempo (s)');
ylabel('w (r/seg)');
legend('Real', 'Estimada', 'Location', 'best');
grid on;

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