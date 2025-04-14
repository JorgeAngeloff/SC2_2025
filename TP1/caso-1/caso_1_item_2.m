clear; clc; close; 

data = xlsread('Curvas_Medidas_RLC_2025.xls', 'Hoja1');
t = data(1001:5000, 1);    % Tiempo (desde fila 1001 hasta 5000) que corresponde al primer escalon sin retardo
i = data(1001:5000, 2);    % Corriente
vc = data(1001:5000, 3);   % Tensión en el capacitor
vi = data(1001:5000, 4);   % Tensión de entrada

StepAmplitude = 12;  % Amplitud del escalón (12V)

% normalizo el t para que el escalón comience en t=0
t = t - t(1);  % Ahora t(1) = 0

% Selección de puntos para el método 
t_inic = 0.002;  % Tiempo inicial para primer punto 
[val lugar] = min(abs(t_inic - t)); 
y_t1 = vc(lugar);
t_t1 = t(lugar);

[val lugar] = min(abs(2*t_inic - t));
y_2t1 = vc(lugar);

[val lugar] = min(abs(3*t_inic - t));
y_3t1 = vc(lugar);

% Cálculo de parámetros intermedios 
K = vc(end) / StepAmplitude;  % Ganancia estática
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
sys_G_ang = tf(K * [T3_ang 1], conv([T1_ang 1], [T2_ang 1]))

[y_estimada, t_estimada] = step(StepAmplitude * sys_G_ang, t(end));% para poder graficarla

% Verificacion gráfica
figure;
plot(t, vc, 'b', 'LineWidth', 1.5); hold on;           % Datos reales en azul
plot(t_estimada, y_estimada, 'r--', 'LineWidth', 1.5); % Modelo estimado rojo punteado
hold off;

title('Comparación: Respuesta Real vs Estimada');
xlabel('Tiempo (s)');
ylabel('Vc (V)');
legend('Real', 'Estimada', 'Location', 'best');
grid on;

%% Calcular derivada hacia atrás de vc
dt = t(2) - t(1);  % paso de tiempo

dvc_dt = [NaN; diff(vc) / dt];  % Vector columna, igual a vc

% Calcular C en cada instante
C_estimada = i ./ dvc_dt;

% Opcional: podés descartar los valores donde haya NaN o inf
C_estimada(1) = NaN;

C_estimada;

%% Función de transferencia teórica con los valores estimados de R, L y C
C=2.177e-6;
L=29.4e-3;
R=282.3;

num_teo = 1; %se desprecia el cero
den_teo = [L*C R*C 1];
sys_teo = tf(num_teo, den_teo);

[y_teo, t_teo] = step(StepAmplitude * sys_teo, t(end));%para poder graficarla

% Graficar todas las curvas juntas
figure;
plot(t, vc, 'b', 'LineWidth', 1.5); hold on;                   % Curva real
plot(t_estimada, y_estimada, 'r--', 'LineWidth', 1.5);         % Curva estimada
plot(t_teo, y_teo, 'g-.', 'LineWidth', 1.5);                   % Curva teórica

title('Comparación: Real vs Estimada vs Teórica');
xlabel('Tiempo (s)');
ylabel('Vc (V)');
legend('Real', 'Estimada', 'Teórica', 'Location', 'best');
grid on;


%% Función de transferencia teórica CON el cero (sin despreciar)
num_teo_cero = [0.0001313 1]; %como nos dio sys_G_ang
den_teo = [L*C R*C 1];
sys_teo_cero = tf(num_teo_cero, den_teo);

[y_teo_cero, t_teo_cero] = step(StepAmplitude * sys_teo_cero, t(end));%para graficarla

% Graficar todas las curvas juntas
figure;
plot(t, vc, 'b', 'LineWidth', 1.5); hold on;                    % Curva real
plot(t_estimada, y_estimada, 'r--', 'LineWidth', 1.5);          % Curva estimada
plot(t_teo, y_teo, 'g-.', 'LineWidth', 1.5);                     % Teórica sin cero
plot(t_teo_cero, y_teo_cero, 'm:', 'LineWidth', 1.5);           % Teórica con cero

title('Comparación: Real vs Estimada vs Teórica (con y sin cero)');
xlabel('Tiempo (s)');
ylabel('Vc (V)');
legend('Real', 'Estimada', 'Teórica sin cero', 'Teórica con cero', 'Location', 'best');
grid on;

%%
clear; clc; close; 

data = xlsread('Curvas_Medidas_RLC_2025.xls', 'Hoja1');
t = data(:, 1);    % Tiempo (desde fila 1001 hasta 5000) que corresponde al primer escalon sin retardo
i = data(:, 2);    % Corriente
vc = data(:, 3);   % Tensión en el capacitor
vi = data(:, 4);   % Tensión de entrada

subplot(3,1,1); plot(t,i,'r'); title('I1,t');  
subplot(3,1,2); plot(t,vc,'b'); title('Vc,t'); 
subplot(3,1,3); plot(t,vi,'g');  title('U1,t');


