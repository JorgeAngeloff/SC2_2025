clear; clc; close all;

% Definición de Parámetros del Sistema
td = 0.2; % Retardo en segundos
h = 0.0001; % Paso de integración
t = 0:h:10; % Vector de tiempo de simulación
N = round(td / h); % Tamaño del buffer circular

% Inicialización de variables
R_U = zeros(1, N); % Buffer circular con valores nulos
salida_buffer = zeros(size(t)); % Salida del sistema
error = zeros(size(t)); % Error
u = zeros(size(t)); % Acción de control
ref = ones(size(t)); % Entrada escalón unidad

% Definición del Controlador PI
Kp = 1; % Ganancia proporcional
Ti = 1; % Constante de integración

% Implementación del Controlador PI con Buffer Circular para Retardo
for i = 2:length(t)
    error(i) = ref(i) - salida_buffer(i-1); % Cálculo del error
    U_ = Kp * (error(i) + (1/Ti) * sum(error(1:i)) * h); % Controlador PI discreto
    
    %  Aplicación del Retardo con Buffer Circular
    R_U = circshift(R_U, [0, -1]); % Desplaza la memoria circular
    R_U(end) = U_; % Guarda la acción de control actual
    u(i) = R_U(1); % Usa el valor retardado
    
    %  Simulación de la respuesta con integración numérica (Euler)
    salida_buffer(i) = salida_buffer(i-1) + h * (u(i) - salida_buffer(i-1));
end

% Gráfico de la Respuesta del Sistema con Retardo
figure;
plot(t, salida_buffer, 'b', 'LineWidth', 2);
title('Respuesta del Sistema con Controlador PI y Retardo Implementado');
xlabel('Tiempo (s)');
ylabel('Salida del Sistema');
grid on;
legend('Salida con Buffer Circular');
