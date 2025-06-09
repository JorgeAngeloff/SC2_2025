clear all; clc; close all;

%% DEFINICIÓN DEL SISTEMA Y TRANSFORMACIÓN A ESPACIO DE ESTADOS

% Definimos una función de transferencia G(s) con parámetro P
P = 1;
num = [1+P -P];    % Numerador de la FT
den = [1 3 6];     % Denominador de la FT

% Convertimos a espacio de estados usando forma canónica controlable
A = [  0     1;
     -den(3) -den(2)];
B = [0; 1];
C = [num(2), num(1)];  % C sale de la coincidencia numérica de FT y forma canónica
D = 0;

% Verificamos que al volver a transformar a FT nos dé lo mismo
[n, d] = ss2tf(A, B, C, D);  % (solo para verificación)

%% AMPLIACIÓN DEL SISTEMA PARA CONTROL CON INTEGRADOR

% Se agrega un integrador del error → sistema aumentado
Aa = [A       [0;0];
     -C        0];   % Matriz A aumentada
Ba = [B; 0];          % Matriz B aumentada

%% CONTROLADOR POR ASIGNACIÓN DE POLOS

% Verificamos la controlabilidad del sistema aumentado
Mat_Aa = Aa;
Mat_Ba = Ba;
Mat_Ma = [Mat_Ba Mat_Aa*Mat_Ba Mat_Aa^2*Mat_Ba];  % Matriz de controlabilidad aumentada
disp("Rango de la matriz de controlabilidad (debe ser 3):")
rank(Mat_Ma)  % Si es 3 → controlable

% Calculamos autovalores actuales del sistema aumentado (sin controlar)
auto_val = eig(Mat_Aa);
% Polinomio característico actual del sistema aumentado (sin controlar)
c_ai = poly(auto_val);  % = [1 a2 a1 a0]

% Construcción de la matriz de transformación Wₐ para pasar a forma canónica controlable
% Esto se hace para poder aplicar la fórmula directa para Ka
Mat_Wa = [c_ai(3) c_ai(2) 1;
          c_ai(2) 1        0;
          1       0        0];

% Matriz de transformación Tₐ = Mₐ * Wₐ
Mat_Ta = Mat_Ma * Mat_Wa;

% Verificación (opcional): cambio de base a forma controlable
A_controlable = inv(Mat_Ta) * Mat_Aa * Mat_Ta;

% Elegimos los polos deseados del sistema controlado
Polos_deseados = [-0.3; -5; -1];  % Tiempo de respuesta lento/moderado

% Calculamos el polinomio deseado (producto de (s - μi))
alfa_ia = poly(Polos_deseados);  % = [1 α2 α1 α0]

% Cálculo del vector de ganancia Ka usando fórmula directa:
% Ka = (αd - αa) * T⁻¹
Ka = fliplr(alfa_ia(2:4) - c_ai(2:4)) * inv(Mat_Ta);  % fliplr acomoda términos en orden correcto
K=Ka;
% Resultado del controlador Ka
disp('Autovalores del sistema controlado (Aa - Ba*Ka):')
eig(Mat_Aa - Mat_Ba * Ka)

%% OBSERVADOR DE ESTADOS (ESTIMADOR)

% Trabajamos con el sistema original (no aumentado)
% Aplicamos el principio de dualidad para construir el observador

A_d = A';      % Transpuesta de A
B_d = C';      % Transpuesta de C
C_d = B';      % Transpuesta de B

% Elegimos polos para el observador → más rápidos que el sistema
Polos_Obs = [-10; -9];
alfa_ia_d = poly(Polos_Obs);  % Polinomio deseado del observador

% Calculamos matriz de controlabilidad dual
Mat_M_d = [B_d A_d*B_d];  % Matriz de controlabilidad del sistema dual

% Polinomio característico actual del sistema sin controlar
auto_val_d = eig(A);
c_ai_d = poly(auto_val_d);  % = [1 a1 a0]

% Matriz de transformación W (dual) para cambio de base en el observador
Mat_W = [c_ai_d(2) 1;
         1        0];

% Matriz de transformación T para observador
Mat_T_d = Mat_M_d * Mat_W;

% Cálculo de la ganancia del observador Ko (usamos dualidad)
Ko = (fliplr(alfa_ia_d(2:3) - c_ai_d(2:3)) * inv(Mat_T_d))';
disp('Autovalores del observador (A - Ko*C):')
eig(A - Ko * C)

%% SIMULACIÓN EN LAZO CERRADO

% Parámetros de simulación
h = 4e-3;           % Paso de integración
TFinal = 16.9730;   % Tiempo final simulado
Max_T = round(TFinal / h);  % Número de pasos de integración

% Inicialización de variables
e = 0; ys = []; ypp(1) = 0; yp(1) = 0; y(1) = 0; t = [];
x = [yp(1); y(1)];
ref = 1; psi = 0; u = 0;  % psi = integrador del error
x_hat = [0; 0];           % Estado estimado por el observador
y_o(1) = 0; acc(1) = 0;   % Inicialización de variables

% Bucle de simulación
for ii = 2:Max_T
    ua = u;
    
    % Controlador en lazo cerrado con integrador y estado estimado
    u = -K * [x_hat; psi];
    
    % Integración del error de seguimiento
    psi_p = ref - y(ii-1);  % Error instantáneo
    psi = psi + psi_p * h;  % Integrador del error
    
    % Dinámica del sistema real (2 s - 1) / (s^2 + 3s + 6)
    up = (u - ua) / h;
    ypp = 2 * up - 1 * u - 3 * yp(ii-1) - 6 * y(ii-1);
    yp(ii) = yp(ii-1) + h * ypp;
    y(ii) = y(ii-1) + h * yp(ii);
    
    % Salida estimada y actualización del observador
    x_hat_p = A * x_hat + B * u + Ko * (y(ii-1) - C * x_hat);
    x_hat = x_hat + x_hat_p * h;
    
    % Guardamos resultados
    ys(ii-1) = y(ii);
    t(ii-1) = ii * h;
    y_o(ii-1) = C * x_hat;
    acc(ii-1) = u;
end

%% PLOTEO DE RESULTADOS

subplot(3,1,1); plot(t, ys, 'k'); title('Salida');
subplot(3,1,2); plot(t, ys - y_o, 'k'); title('Error de observación');
subplot(3,1,3); plot(t, acc, 'k'); title('Acción de control'); xlabel('Tiempo [seg.]');
