% Parametros del motor
J = 0.01;  % Momento de inercia (kg.m^2)
B = 0.1;   % Coeficiente de fricción viscosa (N.m.s)
K = 0.01;  % Constante de torque y FEM (N.m/A)
R = 1;     % Resistencia (Ohm)
L = 0.5;   % Inductancia (H)

% Matrices del espacio de estados
A = [-B/J  K/J;
     -K/L -R/L];
B = [0; 1/L];
C = [1 0];
D = 0;

% Crear sistema en espacio de estados
sys = ss(A, B, C, D);

% Simulacion
T = 0:0.01:5; % Tiempo de simulacion de 0 a 5 segundos
U = ones(size(T)); % Entrada escalón (voltaje constante de 1V)
[Y, T, X] = lsim(sys, U, T);

% Graficar respuesta
tiledlayout(2,1);
nexttile;
plot(T, X(:,1));
xlabel('Tiempo (s)'); ylabel('Velocidad (rad/s)');
title('Respuesta del Motor DC - Velocidad Angular'); grid on;

nexttile;
plot(T, X(:,2));
xlabel('Tiempo (s)'); ylabel('Corriente (A)');
title('Respuesta del Motor DC - Corriente'); grid on;
