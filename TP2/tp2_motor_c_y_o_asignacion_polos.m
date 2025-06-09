clear; clc;

%% Parámetros del modelo estimado (controlador interno)
Ra = 2.258930051299405;
Laa = 0.005026901184834716;
Ki_ = 0.25965987053759737;
Jm = 0.0028472626983113334;
Bm = 0.0014165170369840668;
Km = 0.2500481104997174;

% Matrices del modelo interno
Mat_A = [-Ra/Laa, -Km/Laa,     0;
          Ki_/Jm, -Bm/Jm,      0;
          0,       1,          0];

Mat_B = [1/Laa; 0; 0];
Mat_C = [0, 0, 1];

%% Controlador por asignación de polos
t_etapa = 0.001;
Aa = [Mat_A, zeros(3,1); -Mat_C, 0];
Ba = [Mat_B; 0];

% Los polos deberan ser mayores que los determinados por el tiempo de etapa
fprintf("Polos menores a:\n")
p_min=1/(3*t_etapa)

% Polos deseados para el sistema ampliado
polos_deseados = [-300, -50, -15, -10];
Ka = place(Aa, Ba, polos_deseados);% en vez de cambiar de base a mano

fprintf("Polos en lazo cerrado del controlador:\n")
disp(eig(Aa - Ba * Ka))

%% Observador de Luenberger
Ad = Mat_A';
Bd = Mat_C';
Cd = Mat_B';

polos_obs = [-330, -200, -220];
Ko = place(Ad, Bd, polos_obs)';
fprintf("Polos del observador:\n")
disp(eig(Mat_A - Ko * Mat_C))

% Dinamica completa del sistema: polos de controlador+ polos de observador

%% Simulación
tF = 10;
N = tF / t_etapa;

X = zeros(3,1);     % Estado real
x_hat = zeros(3,1); % Estado estimado
psi = 0;            % Integrador

TL_Max = 0.12;

% Inicialización de vectores
e = zeros(1, N);
x1 = zeros(1, N); x2 = zeros(1, N); x3 = zeros(1, N);
acc = zeros(1, N); TL_D1 = zeros(1, N); titaRef_ = zeros(1, N);

for jj = 1:N
    tiempo = (jj - 1) * t_etapa;

    % Entrada torque y referencia
    TL_ap = (tiempo > 0.7) * TL_Max;
    if tiempo <= 5
        titaRef = pi/2;
    else
        titaRef = -pi/2;
        TL_ap = TL_Max;
    end

    % Salida medida real
    Y = Mat_C * X;

    % Ley de control
    e(jj) = titaRef - Y;
    U = -Ka * [x_hat; psi];
    acc(jj) = U;

    % Simulación de planta real con parámetros reales
    X = modmotor(t_etapa, X, [U; TL_ap]);

    % Observador
    x_hat_dot = Mat_A * x_hat + Mat_B * U + Ko * (Y - Mat_C * x_hat);
    x_hat = x_hat + x_hat_dot * t_etapa;

    % Integrador
    psi = psi + e(jj) * t_etapa;

    % Guardar datos
    x1(jj) = X(1);
    x2(jj) = X(2);
    x3(jj) = X(3);
    TL_D1(jj) = TL_ap;
    titaRef_(jj) = titaRef;
end

t = linspace(0, tF, N);
%%%%%%%%%%%%%

figure;

subplot(4,1,1)
plot(t, titaRef_, '--', t, x3, 'LineWidth', 1.4);
title('Control por espacio de estados con observador');
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
