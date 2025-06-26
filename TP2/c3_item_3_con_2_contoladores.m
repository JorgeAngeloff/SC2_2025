clc; clear; close all;

%% Constantes comunes
M = 1.5;           % masa del carro (kg)
long = 1.6;        % longitud del péndulo (m)
g = 9.8;           % gravedad (m/s^2)
Fricc = 0.1;       % fricción viscosa

%% Función para obtener matrices del sistema, observador y controlador
obtener_matrices = @(m) deal(...
    [0 1 0 0;
     0 -Fricc/M -m*g/M 0;
     0 0 0 1;
     0 -Fricc/(long*M) -g*(m+M)/(long*M) 0], ... % A
    [0; 1/M; 0; 1/(long*M)], ...                 % B
    [1 0 0 0;
     0 0 1 0]);                                 % C

%% Diseño para m1
m1 = 0.1;
[A1, B1, C1] = obtener_matrices(m1);
%expandimos para agregar integrador
Aa1 = [A1 zeros(4,1); -C1(1,:) 0]; Ba1 = [B1; 0];
%lqr
Q1 = diag([8e2, 1e1, 1e4, 1e3, 500]); R1 = 1e2;
H1 = [Aa1 -Ba1*(R1\Ba1'); -Q1 -Aa1'];
[V1,D1] = eig(H1);
MX1_1 = V1(1:5, real(diag(D1)) < 0); MX2_1 = V1(6:end, real(diag(D1)) < 0);
P1 = real(MX2_1 / MX1_1);
Ka1 = (R1 \ Ba1') * P1; %esta ganancia incluye K y Ki

%observador de luenberger con lqr
A1_o = A1'; B1_o = C1'; Q1_o = diag([10, 1e2, 1, 1e3]); R1_o = diag([1e-2, 1e-1]);
H1_o = [A1_o -B1_o*(R1_o\B1_o'); -Q1_o -A1_o'];
[V1o,D1o] = eig(H1_o);
MX1o_1 = V1o(1:4, real(diag(D1o)) < 0); MX2o_1 = V1o(5:end, real(diag(D1o)) < 0);
P1_o = real(MX2o_1 / MX1o_1);
Ko1 = P1_o * (C1' / R1_o); %ganancia ko del observador  

%% Diseño para m2
m2 = 1.0;
[A2, B2, C2] = obtener_matrices(m2);
%expandimos para agregar integrador
Aa2 = [A2 zeros(4,1); -C2(1,:) 0]; Ba2 = [B2; 0];
%lqr
Q2 = diag([5e3, 1e1, 1e5, 1e3, 500]); R2 = 1e3;
H2 = [Aa2 -Ba2*(R2\Ba2'); -Q2 -Aa2'];
[V2,D2] = eig(H2);
MX1_2 = V2(1:5, real(diag(D2)) < 0); MX2_2 = V2(6:end, real(diag(D2)) < 0);
P2 = real(MX2_2 / MX1_2);
Ka2 = (R2 \ Ba2') * P2;%esta ganancia incluye K y Ki

A2_o = A2'; B2_o = C2'; Q2_o = Q1_o; R2_o = R1_o;
H2_o = [A2_o -B2_o*(R2_o\B2_o'); -Q2_o -A2_o'];
[V2o,D2o] = eig(H2_o);
MX1o_2 = V2o(1:4, real(diag(D2o)) < 0); MX2o_2 = V2o(5:end, real(diag(D2o)) < 0);
P2_o = real(MX2o_2 / MX1o_2);
Ko2 = P2_o * (C2' / R2_o);%ganancia ko del observador 

%% Simulación
h = 1e-3; t_simul = 50; N = t_simul/h;
ref = [10*ones(1,N/2), zeros(1,N/2)];
x_op = [0; 0; pi; 0];
u_op = 0;

x_hat = x_op;


delta = zeros(1,N); delta_p = zeros(1,N);
phi = zeros(1,N); omega = zeros(1,N);
x_hat = zeros(4,1); u = zeros(1,N); psi = zeros(1,N);
delta_O = zeros(1,N); phi_O = zeros(1,N);
phi(1) = pi; estado = [delta(1); delta_p(1); phi(1); omega(1)];

controlador = Ka1; observador = Ko1; A = A1; B = B1; C = C1; m = m1;

for i = 1:N-1
    if delta(i) >= 9.99 && m ~= m2
        m = m2;
        fprintf('>> Cambio de masa a m = %.2f kg en t = %.2f s (x = %.2f m)\n', m, i*h, delta(i));
        controlador = Ka2; observador = Ko2; A = A2; B = B2; C = C2;
    end

    Y = C * estado;
    psi_p = ref(i) - Y(1);
    psi(i+1) = psi(i) + psi_p*h;
    u(i) = -controlador * [x_hat; psi(i+1)];

    den = long * (4/3 - m*cos(phi(i))^2/(m+M));
    num = g*sin(phi(i)) - cos(phi(i))*(u(i) + m*long*omega(i)^2*sin(phi(i)) - Fricc*delta_p(i))/(M+m);
    phi_pp = num / den;
    delta_pp = (1/(M+m))*(u(i) - m*long*phi_pp*cos(phi(i)) + m*long*omega(i)^2*sin(phi(i)) - Fricc*delta_p(i));

    Y_o = C * x_hat;
x_hat_p = A*(x_hat - x_op) + B*(u(i) - u_op) + observador*(Y - Y_o);
    x_hat = x_hat + h*x_hat_p;

    delta_p(i+1) = delta_p(i) + h*delta_pp;
    delta(i+1) = delta(i) + h*delta_p(i);
    omega(i+1) = omega(i) + h*phi_pp;
    phi(i+1) = phi(i) + h*omega(i);
    estado = [delta(i+1); delta_p(i+1); phi(i+1); omega(i+1)];

    delta_O(i+1) = x_hat(1);
    phi_O(i+1) = x_hat(3);
end
t = (0:N-1)*h;
u(N) = -controlador * [x_hat; psi(end)];

%% Gráficas
figure('Name','Estados del Sistema');
subplot(2,2,1); plot(t, delta); ylabel('\delta (m)'); grid on;
subplot(2,2,2); plot(t, phi); ylabel('\phi (rad)'); grid on;
subplot(2,2,3); plot(t, delta_p); ylabel('\deltȧ (m/s)'); xlabel('Tiempo (s)'); grid on;
subplot(2,2,4); plot(t, omega); ylabel('\phi̇ (rad/s)'); xlabel('Tiempo (s)'); grid on;
sgtitle('Evolución de los Estados');

figure('Name','Acción de Control');
plot(t, u); xlabel('Tiempo (s)'); ylabel('u(t) (N)'); grid on;
