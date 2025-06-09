clear all; clc; close all;

% Definición del sistema: G(s) = (2s - 1)/(s^2 + 3s + 6)
P = 1;
num = [1 + P, -P];
den = [1, 3, 6];

% Modelo en espacio de estados (forma canónica controlable)
A = [0 1; -den(3) -den(2)];
B = [0; 1];
C = [num(2), num(1)];
D = 0;

% Ampliación del sistema con integrador del error
Aa = [A, zeros(2,1); -C, 0];
Ba = [B; 0];
Mat_Aa = Aa; Mat_Ba = Ba;

% Cálculo del controlador LQR
Q = diag([100 50 10]); R = 1e-1;

% Hamiltoniano para solución del Riccati
Ha = [Aa, -Ba * inv(R) * Ba'; -Q, -Aa'];

%Extraemos autovalores estables:Se obtienen los autovalores de Ha, y se seleccionan los vectores propios correspondientes a autovalores con parte real negativa (i.e., estables), que son necesarios para garantizar estabilidad.
[V,D] = eig(Ha); MX1X2 = [];

for ii = 1:size(Ha,1)
    if real(D(ii,ii)) < 0
        MX1X2 = [MX1X2 V(:,ii)];
    end
end

%División del vector propio en dos bloques:componentes asociadas al estado (MX1) y al costo (MX2)
MX1 = MX1X2(1:end/2,:); 
MX2 = MX1X2(end/2+1:end,:);


%solución de la ecuación algebraica de Riccati
P = real(MX2 * inv(MX1));

%Este Ka es la ganancia óptima del controlador. En este caso, está diseñada para el sistema ampliado, así que Ka tiene dos partes: K: para los estados originales,Ki: para el estado integrador del error (acción integral).
Ka = inv(R) * Ba' * P;  % Ganancias ampliadas [K Ki]

disp('Controlador ampliado en ')
eig(Mat_Aa - Mat_Ba * Ka)

% Cálculo del observador de estados
A_d = A'; B_d = C'; C_d = B'; %dual
Qo = 1e5 * diag([1 1]); %solo para los estados, no para el error
Ro = 1e1;

Ho = [A_d, -B_d * inv(Ro) * B_d'; -Qo, -A_d']; %hamiltoniano
[V,D] = eig(Ho); MX1X2 = [];

for ii = 1:size(Ho,1)
    if real(D(ii,ii)) < 0
        MX1X2 = [MX1X2 V(:,ii)];
    end
end

MX1 = MX1X2(1:end/2,:); MX2 = MX1X2(end/2+1:end,:);
Po = real(MX2 * inv(MX1));
Ko = (inv(Ro) * B_d' * Po)';  % Ganancia del observador

disp('Observador en ')
eig(A - Ko * C)

% Simulación
h = 1e-3; T = 15;
Max_T = T/h;

y(1) = 0; yp(1) = 0;
ref = 1; psi = 0; u = 0;
x_hat = [0;0];

for ii = 2:Max_T
    ua = u;
    u = -Ka * [x_hat; psi];
    psi_p = ref - y(ii-1);
    psi = psi + psi_p * h;
    
    up = (u - ua)/h;
    ypp = 2*up - 1*u - 3*yp(ii-1) - 6*y(ii-1);
    yp(ii) = yp(ii-1) + h * ypp;
    y(ii) = y(ii-1) + h * yp(ii);
    
    ys(ii-1) = y(ii);
    t(ii-1) = ii * h;
    
    x_hat_p = A * x_hat + B * u + Ko * (y(ii-1) - C * x_hat);
    x_hat = x_hat + x_hat_p * h;
    
    y_o(ii-1) = C * x_hat;
    acc(ii-1) = u;
end

% Gráficas
subplot(3,1,1); plot(t, ys, 'k'); title('Salida')
subplot(3,1,2); plot(t, ys - y_o, 'k'); title('Error de observación')
subplot(3,1,3); plot(t, acc, 'k'); title('Acción de control'); xlabel('Tiempo [seg.]')

