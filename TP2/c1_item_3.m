%% c
close all;clear; clc;

% ==== PARÁMETROS DEL SISTEMA ====
Ra = 2.258930051299405;
Laa = 0.005026901184834716;
Ki_ = 0.25965987053759737;
Jm = 0.0028472626983113334;
Bm = 0.0014165170369840668;
Km = 0.2500481104997174;

A = [-Ra/Laa, -Km/Laa, 0;
      Ki_/Jm, -Bm/Jm, 0;
      0,       1,     0];
B = [1/Laa; 0; 0];
C = [0, 0, 1];

% ==== TIEMPO ====
Tf = 20;
Ts = 0.001;
t = 0:Ts:Tf;

% ==== REFERENCIA ====
ref = zeros(size(t));
for k = 1:length(t)
    ref(k) = (-1)^(floor(t(k)/5)) * pi/2;
end

% ==== CONTROLADOR LQI ====
Qi = 1000;
Qx = diag([0.01, 0.1, 5]);
Q = blkdiag(Qx, Qi);
R = 1;
Aa = [A, zeros(3,1); -C, 0];
Ba = [B; 0];
Ka = lqr(Aa, Ba, Q, R);
K = Ka(1:3);
Ki = Ka(4);

% ==== ZONA MUERTA ====
ZM = 0.1;  % Probar con 0.1, 0.3, 0.5, 0.7, 1.0

% ==== ESTADOS ====
X = zeros(3,1);
ei = 0;
Y = zeros(1, length(t));
U = zeros(1, length(t));
Uaplicada = zeros(1, length(t));

% ==== SIMULACIÓN ====
for k = 1:length(t)
    % Control sin observador
    e = ref(k) - X(3);
    ei = ei + e * Ts;
    u = -K*X - Ki*ei;
    u = max(min(u, 5), -5);

    % Zona muerta
    if abs(u) > ZM
        u_aplicada = u - ZM * sign(u);
    else
        u_aplicada = 0;
    end

   %torque
   if t(k) >= 0.7
    TL = 0.12;  % Carga aplicada desde 0.7s
else
    TL = 0;
   end
   
    X = modmotor(Ts, X, [u_aplicada; TL]);

    % Guardado
    Y(k) = X(3);
    U(k) = u;
    Uaplicada(k) = u_aplicada;
end

% ==== GRÁFICAS ====
figure;
subplot(3,1,1);
plot(t, ref, 'k--', t, Y, 'b');
xlabel('Tiempo (s)'); ylabel('\theta (rad)');
legend('Referencia','Salida'); grid on;

subplot(3,1,2);
plot(t, U, 'r');
xlabel('Tiempo (s)'); ylabel('u(t) [V]');
title('Control (antes de zona muerta)'); grid on;

subplot(3,1,3);
plot(t, Uaplicada, 'm');
xlabel('Tiempo (s)'); ylabel('u aplicada [V]');
title('Control después de zona muerta'); grid on;
