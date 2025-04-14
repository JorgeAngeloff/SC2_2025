% Parámetros físicos del motor
Laa = 366e-6;
J = 5e-9;
Ra = 55.6;
B = 0;
Ki = 6.49e-3;
Km = 6.53e-3;

% Función de transferencia de la planta (motor)
num = [Ki];
den = [Laa*J, Ra*J + Laa*B, Ra*B + Ki*Km];
P = tf(num, den); % Planta

% Parámetros del PID
% Kp = 5; Ki = 0; Kd = 0;
Kp=6;Ki=920;Kd=0;

% Función de transferencia del PID
C = Kp + Ki*tf(1, [1 0]) + Kd*tf([1 0], 1); % Kp + Ki/s + Kd*s

% Sistema en lazo cerrado: G(s) = C(s)*P(s) / (1 + C(s)*P(s))
sys_cl = feedback(C*P, 1);

% Respuesta al escalón (de referencia a 2 rad/s)
step(2*sys_cl, 0.001);
title('Respuesta del sistema en lazo cerrado con PID');
xlabel('Tiempo [s]');
ylabel('Velocidad angular \omega [rad/s]');
