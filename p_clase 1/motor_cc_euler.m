clear; close all;
X = -[0; 0];
ii = 0;
t_etapa = 1e-7;
wRef = 2;
tF = 0.0003; %tF = 0.001;

% Constantes del PID

% Kp=.500;Ki=0.001;Kd=0.0001;color_='r';
% Kp=1;Ki=0;Kd=0.0001;color_='k';
% Kp=5;Ki=0;Kd=0;color_='b';
Kp = 6; Ki = 920; Kd = 0; color_ = 'b';

Ts = t_etapa;

A1 = ((2*Kp*Ts) + (Ki*(Ts^2)) + (2*Kd)) / (2*Ts);
B1 = (-2*Kp*Ts + Ki*(Ts^2) - 4*Kd) / (2*Ts);
C1 = Kd / Ts;

e = zeros(tF / t_etapa, 1); u = 0;

for t = 0:t_etapa:tF
    ii = ii + 1; k = ii + 2;

    X = modmotor(t_etapa, X, u);

    e(k) = wRef - X(1); % Error

    u = u + A1*e(k) + B1*e(k-1) + C1*e(k-2); % PID

   
%Saturacion de la accion de control:

%1)u_t pasa a ser un valor teorico sin limites,
% guarda el valor y luego lo compara para saturar la
% entrada si hace falta

% if(u_t>12)
%     u=12;
% elseif(u_t<-12)
%     u=-12;
% else
%     u=u_t;
% end

%2) usando las funciones max y min
% u = max(min(u, 12), -12); 

x1(ii)=X(1);%Omega
x2(ii)=X(2);%wp
acc(ii)=u;

end

%Grafico de simulacion
% t=0:t_etapa:tF;
% subplot(2,1,1);hold on;
% plot(t,x1,color_);title('Salida y, \omega_t');
% subplot(2,1,2);hold on;
% plot(t,acc,color_);title('Entrada u_t, v_a');
% xlabel('Tiempo [Seg.]');

%Verificacion con modelo continuo
% Parámetros físicos del motor
Laa = 366e-6;
J = 5e-9;
Ra = 55.6;
B = 0;
Ki_motor = 6.49e-3;
Km = 6.53e-3;

% FT de la planta
num = [Ki_motor];
den = [Laa*J, Ra*J + Laa*B, Ra*B + Ki_motor*Km];
P = tf(num, den);

% FT del PID
C = Kp + Ki*tf(1, [1 0]) + Kd*tf([1 0], 1);

% Sistema en lazo cerrado
sys_cl = feedback(C*P, 1);

%Se preparan los graficos

% Vector de tiempo para la simulación discreta
t = 0:t_etapa:tF;

% Obtener los datos de salida del modelo continuo
[y_step, t_step] = step(2*sys_cl, t); % Importante que usemos t para 
                                      % que tenga la misma cantidad de puntos 
                                      % y sea fácil comparar las curvas

% Ubtener los datos de la accion de control del modelo continuo
[y_control, t_control] = step(2*C/(1 + C*P), t);

% Graficar ambos resultados en una sola figura
figure;

% Salida velocidad angular
subplot(2,1,1); hold on;
plot(t, x1, 'r', 'DisplayName', 'Simulación discreta');
plot(t_step, y_step, 'b--', 'DisplayName', 'Modelo continuo (PID)');
legend('show');
title('Comparación: Simulación vs Modelo continuo con PID');
xlabel('Tiempo [s]');
ylabel('Velocidad angular \omega [rad/s]');

% Acción de control
subplot(2,1,2); hold on;
plot(t, acc, 'r', 'DisplayName', 'Acción de control (u) - Simulación');
plot(t_control, y_control, 'b--', 'DisplayName', 'Acción de control - Modelo continuo');
title('Acción de control u(t)');
xlabel('Tiempo [s]');
ylabel('Voltaje [V]');
legend('show');



