%Deja de seguir a la referencia en 243.016mseg con TL=2.10695e-5N,Ia=213.488mA 
%El motor gira a w=0 rad/seg en 249.70m4seg con TL=2.16493e-5, Ia=215.781mA
clc; clear; close all;
X = -[0; 0; 0];
ii = 0;
t_etapa = 1e-7;


wRef = 20;
tF = 0.25;%0.0003; %tF = 0.001;
TL=0;
delta_TL = 8.67e-12; 
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

    X = modmotor(t_etapa, X, u, TL);

    e(k) = wRef - X(1); % Error

    u = u + A1*e(k) + B1*e(k-1) + C1*e(k-2); % PID
   
%Saturacion de la accion de control:
u = max(min(u, 12), -12); 

x1(ii)=X(1);%Omega
x2(ii)=X(2);%wp
x3(ii)=X(3);%ia
acc(ii)=u;
TL_vec(ii) = TL;
TL = TL + delta_TL;


end
t = 0:t_etapa:tF;

% figure;
% yyaxis left
% plot(t, x1, 'b'); ylabel('\omega [rad/s]');
% title('Velocidad angular y Torque de carga');
% xlabel('Tiempo [s]');
% 
% yyaxis right
% plot(t, TL_vec, 'r--'); ylabel('T_L [Nm]');
% legend('\omega(t)', 'T_L(t)');


%Grafico de simulacion
t=0:t_etapa:tF;
subplot(3,1,1);hold on;
plot(t,x1,color_);title('Salida y, \omega_t');
subplot(3,1,2);hold on;
plot(t,acc,color_);title('Entrada u_t, v_a');
xlabel('Tiempo [Seg.]');
subplot(3,1,3);hold on;

% %Grafico de simulacion
t=0:t_etapa:tF;
subplot(4,1,1);hold on;
plot(t,x1,color_);title('Velocidad angular, \omega_t');
subplot(4,1,2);hold on;
plot(t,TL_vec,color_);title('Entrada Torque, TL');
xlabel('Tiempo [Seg.]');
subplot(4,1,3);hold on;
plot(t,acc,color_);title('Entrada u_t, v_a');
xlabel('Tiempo [Seg.]');
subplot(4,1,4);hold on;
plot(t,x3,color_);title('Corriente de armadura , i_a');
xlabel('Tiempo [Seg.]');


