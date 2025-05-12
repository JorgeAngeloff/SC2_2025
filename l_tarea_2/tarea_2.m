 clear; clc; close all;
% Obtener la función de transferencia continua G(s)
G=zpk([],[0 -2],[5])
Tm=(0.15)

% Hallar la FT discreta de lazo abierto G_D(s) del sistema 
% de la figura con Z0H a la entrada y el tiempo de muestreo asignado Tm
Gd=c2d(G,Tm,'zoh')

% rlocus(Gd);
  % sisotool(Gd)
%  C
%   F=feedback(C*Gd,1) % sistema de lazo cerrado
% % 
%  pole(F)
%  zero(F)
%    % pzmap(F)
%  step(F,5) % respuesta al escalo

%%
figure;

% Paleta de colores personalizada
col_salida      = [0 0.4470 0.7410];  % azul
col_controlador = [0.4660 0.6740 0.1880]; % verde
col_error       = [0.8500 0.3250 0.0980];  % rojo
col_derivativa  = [0.4940 0.1840 0.5560];  % violeta
col_integral    = [0.9290 0.6940 0.1250];  % mostaza
col_proporcional= [0.3010 0.7450 0.9330];  % celeste

subplot(3,2,1);
plot(tout, yout(:,1), 'Color', col_salida, 'LineWidth', 1.5);
title('1. Salida del sistema');
xlabel('Tiempo [s]'); ylabel('Salida');
grid on;

subplot(3,2,2);
plot(tout, yout(:,2), 'Color', col_controlador, 'LineWidth', 1.5);
title('2. Salida del controlador');
xlabel('Tiempo [s]'); ylabel('u(t)');
grid on;

subplot(3,2,3);
plot(tout, yout(:,3), 'Color', col_error, 'LineWidth', 1.5);
title('3. Error');
xlabel('Tiempo [s]'); ylabel('e(t)');
grid on;

subplot(3,2,4);
plot(tout, yout(:,4), 'Color', col_derivativa, 'LineWidth', 1.5);
title('4. Acción derivativa');
xlabel('Tiempo [s]'); ylabel('D(t)');
grid on;

subplot(3,2,5);
plot(tout, yout(:,5), 'Color', col_integral, 'LineWidth', 1.5);
title('5. Acción integral');
xlabel('Tiempo [s]'); ylabel('I(t)');
grid on;

subplot(3,2,6);
plot(tout, yout(:,6), 'Color', col_proporcional, 'LineWidth', 1.5);
title('6. Acción proporcional');
xlabel('Tiempo [s]'); ylabel('P(t)');
grid on;

sgtitle('Análisis completo del PID discreto', 'FontWeight', 'bold');
%%
figure;

% Colores diferenciados
col_salida      = [0 0.4470 0.7410];     % azul
col_controlador = [0.4660 0.6740 0.1880];% verde
col_error       = [0.8500 0.3250 0.0980];% rojo
col_adelanto    = [0.4940 0.1840 0.5560];% violeta

subplot(4,1,1);
plot(tout, yout(:,1), 'Color', col_salida, 'LineWidth', 1.5);
title('1. Salida del sistema');
xlabel('Tiempo [s]'); ylabel('Salida');
grid on;

subplot(4,1,2);
plot(tout, yout(:,2), 'Color', col_controlador, 'LineWidth', 1.5);
title('2. Salida del controlador');
xlabel('Tiempo [s]'); ylabel('u(t)');
grid on;

subplot(4,1,3);
plot(tout, yout(:,3), 'Color', col_error, 'LineWidth', 1.5);
title('3. Error');
xlabel('Tiempo [s]'); ylabel('e(t)');
grid on;

subplot(4,1,4);
plot(tout, yout(:,4), 'Color', col_adelanto, 'LineWidth', 1.5);
title('4. Acción de adelanto');
xlabel('Tiempo [s]'); ylabel('Adelanto');
grid on;

sgtitle('Análisis del Controlador de Adelanto Digital', 'FontWeight', 'bold');
