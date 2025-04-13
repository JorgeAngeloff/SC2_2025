%% Ítem 3 - Validación con corriente simulada
clear all;clc;

C=2.177e-6; %valores obtenidos en Item-2
L=29.4e-3;
R=282.3;

data = xlsread('Curvas_Medidas_RLC_2025.xls', 'Hoja1');
t_v = data(5000:10000, 1);    % Tiempo de verificacion a partir del primer escalon negativo
i_v = data(5000:10000, 2);    % Corriente de verificacion
vc_v = data(5000:10000, 3);   % Tensión en el capacitor de verificacion
vi_v = data(5000:10000, 4);   % Tensión de entrada de verificacion

t_v = t_v - t_v(1);  % tiempo normalizado para que el pico de corriente empiece en t_v=0

% subplot(3,1,1); plot(t_v,i_v,'r'); title('I_{verificacion},t');  
% subplot(3,1,2); plot(t_v,vi_v,'b'); title('Vi_{verificacion},t'); 
% subplot(3,1,3); plot(t_v,vc_v,'g'); title('Vc_{verificacion},t'); 

dt_v = t_v(2) - t_v(1);  % paso de tiempo

dvc_dt_v = [NaN; diff(vc_v) / dt_v];  % se aproxima la derivada con diferencia para atras

i_est=C*dvc_dt_v;

figure;
plot(t_v, i_v, 'r', 'LineWidth', 1.5); hold on;
plot(t_v, i_est, 'g--', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Corriente (A)');
title('Comparación: Corriente Real vs Estimada');
legend('Corriente Real (medida)', 'Corriente Estimada (C·dVc/dt)', 'Location', 'best');
grid on;
