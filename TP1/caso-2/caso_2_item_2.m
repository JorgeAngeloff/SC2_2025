clear; clc; close; 

data = xlsread('Curvas_Medidas_Motor_2025.xls', 'Hoja1');
t = data(:, 1);    % Tiempo (desde fila 1001 hasta 5000) que corresponde al primer escalon sin retardo
w = data(:, 2);    % Corriente
ia = data(:, 3);   % Tensión en el capacitor
v = data(:, 4);   % Tensión de entrada
TL = data(:, 5);   % Tensión de entrada


%Grafico 
subplot(4,1,1); plot(t,w,'r'); title('w,t');  
subplot(4,1,2); plot(t,ia,'b'); title('ia,t'); 
subplot(4,1,3); plot(t,v,'g');  title('v,t');
subplot(4,1,4); plot(t,TL,'g');  title('TL,t');