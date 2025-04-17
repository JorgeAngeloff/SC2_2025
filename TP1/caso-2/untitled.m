clc; clear; close all;
syms T1 T2 T3 K1_C K2_C s La Ki Jm Km Bm  % Ra ya no se declara como simbólica

Ra = 2 / 0.822;  % Valor estimado experimentalmente

% Funciones de transferencia de Chen (normalizadas)
Wr_Va_Chen = K1_C / (T1*T2*s^2 + (T1 + T2)*s + 1);
Wr_TL_Chen = K2_C*(T3*s + 1) / (T1*T2*s^2 + (T1 + T2)*s + 1);

% Tus funciones estimadas (con el cero eliminado)
Wr_Va_est = 3.765 / (0.01533*s^2 + 0.3673*s + 1);
Wr_TL_est = 33.75 / (3.234e5*s^2 + 1932*s + 1);

% Ecuaciones estructurales del modelo físico
eq1 = K1_C/(T1*T2) == Ki/(Jm*La);
eq2 = K2_C/(T1*T2) == 1/(Ra*Jm*La);
eq3 = T3 == La/Ra;
eq4 = (Bm*La + Jm*Ra)/(Jm*La) == (T1 + T2)/(T1*T2);
eq5 = (Bm*Ra + Ki*Km)/(Jm*La) == 1/(T1*T2);

% Extraídas del denominador de la función estimada ωr/Va
eq6 = T1*T2 == 0.01533;
eq7 = T1 + T2 == 0.3673;

% Resolver el sistema (sin incluir Ra como incógnita)
vars = [La, Ki, Jm, Km, Bm, K1_C, K2_C, T1, T2, T3];
sols = solve([eq1, eq2, eq3, eq4, eq5, eq6, eq7], vars);

% Mostrar resultados
disp('Soluciones encontradas:')
disp(sols)

% Mostrar con formato bonito
pretty(sols.La)
pretty(sols.Ki)
pretty(sols.Jm)
pretty(sols.Km)
pretty(sols.Bm)

% Convertir a valores numéricos con vpa (usar la primera solución por ejemplo)
disp('Valores numéricos (1ra solución):')
disp('----------------------------------')
disp(['La  = ', char(vpa(sols.La(1), 6))])
disp(['Ki  = ', char(vpa(sols.Ki(1), 6))])
disp(['Jm  = ', char(vpa(sols.Jm(1), 6))])
disp(['Km  = ', char(vpa(sols.Km(1), 6))])
disp(['Bm  = ', char(vpa(sols.Bm(1), 6))])
disp(['Ra  = ', char(vpa(Ra, 6))])  % Este lo calculaste directo

% 
% % Parámetros identificados
% Laa = 2.43309;
% J = 0.00258956;
% Ra = 2.43309;
% B = 0.0594551;
% Ki = 0.411;
% Km = 0.64803;

