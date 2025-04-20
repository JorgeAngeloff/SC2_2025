clc; clear; close all;
syms T1 T2 T3 K1_C K2_C s La Ki Jm Km Bm  % Ra ya no se declara como simbólica

Ra = 2 / 0.822;  % Valor estimado experimentalmente

% Funciones de transferencia de Chen (normalizadas)
Wr_Va_Chen = K1_C / (T1*T2*s^2 + (T1 + T2)*s + 1);
Wr_TL_Chen = K2_C*(T3*s + 1) / (T1*T2*s^2 + (T1 + T2)*s + 1);

% Tus funciones estimadas (con el cero eliminado)
Wr_Va_est = 3.812 / (0.002394*s^2 + 0.1176*s + 1);
Wr_TL_est = -34.57 / (2.856e04*s^2 + 335.9*s + 1);

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

%%
clc; clear; close all;
X = -[0; 0; 0];
ii = 0;
t_etapa = 1e-6;

tF = 1.5;
TL=0;

Ts = t_etapa;

u=0;

for t = 0:t_etapa:tF
    ii = ii + 1; k = ii + 2;

   

   if(t>=0.1010)
       u=2;
   end

   if(t>=0.7010)
       TL=0.12;
   end
    if(t>=1.001)
       TL=0;
    end

     X = modmotor(t_etapa, X, u, TL);

x1(ii)=X(1);%Omega
x2(ii)=X(2);%wp
x3(ii)=X(3);%ia
acc(ii)=u;
TL_vec(ii) = TL;


end
t = 0:t_etapa:tF;

%Grafico de simulacion
t=0:t_etapa:tF;
subplot(3,1,1);hold on;
plot(t,x1);title('Salida y, \omega_t');
subplot(3,1,2);hold on;
plot(t,acc);title('Entrada u_t, v_a');
xlabel('Tiempo [Seg.]');
subplot(3,1,3);hold on;
plot(t,TL_vec);title('Carga, TL');
xlabel('Tiempo [Seg.]');



