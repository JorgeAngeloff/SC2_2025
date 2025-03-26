clear; %close all;
X = -[0; 0; 0; 0];
ii = 0;
t_etapa = 1e-4;
wRef = 2;
thetaRef=1;

tF = 1; %tF = 0.001;
TL=1e-6;

% Constantes del PID

% Kp=.500;Ki=0.001;Kd=0.0001;color_='r';
% Kp=1;Ki=0;Kd=0.0001;color_='k';
% Kp=5;Ki=0;Kd=0;color_='b';
% Kp = 10; Ki = 500; Kd =0.001; color_ = 'r';
Kp = 60; Ki = 9090.9; Kd = 0.198; color_ = 'b';

% Regla empírica de Ziegler-Nichols:
% 
% Ajusta solo Kp hasta que el sistema oscile constantemente (sin amortiguar ni explotar).
% 
% Calcula el período de oscilación Tu​ y la ganancia crítica Ku​.
% Ku=100 T=0.0132
% Usa las reglas de Ziegler-Nichols para establecer valores iniciales:
%     Kp=0.6Ku,Ki=2Kp/Tu,Kd=KpTu/8



Ts = t_etapa;

A1 = ((2*Kp*Ts) + (Ki*(Ts^2)) + (2*Kd)) / (2*Ts);
B1 = (-2*Kp*Ts + Ki*(Ts^2) - 4*Kd) / (2*Ts);
C1 = Kd / Ts;

e = zeros(tF / t_etapa, 1); u = 0;

for t = 0:t_etapa:tF
    ii = ii + 1; k = ii + 2;

    X = modmotor_ang(t_etapa, X, u, TL);

   if(t>=0.05)
       thetaRef=2;
   end

    e(k) = thetaRef - X(3); % Error

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
u = max(min(u, 12), -12); 



x1(ii)=X(1);%Omega
x2(ii)=X(2);%wp
x3(ii)=X(3);%theta
x4(ii)=X(4);%ia

acc(ii)=u;
carga(ii)=TL;
end

%Grafico
t=0:t_etapa:tF;
subplot(3,1,1);
hold on;
plot(t,x3,'r');title('Salida theta, \theta');
xlabel('Tiempo [Seg.]');

subplot(3,1,2);hold on;
plot(t,x4,'m');title('Salida corriente de armadura, i_a');
xlabel('Tiempo [Seg.]');

subplot(3,1,3);hold on;
plot(t,acc,'b');title('Entrada u_t, v_a');
xlabel('Tiempo [Seg.]');
hold off