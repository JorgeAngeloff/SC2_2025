clear all;clc;

%Se elige funcion de transferencia real de prueba
%Planta con polos distintos
% sys_G=tf(2*[8 1],conv([5 1],[6 1])); 
% sys_G=tf(16*[45 1],conv([25 1],[30 1]))
sys_G=tf(2*[8 1],([ 1 2 2]));

%Genera la respuesta al escalon unitario
% desde 0 a 50 segundos con muestreo cada 2 segundos.
StepAmplitude = 1;
t_s=0:0.1:50;
[y,t0]=step(StepAmplitude*sys_G,t_s);

%Elige puntos de muestreo, tienen que ser equidistantes
%Tiempo inicial más largo para polos distintos
%Selecciona el punto en t=4 segundos como primer punto de muestreo (t1).
t_inic=3;
[val lugar] =min(abs(t_inic-t0)); y_t1=y(lugar);
t_t1=t0(lugar);
ii=1;
[val lugar] =min(abs(2*t_inic-t0));
t_2t1=t0(lugar);
y_2t1=y(lugar);
[val lugar] =min(abs(3*t_inic-t0));
t_3t1=t0(lugar);
y_3t1=y(lugar);

%Cálculo de Parámetros Intermedios
%Calculo de ganacias
K=y(end)/StepAmplitude; %Ganancia estatica con el TVF
k1=(1/StepAmplitude)*y_t1/K-1;%Afecto el valor del Escalon, valores normalizados
k2=(1/StepAmplitude)*y_2t1/K-1;
k3=(1/StepAmplitude)*y_3t1/K-1;
%Calculo constantes de tiempo
be=4*k1^3*k3-3*k1^2*k2^2-4*k2^3+k3^2+6*k1*k2*k3; %polos distintos

%polos distintos
alfa1=(k1*k2+k3-sqrt(be))/(2*(k1^2+k2)); %depende de la cte de tiempo de p1
alfa2=(k1*k2+k3+sqrt(be))/(2*(k1^2+k2)); %depende de la cte de tiempo de p2
beta=(k1+alfa2)/(alfa1-alfa2); %depende de la cte de tiempo de z1
%beta=(2*k1^3+3*k1*k2+k3-sqrt(be))/(sqrt(be));%polos distintos
% alfa2= (k3 + k1*k2 + (4*k1^3*k3 - 3*k1^2*k2^2 + 6*k1*k2*k3 - 4*k2^3 + k3^2)^(1/2))/(2*(k1^2 + k2));
% alfa1= (k3 + k1*k2 - (4*k1^3*k3 - 3*k1^2*k2^2 + 6*k1*k2*k3 - 4*k2^3 + k3^2)^(1/2))/(2*(k1^2 + k2));

%Convierte los parámetros intermedios a constantes de tiempo físicas
%polos distintos
T1_ang=-t_t1/log(alfa1);
T2_ang=-t_t1/log(alfa2);
T3_ang=beta*(T1_ang-T2_ang)+T1_ang;
T1(ii)=T1_ang;
T2(ii)=T2_ang;
T3(ii)=T3_ang;
T3_ang=sum(T3/length(T3));
T2_ang=sum(T2/length(T2));
T1_ang=sum(T1/length(T1));

%Se valida el modelo identificado
%Compara la identificada con la original
sys_G_ang=tf(K*[T3_ang 1],conv([T1_ang 1],[T2_ang 1])) %Estimada 
step(StepAmplitude*sys_G,'r',StepAmplitude*sys_G_ang,'k',200),hold on
legend('Real','Identificada');
legend('boxoff');