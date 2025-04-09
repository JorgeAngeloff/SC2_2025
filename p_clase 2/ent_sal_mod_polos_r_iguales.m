clear all;clc;
s=tf('s');
%Se elige funcion de transferencia real de prueba
%Planta con polos iguales
% sys_G=tf(3*[10 1],conv([6 1],[6 1])); 
sys_G = tf(5*[4 1], conv([3 1],[3 1])); 

%Genera la respuesta al escalon unitario
% desde 0 a 50 segundos con muestreo cada 2 segundos.
StepAmplitude = 1;
t_s=0:2:50;
[y,t0]=step(StepAmplitude*sys_G,t_s);

%Elige puntos de muestreo, tienen que ser equidistantes
%Tiempo inicial mas corto polos iguales
%Selecciona el punto en t=2 segundos como primer punto de muestreo (t1).
t_inic=2;
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
%para polos iguales
alfa=-k1+sqrt((k1^2)+k2); 
beta=sqrt((k1^2)+k2)/(alfa*t_t1);

%Convierte los parámetros intermedios a constantes de tiempo físicas
%polos iguales
T1_ang=-t_t1/log(alfa);
T2_ang=(beta*(T1_ang^2))+T1_ang;
T1(ii)=T1_ang;
T2(ii)=T2_ang;
T2_ang=sum(T2/length(T2));
T1_ang=sum(T1/length(T1));

%Se valida el modelo identificado
%Compara la identificada con la original
sys_G_ang=tf(K*(T2*s+1)/(T1*s+1)^2) %Estimada con polos iguales
step(StepAmplitude*sys_G,'r',StepAmplitude*sys_G_ang,'k',200),hold on
legend('Real','Identificada');
legend('boxoff');