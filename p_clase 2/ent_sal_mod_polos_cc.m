clear all;
% Kuo Sistemas de control automático Fig 7-18, Sección 7-5 Pp395.
%Codigo realizado por JAP
clear all;

%Se crea el sistema de 2do orden con polos cc real
% wn_r=1.5;sita_r=.85;TMax=20;
% wn_r=1.5;sita_r=.5;TMax=50;
wn_r=1.5;sita_r=.15;TMax=120;

sys_G=tf([wn_r^2],[1 2*sita_r*wn_r wn_r^2]); %ec 2do orden
sys_real_=sys_G

[y,t0]=step(sys_G,TMax/wn_r); %simula la respuesta al escalon y 
                              %devuelve los valores de salida (y) 
                              % y tiempos (t0).

step(sys_G,TMax/wn_r)         %TMax/wn_r se normaliza el tiempo para ajustarse
                              % a distintas wn
%Encontramos el primer max
[ymax a ]=max(y);
t1=t0(a);
%Encontramos el primer min
[ymin b ]=min(y(a:end));
t2=t0(b+a-1);
%Mse marcan el max(estrella) y el min(cuadrado) 
hold on
plot(t1,ymax,'pk');
plot(t2,ymin,'sk');

%Estimamos los valores del sistema de segundo orden
w_d=(2*pi)/(2*(t2-t1)); %wd? Td=(2*(t2-t1)
beta=-log(ymax-1)/pi;
sita_id=beta/(sqrt(1+beta^2));
wn=w_d/(sqrt(1-sita_id^2)); %wn?
%Se crea el sistema de 2do orden con polos cc estimado
den_id=[1 2*sita_id*wn wn^2];
sys_id=tf(wn^2,den_id)

step(sys_id)
legend('Real','Máximo','Valle','Identificada')
% figure(1);
% Nombre_Figura=['Identificacion_Simple'];
% print('-dtiff','-r600',Nombre_Figura);%--FIGURA temporal