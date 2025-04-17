%% Obtener la función de transferencia continua G(s)
G=zpk([],[0 -2],[5]);
Tm=(0.15);

%% Hallar la FT discreta de lazo abierto G_D(s) del sistema 
% de la figura con Z0H a la entrada y el tiempo de muestreo asignado Tm
Gd=c2d(G,Tm,'zoh')

%% Dibujar el mapa de polos y ceros del sistema continuo y el discreto
pzmap(G);
pzmap(Gd)

%% ¿Qué ocurre con el mapa si se multiplica por 10 el periodo de muestreo?
Gd1=c2d(G,10*Tm,'zoh');
pzmap(Gd1);
%Se desplazan el polo y el cero hacia el interior del círculo unitario
%El sistema sigue siendo estable (los polos siguen adentro del círculo 
% unitario), pero la respuesta transitoria se vuelve más lenta

%% Obtener la respuesta al escalon del sistema discreto y determinar si es estable
step(G)
step(Gd1)
%El sistema discreto posee un polo en circulo unitario, lo que
%lo vuelve marginalmente estable, la respuesta es estable pero no
% acotada, resulta una rampa creciente

%% Para el sistema discreto
%* Determinar el tipo de sistema
%* Determinar la constante de error de posición K_P y el error ante un escalon y verificar mediante
% respuesta al escalon de lazo cerrado del sistema discreto como se muestra
Kp=dcgain(Gd)
F=feedback(Gd,1)
step(F)

ess = 1 / (1 + Kp) % Error en estado estacionario

%Gd Es un sistema Tipo 1 por el polo en z=1
%Al calcular ess resulta un error cero, como se observa en la respuesta al
%escalon del sistema realimentado

%% Verificar error ante una rampa de entrada, ¿ converge o diverge? Explique la causa
t=0:Tm:100*Tm % genera rampa
lsim(F,t,t)
%La salida sigue la rampa con un error constante, 
%al ser Gd tipo 1, el polo en z=1 permite seguir 
% señales de rampa, pero con un error constante 
% proporcional a 1/Kv​

%% A lazo cerrado con realimentación unitaria
%* Graficar el lugar de raíces del sistema continuo G(s) y del discreto Gd(s) indicando las ganan-
% cias criticas de estabilidad (si las hubiera)

rlocus(G) %No posee ganancia critica, el sistema es estable para K>0 
rlocus(Gd) %Para K<5.41 Gd es estable

%% ¿ Que ocurre con la estabilidad relativa si se aumenta 10 veces el tiempo de muestreo original ?
rlocus(Gd1) %Para K<0.934 Gd1 es estable
%Al aumentar el tiempo de muestreo de Gd
% la estabilidad relativa disminuye, ahora
% con una menor ganancia el sistema se vuelve inestable