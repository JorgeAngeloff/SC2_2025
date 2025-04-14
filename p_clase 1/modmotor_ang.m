function [X]=modmotor(t_etapa, xant, accion, TL)
Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;
Va=accion;
h=1e-7;
omega= xant(1);
wp= xant(2);

theta = xant(3);
ia = xant(4);

for ii=1:t_etapa/h
wpp =(-wp*(Ra*J+Laa*B)-omega*(Ra*B+Ki*Km)+Va*Ki)/(J*Laa);

iap=(-Ra*ia -Km*omega+Va)/Laa;

wp=wp+h*wpp;

wp=wp-((1/J)*TL); 

omega = omega + h*wp;

ia=ia+iap*h;

theta = theta + h*omega;
end
X=[omega,wp,theta,ia];