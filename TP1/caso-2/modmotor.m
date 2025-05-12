function [X]=modmotor(t_etapa, xant, accion, TL)
Laa=366e-6; J=5e-9;Ra=55.6;B=0;Ki=6.49e-3;Km=6.53e-3;
%Laa = 2.43309; J = 0.00258956; Ra = 2.43309; B = 0.0594551; Ki = 0.411; Km = 0.64803;
% Laa=2.43309;
% Ki=0.411;
% J=0.00258956;
% Km=0.64803;
% B=0.0594551;
% Ra=2.43309;

Va=accion;
h=1e-7;
omega= xant(1);
wp= xant(2);
ia = xant(3);

for ii=1:t_etapa/h
wpp =(-wp*(Ra*J+Laa*B)-omega*(Ra*B+Ki*Km)+Va*Ki)/(J*Laa);
wp=wp+h*wpp;
wp=wp-((1/J)*(TL/10000000));

wp=wp-((1/J)*(TL));

iap=(-Ra*ia -Km*omega+Va)/Laa;
ia=ia+iap*h;


omega = omega + h*wp;
end
X=[omega,wp,ia];
