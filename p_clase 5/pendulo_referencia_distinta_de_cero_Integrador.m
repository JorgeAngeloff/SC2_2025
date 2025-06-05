clc; clear all;
m=.1;Fricc=0.1; long=0.6;g=9.8;M=.5;
%Condiciones iniciales
alfa(1)=.1; color='r';
% alfa(1)=.2; color='g';
alfa(1)=.8; color='b';
ref=10;
omega(1)=0; p_p(1)=0; u(1)=0; p(1)=0; i=1;indice=0;
%Versión linealizada en el equilibrio inestable. Sontag Pp 104.
% estado=[p(i); p_p(i); alfa(i); omega(i)]
Mat_A=[0 1 0 0;0 -Fricc/M -m*g/M 0; 0 0 0 1; 0 Fricc/(long*M) g*(m+M)/(long*M) 0]
Mat_B=[0; 1/M; 0; -1/(long*M)]
Mat_C=[1 0 0 0]; %La salida es posición
% Construcción del sistema ampliado
Mat_Aa=[Mat_A zeros(4,1);-Mat_C 0];
Mat_Ba=[Mat_B;0];
Mat_Ma=[Mat_Ba Mat_Aa*Mat_Ba Mat_Aa^2*Mat_Ba Mat_Aa^3*Mat_Ba Mat_Aa^4*Mat_Ba];%Matriz
%Controlabilidad
%Cálculo del controlador por asignación de polos
auto_val=eig(Mat_Aa);
% c_ai=conv(conv(conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]),[1 - auto_val(4)]),[1 -auto_val(5)]);
c_ai=poly(auto_val);
Mat_Wa=[c_ai(5) c_ai(4) c_ai(3) c_ai(2) 1;c_ai(4) c_ai(3) c_ai(2) 1 0;c_ai(3) c_ai(2) 1 0 0;c_ai(2) 1 0 0 0;1 0 0 0 0];
Mat_Ta=Mat_Ma*Mat_Wa;
A_controlable=inv(Mat_Ta)*Mat_Aa*Mat_Ta; %Verificación de que T esté bien
%Ubicación de los polos de lazo cerrado en mui:
mui(1)=-.7;mui(2)=-.7; mui(3)=-10.00 + 0.4i;mui(4)=conj(mui(3));mui(5)=-1;
alfa_ia=poly(mui)
Ka=fliplr(alfa_ia(2:6)-c_ai(2:6))*inv(Mat_Ta);
auto_val_LC=eig(Mat_Aa-Mat_Ba*Ka);
%Cálculo el tiempo de muestreo para el sistema estable
%auto_val_LC =
% -10.0000 + 0.4000i <---- El mas veloz (Real()). Entre 3 y 10 veces menor, el h del Euler
% -10.0000 - 0.4000i
%
-1.0000 + 0i
%
-0.7000 + 0.0000i
%
-0.7000 - 0.0000i
%ans = 0.2273 -> 1/10/10 ans = 10e-03
%ó 1/10/3------>h=33.3e-3
h=30e-3;tiempo=round(20/h);p_pp=0;tita_pp=0; t=0:h:tiempo*h;
omega=0:h:tiempo*h; alfa=0:h:tiempo*h; p=0:h:tiempo*h;
p_p=0:h:tiempo*h; u=linspace(0,0,tiempo+1);
K=Ka(1:4); KI=-Ka(5); %Los valores del controlador de obtienen del K ampliado
psi(1)=0;
while(i<(tiempo+1))
estado=[p(i); p_p(i); alfa(i); omega(i)];
psi_p=ref-Mat_C* estado;
psi(i+1)=psi(i)+psi_p*h;
u(i)=-K*estado+KI*psi(i+1);
p_pp=(1/(M+m))*(u(i)-m*long*tita_pp*cos(alfa(i))+m*long*omega(i)^2*sin(alfa(i))- Fricc*p_p(i));
tita_pp=(1/long)*(g*sin(alfa(i))-p_pp*cos(alfa(i)));
p_p(i+1)=p_p(i) + h*p_pp;
p(i+1)=p(i) + h*p_p(i) ;
omega(i+1)=omega(i)+h*tita_pp;
alfa(i+1)=alfa(i)+h*omega(i);
y_sal(i)=Mat_C*estado; i=i+1;
end
figure(1);hold on;
subplot(3,2,1);plot(t,omega,color);grid on; title('Velocidad ángulo');hold on;
subplot(3,2,2);plot(t,alfa,color);grid on;title('Ángulo');hold on;
subplot(3,2,3); plot(t,p,color);grid on;title('Posición carro');hold on;
subplot(3,2,4);plot(t,p_p,color);grid on;title('Velocidad carro');hold on;
subplot(3,1,3);plot(t,u,color);grid on;title('Acción de control');xlabel('Tiempo en Seg.');hold on;
figure(2);hold on;subplot(2,2,1);plot(alfa,omega,color);grid on;xlabel('Ángulo');ylabel('Velocidad angular');hold on;subplot(2,2,2);plot(p,p_p,color);grid on;xlabel('Posicion carro');ylabel('Velocidad carro');hold on;