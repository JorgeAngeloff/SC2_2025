clear all;clc;close all;
% G
P=1;
num=[1+P -P];
den=[1 3 6];%
% [A,B,C,D] = tf2ss(num,den);
A(1,1)=0;A(1,2)=1;
A(2,1)=-den(3);A(2,2)=-den(2);
B(1,:)=0;B(2,:)=1;
C(1)=num(2);C(2)=num(1);D=0;
[n,d]=ss2tf(A,B,C,D)
Aa=[A,[0;0];-C,0];
Ba=[B;0];
%Cálculo del controlador por asignación de polos---------------------------
Mat_Aa=Aa;Mat_Ba=Ba;
Mat_Ma=[Mat_Ba Mat_Aa*Mat_Ba Mat_Aa^2*Mat_Ba]; %Matriz Controlabilidad
auto_val=eig(Mat_Aa);
c_ai=conv(conv([1 -auto_val(1)],[1 -auto_val(2)]),[1 -auto_val(3)]);
Mat_Wa=[c_ai(3) c_ai(2) 1 ;c_ai(2) 1 0 ;1 0 0];
Mat_Ta=Mat_Ma*Mat_Wa;
A_controlable=inv(Mat_Ta)*Mat_Aa*Mat_Ta; %Verificación de que T esté bien
%Ubicación de los polos de lazo cerrado en mui:
Polos_deseados=[-12+1j;-12-1j; -5.0];
Polos_deseados=[-.3;-5; -1];
%K=place(Aa,Ba,Polos_deseados);
mui=Polos_deseados;
alfa_ia=poly(Polos_deseados);
%alfa_ia=conv(conv([1 -mui(1)],[1 -mui(2)]),[1 -mui(3)]);
Ka=fliplr(alfa_ia(2:4)-c_ai(2:4))*inv(Mat_Ta);
%Fin cálculo del controlador------------
K=Ka;
disp('Controlador ampliado en ')
eig(Mat_Aa-Mat_Ba*Ka)
%Cálculo del Observador-----------------
A_d=A';
B_d=C';
C_d=B';
Polos_Obs=[-10;-9];
mui_d=Polos_Obs;
alfa_ia_d=poly(Polos_Obs);
Mat_M_d=[B_d A_d*B_d]; %Matriz Controlabilidad dual
auto_val_d=eig(A);
c_ai_d=poly(auto_val_d);
Mat_W=[c_ai_d(2) 1 ;1 0];
Mat_T_d=Mat_M_d*Mat_W;
Ko=(fliplr(alfa_ia_d(2:3)-c_ai_d(2:3))*inv(Mat_T_d))';
disp('Observador en ')
eig(A-Ko*C)
%FIN cálculo del Observador------------------------------------------
%lamdaR=-10, por lo tanto el tiempo asociado es TR=log(.95)/lamdaR, h puede estar
% entre TR/3 y TR/10, o sea entre 1.7e-3 y 5.1e-4
% Taomax es del Controlador, la más lenta: lamda2=-0.1765 TL=log(0.05)/lamda2 =
16.9730;%
h=4e-3;
TFinal=16.9730;
Max_T =round(TFinal/h);
e=0;ys=[];ypp(1)=0;yp(1)=0;y(1)=0;t=[];
y(1)=0;
x=[yp(1);y(1)];ref=1;psi=0;
u=0;x_hat=[0;0];y_o(1)=0;acc(1)=0;
% G =
%2 s - 1
%-------------
%s^2 + 3 s + 6
for ii=2:Max_T
ua=u;
u=-K *[x_hat;psi];
psi_p=ref-y(ii-1);
psi=psi+psi_p*h;
up=(u-ua)/h;
ypp=2*up-1*u-3*yp(ii-1)-6*y(ii-1); %G de lazo abierto%
yp(ii)=yp(ii-1)+h*ypp;
y(ii)=y(ii-1)+h*yp(ii);
ys(ii-1)=y(ii);
t(ii-1)=ii*h;
x_hat_p=A*x_hat+B*u+Ko*(y(ii-1)-C*x_hat);
x_hat=x_hat+x_hat_p*h;
y_o(ii-1)=C*x_hat;
acc(ii-1)=u;
end
subplot(3,1,1);plot(t,ys,'k');title('Salida')
subplot(3,1,2);plot(t,ys-y_o,'k');title('Error de observación')
subplot(3,1,3);plot(t,acc,'k');title('Acción de control');xlabel('Tiempo [seg.]')
% save('PM7_4.mat','-v7');%