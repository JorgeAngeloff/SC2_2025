clear all; clc; close all;

% Definición del sistema: G(s) = (2s - 1)/(s^2 + 3s + 6)
P = 1;
num = [1 + P, -P];
den = [1, 3, 6];

% Modelo en espacio de estados (forma canónica controlable)
A = [0 1; -den(3) -den(2)];
B = [0; 1];
C = [num(2), num(1)];
D = 0;

% Ampliación del sistema con integrador del error
Aa = [A, zeros(2,1); -C, 0];
Ba = [B; 0];
Mat_Aa = Aa; Mat_Ba = Ba;

% Cálculo del controlador LQR
Q = diag([100 50 10]); R = 1e-1;

% Hamiltoniano para solución del Riccati
Ha = [Aa, -Ba * inv(R) * Ba'; -Q, -Aa'];

%%%%%%Extraemos autovalores estables:Se obtienen los autovalores de Ha, y se seleccionan los vectores propios correspondientes a autovalores con parte real negativa (i.e., estables), que son necesarios para garantizar estabilidad.
[V,D] = eig(Ha); MX1X2 = [];

for ii = 1:size(Ha,1)
    if real(D(ii,ii)) < 0
        MX1X2 = [MX1X2 V(:,ii)];
    end
end

%%%%%
%División del vector propio en dos bloques:componentes asociadas al estado (MX1) y al costo (MX2)
MX1 = MX1X2(1:end/2,:); 
MX2 = MX1X2(end/2+1:end,:);

%%%%%

%solución de la ecuación algebraica de Riccati
P = real(MX2 * inv(MX1));

%Este Ka es la ganancia óptima del controlador. En este caso, está diseñada para el sistema ampliado, así que Ka tiene dos partes: K: para los estados originales,Ki: para el estado integrador del error (acción integral).
Ka = inv(R) * Ba' * P;  % Ganancias ampliadas [K Ki]

disp('Controlador ampliado en ')
eig(Mat_Aa - Mat_Ba * Ka)

% Cálculo del observador de estados
A_d = A'; B_d = C'; C_d = B';
Qo = 1e5 * diag([1 1]); Ro = 1e1;

Ho = [A_d, -B_d * inv(Ro) * B_d'; -Qo, -A_d'];
[V,D] = eig(Ho); MX1X2 = [];

for ii = 1:size(Ho,1)
    if real(D(ii,ii)) < 0
        MX1X2 = [MX1X2 V(:,ii)];
    end
end

MX1 = MX1X2(1:end/2,:); MX2 = MX1X2(end/2+1:end,:);
Po = real(MX2 * inv(MX1));
Ko = (inv(Ro) * B_d' * Po)';  % Ganancia del observador

disp('Observador en ')
eig(A - Ko * C)

% Simulación
h = 1e-3; T = 15;
Max_T = T/h;

y(1) = 0; yp(1) = 0;
ref = 1; psi = 0; u = 0;
x_hat = [0;0];

for ii = 2:Max_T
    ua = u;
    u = -Ka * [x_hat; psi];
    psi_p = ref - y(ii-1);
    psi = psi + psi_p * h;
    
    up = (u - ua)/h;
    ypp = 2*up - 1*u - 3*yp(ii-1) - 6*y(ii-1);
    yp(ii) = yp(ii-1) + h * ypp;
    y(ii) = y(ii-1) + h * yp(ii);
    
    ys(ii-1) = y(ii);
    t(ii-1) = ii * h;
    
    x_hat_p = A * x_hat + B * u + Ko * (y(ii-1) - C * x_hat);
    x_hat = x_hat + x_hat_p * h;
    
    y_o(ii-1) = C * x_hat;
    acc(ii-1) = u;
end

% Gráficas
subplot(3,1,1); plot(t, ys, 'k'); title('Salida')
subplot(3,1,2); plot(t, ys - y_o, 'k'); title('Error de observación')
subplot(3,1,3); plot(t, acc, 'k'); title('Acción de control'); xlabel('Tiempo [seg.]')

%% PM7.4 extraído de la literatura (Dorf. Sistemas de Control Moderno 10ta
% edición. Pearson Prentice Hall 2005).
clear all;clc;close all;
% G =
%2 s - 1
%-------------
%s^2 + 3 s + 6
% pkg load signal
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
Mat_Aa=Aa;Mat_Ba=Ba;

%Cálculo del LQR-----------------------------------------------------------
Q=diag([100 50 10 ]);R=1e-1;

%Construcción del Hamiltoniano para el cálculo del controlador
Ha=[Aa -Ba*inv(R)*Ba'; -Q -Aa'];
[n,va]=size(Ha);
[V,D]=eig(Ha);MX1X2=[];

for ii=1:n
if real(D(ii,ii))<0
MX1X2=[MX1X2 V(:,ii)];
end
end

MX1=MX1X2(1:n/2,:); MX2=MX1X2(n/2+1:end,:);
% P=abs(MX2*inv(MX1));
P=real(MX2*inv(MX1));
Ka=inv(R)*Ba'*P;

%Fin cálculo del controlador-----------------------------------------------
disp('Controlador ampliado en ')
eig(Mat_Aa-Mat_Ba*Ka)

%Cálculo del Observador---------------------------------------------------
A_d=A';
B_d=C';
C_d=B';
Qo=1e5*diag([1 1]);Ro=1e1;

%Construcción del Hamiltoniano para el cálculo del observador
Ho=[A_d -B_d*inv(Ro)*B_d'; -Qo -A_d'];
[no,va]=size(Ho);
[V,D]=eig(Ho);MX1X2=[];

for ii=1:no
if real(D(ii,ii))<0
MX1X2=[MX1X2 V(:,ii)];
end
end

MX1=MX1X2(1:no/2,:); MX2=MX1X2(no/2+1:end,:);
Po=real(MX2*inv(MX1));
Ko=(inv(Ro)*B_d'*Po)';
disp('Observador en ')
eig(A-Ko*C)

%FIN cálculo del Observador------------------------------------------
%Los valores del controlador de obtienen del K ampliado


h=1e-3;
T=15;
e=0;ys=[];ypp(1)=0;yp(1)=0;y(1)=0;t=[];
Max_T=T/h;
y(1)=0;
x=[yp(1);y(1)];ref=1;psi=0;
u=0;x_hat=[0;0];y_o(1)=0;acc(1)=0;

for ii=2:Max_T
ua=u;
u=-Ka*[x_hat;psi];
psi_p=ref-y(ii-1);
psi=psi+psi_p*h;
up=(u-ua)/h;
ypp=2*up-1*u-3*yp(ii-1)-6*y(ii-1); %G de lazo abierto
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
% save('PM7_4.mat','-v7');
% save('PM7_4.mat','-v7');