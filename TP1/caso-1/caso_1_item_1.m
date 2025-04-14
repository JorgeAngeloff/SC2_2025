clear; clc; close; 
  
R=220; L=500e-3; C=2.2e-6; %Valores especificados 
  
i=1; %La iteracion comienza en 1 
t=[]; %Genero t 
  %Calculo constantes de tiempo 
h=67e-6; %Con wd=932.3 r/s, h=Td/100
tf=0.05; %Si la entrada cambia cada 10ms, se toma un multiplo
         % de este para ver varios ciclos 
pasos=tf/h; 
te=10e-3;%vamos cambiando cada 1ms la entrada 
  
%Condiciones iniciales 
vc(1)=0; i1(1)=0; u(1)=12;  
vin=12; 
  
%Modelado 
A=[[-R/L -1/L] ; [1/C 0]]; 
B=[1/L ; 0]; 
C=[R  0];  
  
  
X0=[0 0]';x=[0 0]'; %Transpuestas 
  
  
while(i<=pasos) 
    t(i)=i*h; %t aumenta con i 
     
    u(i)=vin; %prearo la entrada para el cambio  
     
   %variables de estado del sistema  
   i1(i)=x(1);  
   vc(i)=x(2);  
    
   %Sistema modelado en el espacio de estados 
   xp=A*(x-X0)+B*u(i);  
   x=x+(h*xp); 
    
   if(t(i)>te)       
       vin=-vin; %Cambiamos el signo de la entrada  
       te=te+10e-3;   %cada 1mseg 
   end 
        
   i=i+1; 
  
end 
  
%Grafico 
subplot(3,1,1); plot(t,i1,'r'); title('I1,t');  
subplot(3,1,2); plot(t,vc,'b'); title('Vc,t'); 
subplot(3,1,3); plot(t,u,'g');  title('U1,t');

